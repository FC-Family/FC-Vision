#include "fc_vision_manager/manager_node.h"

namespace fc_vision
{

void ManagerNode::init(ros::NodeHandle& nh)
{
  perception_utils_.reset(new PerceptionUtils);
  perception_utils_->init(nh);
  map_.reset(new SDFMap);
  map_->setXYoffset(0.0, 0.0);
  map_->initMap(nh);
  map_->initHC(nh);
  replan_fsm_.reset(new ReplanFSM);
  replan_fsm_->init(nh);

  nh.param("manager_node/target_mesh_file", target_mesh_file_, std::string(""));
  nh.param("manager_node/resolution", resolution_, -1.0);
  nh.param("manager_node/reg_T_file", reg_T_, std::string(""));
  nh.param("manager_node/path_file", path_file_, std::string(""));
  nh.param("manager_node/user_p_interval", user_path_interval_, 1.0);
  nh.param("manager_node/vis_inf", vis_inf_, 0.4);

  demo_command_sub_ = nh.subscribe<std_msgs::UInt8>(
      "/fc_vision/demo_command", 10, &ManagerNode::demoCommandCallback, this);
  map_timer_ = nh.createTimer(ros::Duration(1.0), &ManagerNode::mapCallback, this);
  path_timer_ = nh.createTimer(ros::Duration(1.0), &ManagerNode::pathCallback, this);
  replan_timer_ = nh.createTimer(ros::Duration(1.0), &ManagerNode::replanCallback, this);

  if (!igl::read_triangle_mesh(target_mesh_file_, target_vertices_, target_faces_))
  {
    throw std::runtime_error("Failed to load target mesh: " + target_mesh_file_);
  }
  target_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  visibility_utils::meshToPointCloud(target_vertices_, target_faces_, resolution_, target_cloud_);

  pose_buffer_ = boost::circular_buffer<nav_msgs::Odometry>(20);
  gimbal_buffer_ = boost::circular_buffer<geometry_msgs::QuaternionStamped>(20);
  pose_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(
      nh, "/manager_node/odometry", 10));
  gimbal_sub_.reset(new message_filters::Subscriber<geometry_msgs::QuaternionStamped>(
      nh, "/manager_node/gimbal", 10));
  pose_gimbal_sync_.reset(new SynchronizerPoseGimbal(
      SyncPolicyPoseGimbal(20), *pose_sub_, *gimbal_sub_));
  pose_gimbal_sync_->registerCallback(
      boost::bind(&ManagerNode::poseGimbalCallback, this, _1, _2));

  ROS_INFO("\033[42;37m[FC-Vision]\033[47;32m Initialized! \033[0m");
}

void ManagerNode::mapCallback(const ros::TimerEvent&)
{
  if (!map_trigger_ || map_ready_) return;
  loadRegistration();
  updateMap();
  map_ready_ = true;
  map_timer_.stop();
}

void ManagerNode::pathCallback(const ros::TimerEvent&)
{
  if (!path_trigger_ || path_ready_ || !map_ready_) return;
  if (!getCurRobotPose()) return;

  std::vector<Eigen::VectorXd> configured_path = {current_robot_pose_};
  std::vector<bool> configured_indicators = {false};
  const std::string filename = ros::package::getPath("fc_vision_manager") +
      "/config/path_cfg/" + path_file_;
  const YAML::Node config = YAML::LoadFile(filename);
  for (const auto& path : config["path"])
  {
    for (const auto& point : path["waypoints"])
    {
      Eigen::VectorXd waypoint(5);
      waypoint << point[0].as<double>(), point[1].as<double>(), point[2].as<double>(),
          point[3].as<double>() * M_PI / 180.0,
          point[4].as<double>() * M_PI / 180.0;
      configured_path.push_back(waypoint);
      configured_indicators.push_back(
          point.size() == 5 ? false : point[5].as<int>() != 0);
    }
  }

  global_path_.clear();
  global_indicators_.clear();
  global_path_.push_back(configured_path.front());
  global_indicators_.push_back(configured_indicators.front());
  for (size_t i = 1; i < configured_path.size(); ++i)
  {
    std::vector<Eigen::VectorXd> segment;
    path_tools::pieceInterpolate(
        configured_path[i - 1], configured_path[i], user_path_interval_, segment);
    global_path_.insert(global_path_.end(), segment.begin(), segment.end());
    global_indicators_.insert(global_indicators_.end(), segment.size(), true);
    global_path_.push_back(configured_path[i]);
    global_indicators_.push_back(configured_indicators[i]);
  }

  userObsRegion();
  path_ready_ = true;
  path_timer_.stop();
  ROS_INFO("\033[31m[FC-Vision][UserDefine] Finished! \033[32m");
}

void ManagerNode::replanCallback(const ros::TimerEvent&)
{
  if (!replan_trigger_ || replan_ready_ || !path_ready_) return;
  replan_fsm_->setMap(map_);
  replan_fsm_->setGlobalPlan(global_path_, global_indicators_);
  replan_fsm_->startService();
  replan_ready_ = true;
  replan_timer_.stop();
  ROS_INFO("\033[31m[FC-Vision][Replan] Activated! \033[32m");
}

void ManagerNode::shutdown()
{
  map_timer_.stop();
  path_timer_.stop();
  replan_timer_.stop();
  if (replan_fsm_) replan_fsm_->stopService();
}

}  // namespace fc_vision
