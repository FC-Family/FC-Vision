#ifndef FC_VISION_MANAGER_NODE_H
#define FC_VISION_MANAGER_NODE_H

#include "active_perception/perception_utils.h"
#include "path_searching/geometry_utils.hpp"
#include "path_searching/path_tools.hpp"
#include "plan_env/sdf_map.h"
#include "visibility_replan/replan_fsm.h"

#include <geometry_msgs/QuaternionStamped.h>
#include <igl/read_triangle_mesh.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <yaml-cpp/yaml.h>

#include <boost/circular_buffer.hpp>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace fc_vision
{

using SyncPolicyPoseGimbal = message_filters::sync_policies::ApproximateTime<
    nav_msgs::Odometry, geometry_msgs::QuaternionStamped>;
using SynchronizerPoseGimbal = message_filters::Synchronizer<SyncPolicyPoseGimbal>;

class ManagerNode
{
public:
  void init(ros::NodeHandle& nh);
  void shutdown();

private:
  void mapCallback(const ros::TimerEvent& event);
  void pathCallback(const ros::TimerEvent& event);
  void replanCallback(const ros::TimerEvent& event);
  void loadRegistration();
  void updateMap();
  void userObsRegion();
  void triggerStage(uint8_t command);
  bool getCurRobotPose();

  void demoCommandCallback(const std_msgs::UInt8::ConstPtr& command);
  void poseGimbalCallback(
      const nav_msgs::OdometryConstPtr& pose,
      const geometry_msgs::QuaternionStampedConstPtr& gimbal);

  double resolution_ = 0.0;
  double user_path_interval_ = 0.0;
  double vis_inf_ = 0.0;
  std::string reg_T_;
  std::string path_file_;
  std::string target_mesh_file_;
  bool map_trigger_ = false;
  bool map_ready_ = false;
  bool path_trigger_ = false;
  bool path_ready_ = false;
  bool replan_trigger_ = false;
  bool replan_ready_ = false;

  Eigen::MatrixXd target_vertices_;
  Eigen::MatrixXi target_faces_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_target_cloud_;
  std::vector<Eigen::VectorXd> global_path_;
  std::vector<bool> global_indicators_;
  Eigen::VectorXd current_robot_pose_;

  std::unique_ptr<PerceptionUtils> perception_utils_;
  std::shared_ptr<SDFMap> map_;
  std::unique_ptr<ReplanFSM> replan_fsm_;

  ros::Timer map_timer_;
  ros::Timer path_timer_;
  ros::Timer replan_timer_;
  ros::Subscriber demo_command_sub_;
  std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> pose_sub_;
  std::unique_ptr<message_filters::Subscriber<geometry_msgs::QuaternionStamped>> gimbal_sub_;
  std::unique_ptr<SynchronizerPoseGimbal> pose_gimbal_sync_;
  boost::circular_buffer<nav_msgs::Odometry> pose_buffer_;
  boost::circular_buffer<geometry_msgs::QuaternionStamped> gimbal_buffer_;
};

}  // namespace fc_vision

#endif
