#include "fc_vision_manager/manager_node.h"

#include <tf/tf.h>

#include <cmath>

namespace fc_vision
{

void ManagerNode::loadRegistration()
{
  Eigen::Matrix4d transform;
  const std::string filename = ros::package::getPath("fc_vision_manager") +
      "/config/reg_cfg/" + reg_T_;
  const YAML::Node config = YAML::LoadFile(filename);
  int row = 0;
  for (const auto& values : config["T_matrix"])
  {
    transform.row(row++) << values[0].as<double>(), values[1].as<double>(),
        values[2].as<double>(), values[3].as<double>();
  }

  aligned_target_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*target_cloud_, *aligned_target_cloud_, transform);

  Eigen::MatrixXd homogeneous_vertices(target_vertices_.rows(), 4);
  homogeneous_vertices << target_vertices_, Eigen::VectorXd::Ones(target_vertices_.rows());
  target_vertices_ = (transform * homogeneous_vertices.transpose()).transpose().leftCols(3);
  ROS_INFO("\033[31m[FC-Vision][Registration] Finished! \033[32m");
}

void ManagerNode::updateMap()
{
  map_->resetHCMap(aligned_target_cloud_);
  map_->updateESDF3dHCMap();
  map_->updateMapAttributes();
  ROS_INFO("\033[31m[FC-Vision][MapUpdate] Finished! \033[32m");
}

void ManagerNode::userObsRegion()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr observed(new pcl::PointCloud<pcl::PointXYZ>);
  for (const pcl::PointXYZ& point : aligned_target_cloud_->points)
  {
    const Eigen::Vector3d target(point.x, point.y, point.z);
    for (size_t i = 0; i < global_path_.size(); ++i)
    {
      if (global_indicators_[i]) continue;
      const Eigen::VectorXd& viewpoint = global_path_[i];
      perception_utils_->setPose_PY(viewpoint.head(3), viewpoint(3), viewpoint(4));
      if (!perception_utils_->insideFOV(target)) continue;

      bool visible = true;
      const std::vector<Eigen::Vector3d> samples = visibility_utils::sampleLine(
          viewpoint.head(3), target, vis_inf_, map_->getResolution());
      for (const Eigen::Vector3d& sample : samples)
      {
        if (std::abs(map_->getDistance_hc(sample)) < 0.2)
        {
          visible = false;
          break;
        }
      }
      if (visible)
      {
        observed->push_back(point);
        break;
      }
    }
  }
  *aligned_target_cloud_ = *observed;
  map_->inputUserObs(observed);
}

void ManagerNode::triggerStage(uint8_t command)
{
  if (command == 1) map_trigger_ = true;
  if (command == 2) path_trigger_ = true;
  if (command == 3) replan_trigger_ = true;
  if (command == 4 && replan_ready_) replan_fsm_->triggerLocalPlan();
}

void ManagerNode::demoCommandCallback(const std_msgs::UInt8::ConstPtr& command)
{
  if (command) triggerStage(command->data);
}

void ManagerNode::poseGimbalCallback(
    const nav_msgs::OdometryConstPtr& pose,
    const geometry_msgs::QuaternionStampedConstPtr& gimbal)
{
  pose_buffer_.push_back(*pose);
  gimbal_buffer_.push_back(*gimbal);
}

bool ManagerNode::getCurRobotPose()
{
  if (pose_buffer_.empty() || gimbal_buffer_.empty()) return false;

  const nav_msgs::Odometry& pose = pose_buffer_.back();
  const geometry_msgs::QuaternionStamped& gimbal = gimbal_buffer_.back();
  tf::Quaternion gimbal_quaternion;
  tf::quaternionMsgToTF(gimbal.quaternion, gimbal_quaternion);
  double gimbal_roll, gimbal_pitch, gimbal_yaw;
  tf::Matrix3x3(gimbal_quaternion).getRPY(gimbal_roll, gimbal_pitch, gimbal_yaw);

  tf::Quaternion body_quaternion;
  tf::quaternionMsgToTF(pose.pose.pose.orientation, body_quaternion);
  double body_roll, body_pitch, body_yaw;
  tf::Matrix3x3(body_quaternion).getRPY(body_roll, body_pitch, body_yaw);

  current_robot_pose_.resize(5);
  current_robot_pose_ << pose.pose.pose.position.x, pose.pose.pose.position.y,
      pose.pose.pose.position.z, gimbal_pitch + body_pitch, body_yaw;
  return true;
}

}  // namespace fc_vision
