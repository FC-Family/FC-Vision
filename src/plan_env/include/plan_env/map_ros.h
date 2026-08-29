/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Sept. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the header file of bridge between mapping and ROS.
 * License      :    GNU General Public License <http://www.gnu.org/licenses/>.
 * Project      :    FC-Planner is free software: you can redistribute it and/or
 *                   modify it under the terms of the GNU Lesser General Public
 *                   License as published by the Free Software Foundation,
 *                   either version 3 of the License, or (at your option) any
 *                   later version.
 *                   FC-Planner is distributed in the hope that it will be useful,
 *                   but WITHOUT ANY WARRANTY; without even the implied warranty
 *                   of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *                   See the GNU General Public License for more details.
 * Website      :    https://hkust-aerial-robotics.github.io/FC-Planner/
 *⭐⭐⭐*****************************************************************⭐⭐⭐*/

#ifndef _MAP_ROS_H
#define _MAP_ROS_H

#include "plan_env/sdf_map.h"
#include "plan_env/raycast.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <memory>
#include <random>
#include <string>
#include <vector>
#include <cmath>
#include <tf/tf.h>
#include <boost/circular_buffer.hpp>

using std::string;
using std::vector;
using std::list;
using std::shared_ptr;
using std::unique_ptr;

class RayCaster;

namespace fc_vision {
class SDFMap;

class MapROS {
public:
  MapROS(){
  };
  ~MapROS(){
  };
  void setMap(SDFMap* map);
  void init();

private:
  void depthPoseQuatCallback(const sensor_msgs::ImageConstPtr& img,
                             const nav_msgs::OdometryConstPtr& pose,
                             const geometry_msgs::QuaternionStampedConstPtr& quat);
  void depthPoseCallback(const sensor_msgs::ImageConstPtr& img,
                         const nav_msgs::OdometryConstPtr& pose);
  void cloudPoseCallback(const sensor_msgs::PointCloud2ConstPtr& cloud,
                         const nav_msgs::OdometryConstPtr& pose);
  void cloudPoseTrackCallback(const sensor_msgs::PointCloud2ConstPtr& cloud,
                              const nav_msgs::OdometryConstPtr& pose);
  void tarEnvPoseCallback(const sensor_msgs::PointCloud2ConstPtr& target,
                          const sensor_msgs::PointCloud2ConstPtr& env,
                          const nav_msgs::OdometryConstPtr& pose);
  void updateESDFCallback(const ros::TimerEvent& e);
  void visCallback(const ros::TimerEvent& e);
  void partialCallback(const ros::TimerEvent& e);
  void partialDataCallback(const ros::TimerEvent& e);
  void saveInheritMapCallback(const ros::TimerEvent& e);
  // -----------------------
  void publishMapAll();
  void publishMapLocal();
  void publishESDF();
  void publishMappingRange();
  void publishUpdateRange();
  void publishDepth();
  void publishTarget();
  void publishEnvironment();
  // -----------------------
  void proessDepthImage();
  pcl::PointCloud<pcl::PointXYZ> generateAirPointCloud(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, const Eigen::Matrix3d& R_sensor2world, const Eigen::Vector3d& t_sensor2world);

  SDFMap* map_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry, geometry_msgs::QuaternionStamped>
      SyncPolicyImagePoseQuat;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePoseQuat>> SynchronizerImagePoseQuat;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry> SyncPolicyImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry>
      SyncPolicyCloudPose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyCloudPose>> SynchronizerCloudPose;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, nav_msgs::Odometry>
      SyncPolicyTarEnvPose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyTarEnvPose>> SynchronizerTarEnvPose;

  ros::NodeHandle node_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> pose_sub_;
  shared_ptr<message_filters::Subscriber<geometry_msgs::QuaternionStamped>> quat_sub_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> tracking_cloud_sub_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> tracking_target_sub_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> tracking_env_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> tracking_pose_sub_;
  SynchronizerImagePoseQuat sync_image_pose_quat_;
  SynchronizerImagePose sync_image_pose_;
  SynchronizerCloudPose sync_cloud_pose_;
  SynchronizerCloudPose sync_cloud_pose_track_;
  SynchronizerTarEnvPose sync_tar_env_pose_;
  
  // ROS Service
  ros::Publisher map_local_pub_, map_local_inflate_pub_, esdf_pub_, map_all_pub_, map_all_inflate_pub_, map_unknown_pub_, map_free_pub_, mapping_range_pub_, update_range_pub_, depth_pub_, target_pub_, env_pub_, hcmap_occ_pub_, hcmap_inf_pub_, hcmap_esdf_pub_, hcmap_obs_pub_;
  ros::Timer esdf_timer_, vis_timer_, partial_timer_, partial_data_timer_, inherit_timer_;
  ros::Publisher partial_pub_;

  unique_ptr<ros::Rate> mapping_rate_ = nullptr;

  ros::Timer update_map_timer_;

  // type of input
  bool depth_pose_input_ = false;
  bool cloud_pose_input_ = false;
  bool depth_gimbal_pose_input_ = false;
  bool cloud_tar_env_pose_input_ = false;

  // ! Depth & Pose & Gimbal
  boost::circular_buffer<sensor_msgs::Image> buffer_depth_dpg_;
  boost::circular_buffer<nav_msgs::Odometry> buffer_pose_dpg_;
  boost::circular_buffer<geometry_msgs::QuaternionStamped> buffer_quat_dpg_;
  void updateMapDepthPoseQuat(const ros::TimerEvent& e);

  // ! Depth & Pose
  boost::circular_buffer<sensor_msgs::Image> buffer_depth_dp_;
  boost::circular_buffer<nav_msgs::Odometry> buffer_pose_dp_;
  void updateMapDepthPose(const ros::TimerEvent& e);

  // ! Cloud & Pose
  boost::circular_buffer<sensor_msgs::PointCloud2> buffer_cloud_cp_;
  boost::circular_buffer<nav_msgs::Odometry> buffer_pose_cp_;
  void updateMapCloudPose(const ros::TimerEvent& e);

  // ! Cloud & Target & Environment & Pose
  boost::circular_buffer<sensor_msgs::PointCloud2> buffer_all_ctep_;
  boost::circular_buffer<nav_msgs::Odometry> buffer_pose_track_;
  boost::circular_buffer<sensor_msgs::PointCloud2> buffer_target_ctep_;
  boost::circular_buffer<sensor_msgs::PointCloud2> buffer_env_ctep_;
  boost::circular_buffer<nav_msgs::Odometry> buffer_pose_ctep_;
  void updateMapCloudTarEnvPose(const ros::TimerEvent& e);

  // output
  boost::circular_buffer<pcl::PointCloud<pcl::PointNormal>> buffer_partial_;

  // params, depth projection
  Eigen::Matrix3d K_depth_;
  double cx_, cy_, fx_, fy_;
  double depth_filter_maxdist_, depth_filter_mindist_;
  int depth_filter_margin_;
  double k_depth_scaling_factor_;
  int skip_pixel_;
  string frame_id_;
  bool airsim_flag_;
  double mapping_fps_, partial_time_;
  // params, lidar air projection
  double h_fov_, v_fov_min_, v_fov_max_, inf_fov_;
  double h_res_, v_res_;
  double mapping_visible_dist_;
  int dil_pixels_;
  // msg publication
  double esdf_slice_height_;
  double visualization_truncate_height_, visualization_truncate_low_;
  bool show_esdf_time_, show_occ_time_;
  bool show_all_map_, show_local_map_;
  bool show_depth_, show_local_range_;
  bool show_target_, show_env_;
  bool inflate_vis_;

  // Depth Extrinsic Parameters
  double dep_x_, dep_y_, dep_z_;
  double dep_roll_, dep_pitch_, dep_yaw_;
  Eigen::Vector3d d2body_t_vector_;
  Eigen::Matrix3d d2body_R_matrix_;

  // data
  // flags of map state
  bool local_updated_, esdf_need_update_;
  // input
  Eigen::Vector3d camera_pos_;
  Eigen::Quaterniond camera_q_;
  Eigen::Quaterniond gimbal_q_;
  unique_ptr<cv::Mat> depth_image_;
  vector<Eigen::Vector3d> proj_points_;
  int proj_points_cnt;
  double fuse_time_, esdf_time_, max_fuse_time_, max_esdf_time_;
  int fuse_num_, esdf_num_;
  pcl::PointCloud<pcl::PointXYZ> point_cloud_;

  ros::Time map_start_time_;
  geometry_msgs::QuaternionStamped q_gimbal;

  unique_ptr<RayCaster> raycaster_;

  // inheritance map
  bool en_inherit_, already_inherit_;

  friend SDFMap;
};
}

#endif
