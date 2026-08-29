/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2025
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the implementation file of the rgbd mapping in FC-Vision.
 * License      :    GNU General Public License <http://www.gnu.org/licenses/>.
 * Project      :    FC-Vision is free software: you can redistribute it and/or 
 *                   modify it under the terms of the GNU Lesser General Public 
 *                   License as published by the Free Software Foundation, 
 *                   either version 3 of the License, or (at your option) any 
 *                   later version.
 *                   FC-Vision is distributed in the hope that it will be useful,
 *                   but WITHOUT ANY WARRANTY; without even the implied warranty 
 *                   of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 *                   See the GNU General Public License for more details.
 * Website      :    https://hkust-aerial-robotics.github.io/FC-Vision/
 *⭐⭐⭐*****************************************************************⭐⭐⭐*/

#include "rgb_point_map/rgb_point_map.h"

namespace fc_vision
{

void RGBPointMap::init(ros::NodeHandle& nh)
{
  nh.param("rgb_mapping/voxel_size", this->voxel_size_, 0.1f);
  nh.param("rgb_mapping/chunk_size", this->chunk_size_, 32);
  nh.param("rgb_mapping/origin_x",   this->origin_x_, -20.0f);
  nh.param("rgb_mapping/origin_y",   this->origin_y_, -100.0f);
  nh.param("rgb_mapping/origin_z",   this->origin_z_, 0.0f);
  nh.param("rgb_mapping/map_size_x", this->map_size_x_, 100.0f);
  nh.param("rgb_mapping/map_size_y", this->map_size_y_, 100.0f);
  nh.param("rgb_mapping/map_size_z", this->map_size_z_, 100.0f);
  nh.param("rgb_mapping/en_inherit", this->en_inherit_, false);
  nh.param("rgb_mapping/fx",         this->fx_, 0.0);
  nh.param("rgb_mapping/fy",         this->fy_, 0.0);
  nh.param("rgb_mapping/cx",         this->cx_, 0.0);
  nh.param("rgb_mapping/cy",         this->cy_, 0.0);
  nh.param("rgb_mapping/img_rows",   this->img_rows_, 480);
  nh.param("rgb_mapping/img_cols",   this->img_cols_, 640);
  nh.param("rgb_mapping/fov_scale",  this->fov_scale_, 1.0);
  nh.param("rgb_mapping/c2b_x",      this->c2b_x_, 0.0);
  nh.param("rgb_mapping/c2b_y",      this->c2b_y_, 0.0);
  nh.param("rgb_mapping/c2b_z",      this->c2b_z_, 0.0);
  nh.param("rgb_mapping/c2b_roll",   this->c2b_roll_, 0.0);
  nh.param("rgb_mapping/c2b_pitch",  this->c2b_pitch_, 0.0);
  nh.param("rgb_mapping/c2b_yaw",    this->c2b_yaw_, 0.0);
  this->voxel_num_x_ = static_cast<int>(this->map_size_x_ / this->voxel_size_);
  this->voxel_num_y_ = static_cast<int>(this->map_size_y_ / this->voxel_size_);
  this->voxel_num_z_ = static_cast<int>(this->map_size_z_ / this->voxel_size_);
  this->save_map_ = false;
  this->already_save_ = false;

  // pre-computation
  this->cam2body_R_matrix_ = Eigen::AngleAxisd(this->c2b_roll_, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(this->c2b_pitch_, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(this->c2b_yaw_, Eigen::Vector3d::UnitZ());
  this->cam2body_t_vector_ = Eigen::Vector3d(this->c2b_x_, this->c2b_y_, this->c2b_z_);

  this->img_3d_table_.resize(this->img_rows_, vector<Eigen::Vector3d>(this->img_cols_));
  for (int v = 0; v < this->img_rows_; v++)
    for (int u = 0; u < this->img_cols_; u++)
    {
      this->img_3d_table_[v][u](0) = (u - this->cx_) * this->fov_scale_ / this->fx_;
      this->img_3d_table_[v][u](1) = (v - this->cy_) * this->fov_scale_ / this->fy_;
      this->img_3d_table_[v][u](2) = this->fov_scale_;
    }
  this->fov_horizontal_ = 2.0 * atan((double)this->img_cols_ / (2.0 * this->fx_));
  this->fov_vertical_ = 2.0 * atan((double)this->img_rows_ / (2.0 * this->fy_));

  double hor = this->fov_scale_ * tan(this->fov_horizontal_ / 2.0);
  double ver = this->fov_scale_ * tan(this->fov_vertical_ / 2.0);
  Eigen::Vector3d origin(0, 0, 0);
  Eigen::Vector3d left_up(this->fov_scale_, hor, ver);
  Eigen::Vector3d left_down(this->fov_scale_, hor, -ver);
  Eigen::Vector3d right_up(this->fov_scale_, -hor, ver);
  Eigen::Vector3d right_down(this->fov_scale_, -hor, -ver);

  this->cam_vertices1_.push_back(origin);
  this->cam_vertices2_.push_back(left_up);
  this->cam_vertices1_.push_back(origin);
  this->cam_vertices2_.push_back(left_down);
  this->cam_vertices1_.push_back(origin);
  this->cam_vertices2_.push_back(right_up);
  this->cam_vertices1_.push_back(origin);
  this->cam_vertices2_.push_back(right_down);

  this->cam_vertices1_.push_back(left_up);
  this->cam_vertices2_.push_back(right_up);
  this->cam_vertices1_.push_back(right_up);
  this->cam_vertices2_.push_back(right_down);
  this->cam_vertices1_.push_back(right_down);
  this->cam_vertices2_.push_back(left_down);
  this->cam_vertices1_.push_back(left_down);
  this->cam_vertices2_.push_back(left_up);

  if (this->en_inherit_) this->loadInheritMap();

  this->start_mapping_ = false;
  this->total_rgb_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  this->incre_rgb_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  this->img_pixels_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  this->buffer_rgbd_ = boost::circular_buffer<sensor_msgs::PointCloud2>(20);
  this->buffer_rgbd_pose_ = boost::circular_buffer<nav_msgs::Odometry>(20);
  this->buffer_img_ = boost::circular_buffer<sensor_msgs::Image>(50);
  this->buffer_pose_ = boost::circular_buffer<nav_msgs::Odometry>(50);
  this->buffer_quat_ = boost::circular_buffer<geometry_msgs::QuaternionStamped>(50);
  this->buffer_cur_pose_ = boost::circular_buffer<nav_msgs::Odometry>(50);
  
  this->image_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh, "/rgb_mapping/img", 10));
  this->pose_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/rgb_mapping/body_pose", 10));
  this->gimbal_sub_.reset(new message_filters::Subscriber<geometry_msgs::QuaternionStamped>(nh, "/rgb_mapping/gimbal_pose", 10));
  this->sync_image_pose_gimbal_.reset(new message_filters::Synchronizer<SyncPolicyImagePoseGimbal>(
      SyncPolicyImagePoseGimbal(20), *this->image_sub_, *this->pose_sub_, *this->gimbal_sub_));
  this->sync_image_pose_gimbal_->registerCallback(boost::bind(&RGBPointMap::igoCallback, this, _1, _2, _3));

  this->rgbd_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/rgb_mapping/rgbd", 10));
  this->rgbd_pose_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/rgb_mapping/body_pose", 10));
  this->sync_rgbd_pose_.reset(new message_filters::Synchronizer<SyncPolicyCloudPose>(
      SyncPolicyCloudPose(20), *this->rgbd_sub_, *this->rgbd_pose_sub_));
  this->sync_rgbd_pose_->registerCallback(boost::bind(&RGBPointMap::rgbdCallback, this, _1, _2));
  
  this->demo_command_sub_ = nh.subscribe<std_msgs::UInt8>(
    "/fc_vision/demo_command", 10, &RGBPointMap::demoCommandCallback, this);
  this->odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/rgb_mapping/body_pose", 1, &RGBPointMap::odomCallback, this);

  this->map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/rgb_mapping/rgb_map", 1);
  this->incre_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/rgb_mapping/rgb_incre_map", 1);
  this->img_3d_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/rgb_mapping/img_3d", 1);
  this->fov_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/rgb_mapping/fov", 1);

  this->mapping_timer_ = nh.createTimer(ros::Duration(0.2), &RGBPointMap::mappingCallback, this);
  this->publish_map_timer_ = nh.createTimer(ros::Duration(0.2), &RGBPointMap::publishMapCallback, this);
  this->publish_cam_timer_ = nh.createTimer(ros::Duration(0.03), &RGBPointMap::publishCamCallback, this);
  this->save_timer_ = nh.createTimer(ros::Duration(0.2), &RGBPointMap::saveMapCallback, this);

  return;
}

void RGBPointMap::start()
{
  this->start_mapping_ = true;
  ROS_INFO("\033[42;37m[RGBPointMap]\033[47;32m Start Mapping! \033[0m");
  return;
}

void RGBPointMap::mappingCallback(const ros::TimerEvent& e)
{
  if (this->buffer_rgbd_.empty()) return;

  sensor_msgs::PointCloud2 msg = this->buffer_rgbd_.back();
  nav_msgs::Odometry pose_msg = this->buffer_rgbd_pose_.back();

  Eigen::Vector3d cam_pos(pose_msg.pose.pose.position.x,
                          pose_msg.pose.pose.position.y,
                          pose_msg.pose.pose.position.z); 

  pcl::PointCloud<pcl::PointXYZRGB> single_frame_rgb_cloud;
  pcl::fromROSMsg(msg, single_frame_rgb_cloud);

  this->insertPointCloudIncremental(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(single_frame_rgb_cloud), cam_pos, false);
  auto incre_filtered = this->exportIncrementalVoxelCloud();
  auto filtered = this->exportVoxelCloud();

  *this->incre_rgb_cloud_ = *incre_filtered;
  this->incre_rgb_cloud_->width = this->incre_rgb_cloud_->points.size();
  this->incre_rgb_cloud_->height = 1;
  this->incre_rgb_cloud_->is_dense = true;

  *this->total_rgb_cloud_ = *filtered;
  this->total_rgb_cloud_->width = this->total_rgb_cloud_->points.size();
  this->total_rgb_cloud_->height = 1;
  this->total_rgb_cloud_->is_dense = true;

  return;
}

void RGBPointMap::publishMapCallback(const ros::TimerEvent& e)
{
  if (!this->start_mapping_) return;
  if (this->total_rgb_cloud_ == nullptr) return;

  sensor_msgs::PointCloud2 map_msg;
  pcl::toROSMsg(*this->total_rgb_cloud_, map_msg);
  map_msg.header.frame_id = "world";
  map_msg.header.stamp = ros::Time::now();
  this->map_pub_.publish(map_msg);

  if (this->incre_rgb_cloud_ == nullptr) return;

  sensor_msgs::PointCloud2 incre_map_msg;
  pcl::toROSMsg(*this->incre_rgb_cloud_, incre_map_msg);
  incre_map_msg.header.frame_id = "world";
  incre_map_msg.header.stamp = ros::Time::now();
  this->incre_pub_.publish(incre_map_msg);

  return;
}

void RGBPointMap::publishCamCallback(const ros::TimerEvent& e)
{
  if (!this->start_mapping_) return;
  if (this->buffer_img_.empty() || this->buffer_cur_pose_.empty() || this->buffer_quat_.empty()) return;

  sensor_msgs::Image img_msg = this->buffer_img_.back();
  nav_msgs::Odometry pose_msg = this->buffer_cur_pose_.back();
  geometry_msgs::QuaternionStamped quat_msg = this->buffer_quat_.back();

  // * get transformation
  Eigen::Matrix3d sensor2world_R_matrix;
  Eigen::Vector3d sensor2world_t_vector;

  Eigen::Matrix3d img2cam_R_matrix;
  img2cam_R_matrix << 0.0, 0.0, 1.0,
                      -1.0, 0.0, 0.0,
                      0.0, -1.0, 0.0;
  Eigen::Vector3d img2cam_t_vector = Eigen::Vector3d::Zero();

  Eigen::Matrix3d cam2body_R_matrix = Eigen::Matrix3d::Identity();
  Eigen::Vector3d cam2body_t_vector = Eigen::Vector3d::Zero();

  tf::Quaternion tfQuat_body;
  tf::quaternionMsgToTF(pose_msg.pose.pose.orientation, tfQuat_body);
  tf::Matrix3x3 p_body(tfQuat_body);
  double roll_body, pitch_body, yaw_body;
  p_body.getRPY(roll_body, pitch_body, yaw_body);

  tf::Quaternion img_quat_tf(quat_msg.quaternion.x, quat_msg.quaternion.y, quat_msg.quaternion.z, quat_msg.quaternion.w);
  tf::Matrix3x3 m_body(img_quat_tf);
  double roll, pitch, yaw;
  m_body.getRPY(roll, pitch, yaw);
  roll = 0.0;
  Eigen::Matrix3d body2world_R_matrix;
  body2world_R_matrix = Eigen::AngleAxisd(yaw_body, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  Eigen::Vector3d body2world_t_vector(pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, pose_msg.pose.pose.position.z);

  sensor2world_R_matrix = body2world_R_matrix * this->cam2body_R_matrix_ * img2cam_R_matrix;
  sensor2world_t_vector = body2world_R_matrix * this->cam2body_R_matrix_ * img2cam_t_vector + body2world_R_matrix * this->cam2body_t_vector_ + body2world_t_vector;

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = sensor2world_R_matrix;
  T.block<3, 1>(0, 3) = sensor2world_t_vector;

  // * get Cam
  this->processImg(img_msg);
  pcl::transformPointCloud(*this->img_pixels_, *this->img_pixels_, T);

  sensor_msgs::PointCloud2 img_3d_msg;
  pcl::toROSMsg(*this->img_pixels_, img_3d_msg);
  img_3d_msg.header.frame_id = "world";
  img_3d_msg.header.stamp = ros::Time::now();
  this->img_3d_pub_.publish(img_3d_msg);

  Eigen::Vector3d fov_pos = sensor2world_t_vector;
  double fov_pitch = pitch;
  double fov_yaw = yaw_body;
  vector<Eigen::Vector3d> fov_list1, fov_list2;
  this->getFOV(fov_pos, fov_pitch, fov_yaw, fov_list1, fov_list2);

  visualization_msgs::MarkerArray fov_markers;

  double fov_line_scale = 0.1;
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 0;
  mk.ns = "rgb_mapping_fov";
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.color.a = 0.9;
  mk.scale.x = fov_line_scale;
  mk.scale.y = fov_line_scale;
  mk.scale.z = fov_line_scale;
  mk.pose.orientation.w = 1.0;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(fov_list1.size()); ++i) 
  {
    pt.x = fov_list1[i](0);
    pt.y = fov_list1[i](1);
    pt.z = fov_list1[i](2);
    mk.points.push_back(pt);

    pt.x = fov_list2[i](0);
    pt.y = fov_list2[i](1);
    pt.z = fov_list2[i](2);
    mk.points.push_back(pt);
  }

  fov_markers.markers.push_back(mk);
  this->fov_pub_.publish(fov_markers);

  return;
}

void RGBPointMap::saveMapCallback(const ros::TimerEvent& e)
{
  if (!this->save_map_ || this->already_save_) return;

  this->already_save_ = true;

  string map_path = ros::package::getPath("rgb_point_map") + "/inherit_map";
  string rgbd_file = map_path + "/rgbd.pcd";

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbd_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  rgbd_cloud = this->exportVoxelCloud();

  pcl::io::savePCDFileBinaryCompressed(rgbd_file, *rgbd_cloud);

  ROS_INFO("\033[42;37m[RGBPointMap]\033[47;32m Save Mapping! \033[0m");

  return;
}

}
