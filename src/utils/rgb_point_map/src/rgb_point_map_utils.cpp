/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2025
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the utils file of the rgbd mapping in FC-Vision.
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
void RGBPointMap::insertPointCloudIncremental(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, const Eigen::Vector3d& cam_pos, bool inherit)
{
  for (const auto& pt : cloud_in->points) 
  {
    if (!pcl::isFinite(pt)) continue;

    int ix = static_cast<int>(std::floor((pt.x - this->origin_x_) / this->voxel_size_));
    int iy = static_cast<int>(std::floor((pt.y - this->origin_y_) / this->voxel_size_));
    int iz = static_cast<int>(std::floor((pt.z - this->origin_z_) / this->voxel_size_));

    if (ix < 0 || ix >= voxel_num_x_ ||
        iy < 0 || iy >= voxel_num_y_ ||
        iz < 0 || iz >= voxel_num_z_) continue;

    int cx = ix / this->chunk_size_;
    int cy = iy / this->chunk_size_;
    int cz = iz / this->chunk_size_;
    int lx = ix % this->chunk_size_;
    int ly = iy % this->chunk_size_;
    int lz = iz % this->chunk_size_;
    ChunkIndex chunk_idx{cx, cy, cz};
    VoxelIndex voxel_idx{lx, ly, lz};

    chunk_map_[chunk_idx].voxels[voxel_idx].add(pt, cam_pos, inherit);
  }

  return;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RGBPointMap::exportVoxelCloud()
{
  auto cloud_out = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  for (auto& chunkpair : this->chunk_map_) 
  {
    for (auto& voxelpair : chunkpair.second.voxels) 
    {
      pcl::PointXYZRGB pt;
      if (voxelpair.second.average(pt)) 
      {
        cloud_out->points.push_back(pt);
      }
    }
  }
  cloud_out->width = cloud_out->points.size();
  cloud_out->height = 1;
  cloud_out->is_dense = true;

  return cloud_out;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RGBPointMap::exportIncrementalVoxelCloud()
{
  auto cloud_out = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  for (auto& chunkpair : this->chunk_map_) 
  {
    for (auto& voxelpair : chunkpair.second.voxels) 
    {
      if (voxelpair.second.getPublished()) continue;

      pcl::PointXYZRGB pt;
      if (voxelpair.second.average(pt)) 
      {
        cloud_out->points.push_back(pt);
        voxelpair.second.updatePublished();
      }
    }
  }
  cloud_out->width = cloud_out->points.size();
  cloud_out->height = 1;
  cloud_out->is_dense = true;

  return cloud_out;
}

void RGBPointMap::loadInheritMap()
{
  string map_path = ros::package::getPath("rgb_point_map") + "/inherit_map";
  string rgbd_file = map_path + "/rgbd.pcd";

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbd_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::io::loadPCDFile(rgbd_file, *rgbd_cloud);
  Eigen::Vector3d cam_pos(0, 0, 0);
  this->insertPointCloudIncremental(rgbd_cloud, cam_pos, true);

  return;
}

void RGBPointMap::processImg(const sensor_msgs::Image& img)
{
  bool is_rgb8 = (img.encoding == "rgb8");
  cv_bridge::CvImageConstPtr rgb_ptr = cv_bridge::toCvCopy(img, is_rgb8 ? "rgb8" : "bgr8");
  cv::Mat rgb = rgb_ptr->image;

  int rows = rgb.rows;
  int cols = rgb.cols;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  cloud->points.reserve(rows * cols);

  int skip_pixels = 5;

  for (int v = skip_pixels; v < rows - skip_pixels; v += 2)
    for (int u = skip_pixels; u < cols - skip_pixels; u += 2)
    {
      pcl::PointXYZRGB pt;
      Eigen::Vector3d pt_3d = this->img_3d_table_[v][u];
      pt.x = pt_3d(0);
      pt.y = pt_3d(1);
      pt.z = pt_3d(2);

      const cv::Vec3b& color = rgb.at<cv::Vec3b>(v, u);
      if (is_rgb8) 
      {
        pt.r = color[0];
        pt.g = color[1];
        pt.b = color[2];
      }
      else 
      {
        pt.b = color[0];
        pt.g = color[1];
        pt.r = color[2];
      }
      cloud->points.push_back(pt);
    }

  *this->img_pixels_ = *cloud;
  this->img_pixels_->width = this->img_pixels_->points.size();
  this->img_pixels_->height = 1;
  this->img_pixels_->is_dense = true;

  return;
}

void RGBPointMap::getFOV(const Eigen::Vector3d& pos, const double& pitch, const double& yaw, vector<Eigen::Vector3d>& list1, vector<Eigen::Vector3d>& list2)
{
  list1.clear();
  list2.clear();

  Eigen::Matrix3d Rwb_y, Rwb_p;
  Rwb_y << cos(yaw), -sin(yaw), 0.0, sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 1.0;
  Rwb_p << cos(pitch), 0.0, -sin(pitch), 0.0, 1.0, 0.0, sin(pitch), 0.0, cos(pitch);
  for (int i = 0; i < (int)this->cam_vertices1_.size(); ++i) 
  {
    auto p1 = Rwb_y * Rwb_p * this->cam_vertices1_[i] + pos;
    auto p2 = Rwb_y * Rwb_p * this->cam_vertices2_[i] + pos;
    list1.push_back(p1);
    list2.push_back(p2);
  }

  return;
}

void RGBPointMap::rgbdCallback(const sensor_msgs::PointCloud2ConstPtr& msg, const nav_msgs::OdometryConstPtr& pose)
{
  if (!this->start_mapping_) return;
  if (msg->width * msg->height == 0 || msg->data.empty() || pose == nullptr) return;

  this->buffer_rgbd_.push_back(*msg);
  this->buffer_rgbd_pose_.push_back(*pose);

  return;
}

void RGBPointMap::igoCallback(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& pose, const geometry_msgs::QuaternionStampedConstPtr& quat)
{
  if (!this->start_mapping_) return;
  if (img == nullptr || pose == nullptr || quat == nullptr) return;

  this->buffer_img_.push_back(*img);
  this->buffer_pose_.push_back(*pose);
  this->buffer_quat_.push_back(*quat);

  return;
}

void RGBPointMap::odomCallback(const nav_msgs::OdometryConstPtr& pose)
{
  if (!this->start_mapping_) return;
  if (pose == nullptr) return;
  
  this->buffer_cur_pose_.push_back(*pose);

  return;
}

void RGBPointMap::demoCommandCallback(const std_msgs::UInt8::ConstPtr& command)
{
  if (command && command->data == 1) this->start_mapping_ = true;
}

}
