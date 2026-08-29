/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Sept. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the main file of dual volumetric mapping module, which
 *                   is mainly modified from FC-Planner (https://github.com/HKUST-Aerial-Robotics/FC-Planner).
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

#include "plan_env/sdf_map.h"

namespace fc_vision {
SDFMap::SDFMap() {
}

SDFMap::~SDFMap() {
}

void SDFMap::initMap(ros::NodeHandle& nh) 
{
  mp_.reset(new MapParam);
  md_.reset(new MapData);
  mr_.reset(new MapROS);

  // Params of map properties
  double x_size, y_size, z_size;
  nh.param("sdf_map/resolution", mp_->resolution_, -1.0);
  nh.param("sdf_map/map_size_x", x_size, -1.0);
  nh.param("sdf_map/map_size_y", y_size, -1.0);
  nh.param("sdf_map/map_size_z", z_size, -1.0);
  nh.param("sdf_map/map_back_x", mp_->x_origin_back_, -1.0);
  nh.param("sdf_map/map_back_y", mp_->y_origin_back_, -1.0);
  nh.param("sdf_map/obstacles_inflation", mp_->obstacles_inflation_, -1.0);
  nh.param("sdf_map/target_obs_inf", mp_->target_obstacles_inflation_, -1.0);
  nh.param("sdf_map/obstacles_inf_z", mp_->obs_inf_z_, 0.6);
  nh.param("sdf_map/target_obs_inf_z", mp_->target_obs_inf_z_, 1.2);
  nh.param("sdf_map/local_bound_inflate", mp_->local_bound_inflate_, 1.0);
  nh.param("sdf_map/local_map_margin", mp_->local_map_margin_, 1);
  nh.param("sdf_map/ground_height", mp_->ground_height_, -1.0);
  nh.param("sdf_map/default_dist", mp_->default_dist_, 5.0);
  nh.param("sdf_map/optimistic", mp_->optimistic_, true);
  nh.param("sdf_map/signed_dist", mp_->signed_dist_, false);
  nh.param("sdf_map/airsim_flag", this->airsim_flag_, false);
  nh.param("sdf_map/enable_inherit", this->en_inherit_, false);
  this->en_attributes_ = false;

  if (this->en_inherit_) loadInheritOffset();

  mp_->local_bound_inflate_ = max(mp_->resolution_, mp_->local_bound_inflate_);
  mp_->resolution_inv_ = 1 / mp_->resolution_;
  mp_->map_origin_ = Eigen::Vector3d(this->cur_x_+mp_->x_origin_back_, this->cur_y_ + mp_->y_origin_back_, mp_->ground_height_);
  mp_->map_size_ = Eigen::Vector3d(x_size, y_size, z_size);
  for (int i = 0; i < 3; ++i)
    mp_->map_voxel_num_(i) = ceil(mp_->map_size_(i) / mp_->resolution_);
  mp_->map_min_boundary_ = mp_->map_origin_;
  mp_->map_max_boundary_ = mp_->map_origin_ + mp_->map_size_;
  min_bound = mp_->map_min_boundary_;
  max_bound = mp_->map_max_boundary_;

  // Params of raycasting-based fusion
  nh.param("sdf_map/p_hit", mp_->p_hit_, 0.70);
  nh.param("sdf_map/p_miss", mp_->p_miss_, 0.35);
  nh.param("sdf_map/p_min", mp_->p_min_, 0.12);
  nh.param("sdf_map/p_max", mp_->p_max_, 0.97);
  nh.param("sdf_map/p_occ", mp_->p_occ_, 0.80);
  nh.param("sdf_map/max_ray_length", mp_->max_ray_length_, -0.1);
  nh.param("sdf_map/virtual_ceil_height", mp_->virtual_ceil_height_, -0.1);

  auto logit = [](const double& x) { return log(x / (1 - x)); };
  mp_->prob_hit_log_ = logit(mp_->p_hit_);
  mp_->prob_miss_log_ = logit(mp_->p_miss_);
  mp_->clamp_min_log_ = logit(mp_->p_min_);
  mp_->clamp_max_log_ = logit(mp_->p_max_);
  mp_->min_occupancy_log_ = logit(mp_->p_occ_);
  mp_->unknown_flag_ = 0.01;

  // * Initialize map buffer
  int buffer_size = mp_->map_voxel_num_(0) * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2);
  // * Occupancy buffer
  md_->occupancy_buffer_ = vector<double>(buffer_size, mp_->clamp_min_log_ - mp_->unknown_flag_);
  md_->occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
  md_->count_hit_ = vector<short>(buffer_size, 0);
  md_->count_miss_ = vector<short>(buffer_size, 0);
  md_->flag_rayend_ = vector<char>(buffer_size, -1);
  // * target & environment flags buffer
  md_->target_buffer_ = vector<char>(buffer_size, 0);
  md_->env_buffer_ = vector<char>(buffer_size, 0);
  // * ESDF buffer
  if (mp_->signed_dist_) md_->distance_buffer_neg_ = vector<double>(buffer_size, mp_->default_dist_);
  md_->distance_buffer_ = vector<double>(buffer_size, mp_->default_dist_);
  md_->tmp_buffer1_ = vector<double>(buffer_size, 0);
  md_->tmp_buffer2_ = vector<double>(buffer_size, 0);
  md_->raycast_num_ = 0;
  md_->reset_updated_box_ = true;
  md_->update_min_ = md_->update_max_ = Eigen::Vector3d(0, 0, 0);

  // Try retriving bounding box of map, set box to map size if not specified
  vector<string> axis = { "x", "y", "z" };
  for (int i = 0; i < 3; ++i) {
    nh.param("sdf_map/box_min_" + axis[i], mp_->box_mind_[i], mp_->map_min_boundary_[i]);
    nh.param("sdf_map/box_max_" + axis[i], mp_->box_maxd_[i], mp_->map_max_boundary_[i]);
  }
  posToIndex(mp_->box_mind_, mp_->box_min_);
  posToIndex(mp_->box_maxd_, mp_->box_max_);

  // Inheritance Map
  if (this->en_inherit_) loadInheritMap();

  // Initialize ROS wrapper
  mr_->setMap(this);
  mr_->node_ = nh;
  mr_->init();

  caster_.reset(new RayCaster);
  caster_->setParams(mp_->resolution_, mp_->map_origin_);

  ROS_INFO("\033[33m[Real-world Map in FC-Vision] Initialized! \033[32m");

  return;
}

void SDFMap::setXYoffset(double x_offset, double y_offset)
{
  this->cur_x_ = x_offset;
  this->cur_y_ = y_offset;

  return;
}

void SDFMap::getXYoffset(double& x_offset, double& y_offset)
{
  x_offset = this->cur_x_;
  y_offset = this->cur_y_;

  return;
}

void SDFMap::initHC(ros::NodeHandle& nh)
{
  hcmp_.reset(new HCMapParam);
  hcmd_.reset(new HCMapData);

  nh.param("hcmap/resolution", hcmp_->resolution_, -1.0);
  nh.param("hcmap/interval", hcmp_->proj_interval, -1.0);
  nh.param("hcmap/plane_thickness", hcmp_->thickness, -1.0);
  nh.param("hcmap/inflateVoxel", inflate_num, -1);
  nh.param("hcmap/visible_range", hcmp_->size_inflate, -1.0);
  nh.param("hcmap/ground_enabled", zFlag, false);
  nh.param("hcmap/ground_height", zPos, -1.0);
  nh.param("sdf_map/ground_height", hcmp_->z_min_, -1.0);
  nh.param("sdf_map/map_size_x", hcmp_->x_size_, -1.0);
  nh.param("sdf_map/map_size_y", hcmp_->y_size_, -1.0);
  nh.param("sdf_map/map_size_z", hcmp_->z_size_, -1.0);

  ROS_INFO("\033[33m[Target Map in FC-Vision] Initialized! \033[32m");

  return;
}

void SDFMap::initHCMap(ros::NodeHandle& nh, pcl::PointCloud<pcl::PointXYZ>::Ptr& model)
{
  hcmp_.reset(new HCMapParam);
  hcmd_.reset(new HCMapData);

  nh.param("hcmap/resolution", hcmp_->resolution_, -1.0);
  nh.param("hcmap/interval", hcmp_->proj_interval, -1.0);
  nh.param("hcmap/plane_thickness", hcmp_->thickness, -1.0);
  nh.param("hcmap/inflateVoxel", inflate_num, -1);
  nh.param("hcmap/visible_range", hcmp_->size_inflate, -1.0);
  nh.param("hcmap/ground_enabled", zFlag, false);
  nh.param("hcmap/ground_height", zPos, -1.0);
  
  /* find scene size */
  pcl::PointXYZ pt_min;
  pcl::PointXYZ pt_max;
  pcl::getMinMax3D(*model,pt_min,pt_max);
  // ROS_INFO("Map Minimum point: x=%f, y=%f, z=%f", pt_min.x, pt_min.y, pt_min.z);
  // ROS_INFO("Map Maximum point: x=%f, y=%f, z=%f", pt_max.x, pt_max.y, pt_max.z);
  double x_size, y_size, z_size;
  x_size = 4*hcmp_->size_inflate+pt_max.x - pt_min.x;
  y_size = 4*hcmp_->size_inflate+pt_max.y - pt_min.y;
  z_size = 4*hcmp_->size_inflate+pt_max.z - pt_min.z;

  hcmp_->resolution_inv_ = 1 / hcmp_->resolution_;
  hcmp_->map_origin_ = Eigen::Vector3d(pt_min.x-2*hcmp_->size_inflate, pt_min.y-2*hcmp_->size_inflate, pt_min.z-2*hcmp_->size_inflate);
  posToIndex_hc(hcmp_->map_origin_, hcmp_->map_origin_idx_);
  hcmp_->map_size_ = Eigen::Vector3d(x_size, y_size, z_size);
  for (int i = 0; i < 3; ++i)
    hcmp_->map_voxel_num_(i) = ceil(hcmp_->map_size_(i) / hcmp_->resolution_);
  hcmp_->map_min_boundary_ = hcmp_->map_origin_;
  hcmp_->map_max_boundary_ = hcmp_->map_origin_ + hcmp_->map_size_;
  // if (zFlag == true)
  // {
  //   hcmp_->map_min_boundary_(2) = zPos;
  // }
  hcmp_->box_mind_ = hcmp_->map_min_boundary_;
  hcmp_->box_maxd_ = hcmp_->map_max_boundary_;

  posToIndex_hc(hcmp_->box_mind_, hcmp_->box_min_);
  posToIndex_hc(hcmp_->box_maxd_, hcmp_->box_max_);

  int buffer_size = hcmp_->map_voxel_num_(0) * hcmp_->map_voxel_num_(1) * hcmp_->map_voxel_num_(2);
  hcmd_->occupancy_buffer_hc_ = vector<char>(buffer_size, 0);
  hcmd_->occupancy_inflate_buffer_hc_ = vector<char>(buffer_size, 0);
  hcmd_->occupancy_buffer_internal_ = vector<char>(buffer_size, 0);

  internal_cast_.reset(new RayCaster);
  internal_cast_->setParams(hcmp_->resolution_, hcmp_->map_origin_);

  for (auto occ:model->points)
  {
    Eigen::Vector3d pt_occ;
    Eigen::Vector3i id_occ;
    int adr_occ;
    pt_occ << occ.x, occ.y, occ.z;
    posToIndex_hc(pt_occ, id_occ);
    adr_occ = toAddress_hc(id_occ);
    hcmd_->occupancy_buffer_hc_[adr_occ] = 1;
  }
}

void SDFMap::inputFreePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& points)
{
  for (auto free:points->points)
  {
    Eigen::Vector3d pt_free;
    Eigen::Vector3i id_free;
    int adr_free;
    pt_free << free.x, free.y, free.z;
    posToIndex_hc(pt_free, id_free);
    adr_free = toAddress_hc(id_free);
    if(hcmd_->occupancy_buffer_hc_[adr_free] == 0)
      hcmd_->occupancy_buffer_hc_[adr_free] = 2;
  }
}

void SDFMap::InternalSpace(map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>& seg_cloud, Eigen::MatrixXd& vertices, map<int, vector<int>>& segments)
{ 
  Eigen::Vector3d pt_w;
  Eigen::Vector3i idx;
  int vox_adr, seg_id;
  Eigen::Vector3d seg_start, seg_end, proj_pt, seg_dir;
  double seg_length;
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud;

  for (const auto& id_pts:seg_cloud)
  {
    seg_id = id_pts.first;
    seg_start = vertices.row(segments[seg_id][0]);
    seg_end = vertices.row(segments[seg_id][1]);
    seg_dir = (seg_end-seg_start).normalized();
    seg_length = (seg_end-seg_start).norm();
    temp_cloud = id_pts.second;
    
    for (auto pt:id_pts.second->points)
    {
      pt_w << pt.x, pt.y, pt.z;
      posToIndex_hc(pt_w, idx);
      vox_adr = toAddress_hc(idx);
      hcmd_->occupancy_buffer_hc_[vox_adr] = 1;
      hcmd_->occupancy_inflate_buffer_hc_[vox_adr] = 1;

      for (int i=-inflate_num; i<inflate_num+1; i=i+1)
        for (int j=-inflate_num; j<inflate_num+1; j=j+1)
          for (int k=-inflate_num; k<inflate_num+1; k=k+1)
          {
            if (i == 0 && j == 0 && k == 0) {
              continue;
            }
            Eigen::Vector3i neighbor;
            neighbor(0)=idx.x() + i; neighbor(1)=idx.y() + j; neighbor(2)=idx.z() + k;
            hcmd_->occupancy_inflate_buffer_hc_[toAddress_hc(neighbor)] = 1;
          }
    }
    
    /* cutting plane method */
    const std::vector<int> offsets {-1, 0, 1};
    hcmd_->seg_occ_visited_buffer_.clear();
    hcmd_->seg_occ_visited_buffer_.resize(temp_cloud->points.size(), false);
    vector<Eigen::Vector3d> proj_points;
    for (int i=0; hcmp_->proj_interval*i<seg_length; ++i)
    {
      proj_pt = seg_start + hcmp_->proj_interval*i*seg_dir;
      proj_points.push_back(proj_pt);
    }
    proj_points.push_back(seg_end);
    
    vector<Eigen::Vector3d> cut_plane;
    for (auto proj_pt:proj_points)
    {
      cut_plane = points_in_plane(temp_cloud, proj_pt, seg_dir, hcmp_->thickness);
      for (auto in_pt:cut_plane)
      {
        internal_cast_->input(proj_pt, in_pt);
        internal_cast_->nextId(idx);
        while (internal_cast_->nextId(idx))
        {
          hcmd_->occupancy_buffer_internal_[toAddress_hc(idx)] = 1;

        }
      }
    }
    
    for (int j=0; j<(int)hcmd_->seg_occ_visited_buffer_.size(); ++j)
    {
      Eigen::Vector3d remain_pt;
      double dist = 0.0, dist_min = 100000.0;
      int distri_id = -1;

      if (hcmd_->seg_occ_visited_buffer_[j] == false)
      {
        remain_pt << temp_cloud->points[j].x, temp_cloud->points[j].y, temp_cloud->points[j].z;
        for (int k=0; k<(int)proj_points.size(); ++k)
        {
          dist = (proj_points[k] - remain_pt).norm();
          if (dist < dist_min)
          {
            dist_min = dist;
            distri_id = k;
          }
        }

        internal_cast_->input(proj_points[distri_id], remain_pt);
        internal_cast_->nextId(idx);
        while (internal_cast_->nextId(idx))
        {
          hcmd_->occupancy_buffer_internal_[toAddress_hc(idx)] = 1;

        }
      }
    }

  }

}

void SDFMap::setInternal(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const unordered_map<int, Eigen::Vector3d>& cor_inliers)
{
  // voxel inflation
  Eigen::Vector3d pt_w;
  Eigen::Vector3i idx;
  int vox_adr;
  for (auto pt:cloud->points)
  {
    pt_w << pt.x, pt.y, pt.z;
    posToIndex_hc(pt_w, idx);
    vox_adr = toAddress_hc(idx);
    hcmd_->occupancy_buffer_hc_[vox_adr] = 1;
    hcmd_->occupancy_inflate_buffer_hc_[vox_adr] = 1;

    for (int i=-inflate_num; i<inflate_num+1; i=i+1)
      for (int j=-inflate_num; j<inflate_num+1; j=j+1)
        for (int k=-inflate_num; k<inflate_num+1; k=k+1)
        {
          if (i == 0 && j == 0 && k == 0) continue;
          Eigen::Vector3i neighbor;
          neighbor(0)=idx.x() + i; neighbor(1)=idx.y() + j; neighbor(2)=idx.z() + k;
          hcmd_->occupancy_inflate_buffer_hc_[toAddress_hc(neighbor)] = 1;
        }
  }

  // internal space generation
  Eigen::Vector3d pt_vec, inlier_vec;
  for (auto& pair:cor_inliers)
  {
    pt_vec << cloud->points[pair.first].x, cloud->points[pair.first].y, cloud->points[pair.first].z;
    inlier_vec = pair.second;
    
    internal_cast_->input(inlier_vec, pt_vec);
    while (internal_cast_->nextId(idx))
    {
      hcmd_->occupancy_buffer_internal_[toAddress_hc(idx)] = 1;
    }
  }
}

void SDFMap::resetHCMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& model)
{
  double x_size, y_size, z_size;
  x_size = hcmp_->x_size_;
  y_size = hcmp_->y_size_;
  z_size = hcmp_->z_size_;

  hcmp_->resolution_inv_ = 1 / hcmp_->resolution_;
  hcmp_->map_origin_ = mp_->map_origin_;
  posToIndex_hc(hcmp_->map_origin_, hcmp_->map_origin_idx_);
  hcmp_->map_size_ = Eigen::Vector3d(x_size, y_size, z_size);
  for (int i = 0; i < 3; ++i)
    hcmp_->map_voxel_num_(i) = ceil(hcmp_->map_size_(i) / hcmp_->resolution_);
  hcmp_->map_min_boundary_ = hcmp_->map_origin_;
  hcmp_->map_max_boundary_ = hcmp_->map_origin_ + hcmp_->map_size_;
  hcmp_->box_mind_ = hcmp_->map_min_boundary_;
  hcmp_->box_maxd_ = hcmp_->map_max_boundary_;
  posToIndex_hc(hcmp_->box_mind_, hcmp_->box_min_);
  posToIndex_hc(hcmp_->box_maxd_, hcmp_->box_max_);

  int buffer_size = hcmp_->map_voxel_num_(0) * hcmp_->map_voxel_num_(1) * hcmp_->map_voxel_num_(2);
  // * Occupancy buffer
  hcmd_->occupancy_buffer_hc_ = vector<char>(buffer_size, 0);
  hcmd_->occupancy_inflate_buffer_hc_ = vector<char>(buffer_size, 0);
  hcmd_->occupancy_buffer_internal_ = vector<char>(buffer_size, 0);
  hcmd_->count_observed_buffer_ = vector<char>(buffer_size, 0);
  // * ESDF buffer
  if (mp_->signed_dist_) hcmd_->distance_buffer_neg_hc_ = vector<double>(buffer_size, mp_->default_dist_);
  hcmd_->distance_buffer_hc_ = vector<double>(buffer_size, mp_->default_dist_);
  hcmd_->tmp_buffer1_hc_ = vector<double>(buffer_size, 0);
  hcmd_->tmp_buffer2_hc_ = vector<double>(buffer_size, 0);

  internal_cast_.reset(new RayCaster);
  internal_cast_->setParams(hcmp_->resolution_, hcmp_->map_origin_);

  for (auto occ:model->points)
  {
    Eigen::Vector3d pt_occ;
    Eigen::Vector3i id_occ;
    int adr_occ;
    pt_occ << occ.x, occ.y, occ.z;

    if (!isInMap_hc(pt_occ)) continue; 

    posToIndex_hc(pt_occ, id_occ);
    adr_occ = toAddress_hc(id_occ);
    hcmd_->occupancy_buffer_hc_[adr_occ] = 1;
    hcmd_->occupancy_inflate_buffer_hc_[adr_occ] = 1;
    
    for (int i=-inflate_num; i<inflate_num+1; i=i+1)
      for (int j=-inflate_num; j<inflate_num+1; j=j+1)
        for (int k=-inflate_num; k<inflate_num+1; k=k+1)
        {
          if (i == 0 && j == 0 && k == 0) continue;
          Eigen::Vector3i neighbor;
          neighbor(0)=id_occ.x() + i; neighbor(1)=id_occ.y() + j; neighbor(2)=id_occ.z() + k;

          if (!isInMap_hc(neighbor)) continue;

          hcmd_->occupancy_inflate_buffer_hc_[toAddress_hc(neighbor)] = 1;
        }
  }

  return;
}

void SDFMap::resetHCVirtualBound(const double min_x, const double max_x, const double min_y, const double max_y)
{
  // x-axis
  for (double x = min_x; x <= max_x; x += hcmp_->resolution_)
  {
    for (double z = hcmp_->map_min_boundary_(2); z <= hcmp_->map_max_boundary_(2); z += hcmp_->resolution_)
    {
      Eigen::Vector3d pt_y_min(x, min_y, z);
      Eigen::Vector3d pt_y_max(x, max_y, z);

      if (isInMap_hc(pt_y_min))
      {
        Eigen::Vector3i idx_y_min;
        posToIndex_hc(pt_y_min, idx_y_min);
        hcmd_->occupancy_inflate_buffer_hc_[toAddress_hc(idx_y_min)] = 1;
      }

      if (isInMap_hc(pt_y_max))
      {
        Eigen::Vector3i idx_y_max;
        posToIndex_hc(pt_y_max, idx_y_max);
        hcmd_->occupancy_inflate_buffer_hc_[toAddress_hc(idx_y_max)] = 1;
      }
    }
  }

  // y-axis
  for (double y = min_y; y <= max_y; y += hcmp_->resolution_)
  {
    for (double z = hcmp_->map_min_boundary_(2); z <= hcmp_->map_max_boundary_(2); z += hcmp_->resolution_)
    {
      Eigen::Vector3d pt_x_min(min_x, y, z);
      Eigen::Vector3d pt_x_max(max_x, y, z);
      
      if (isInMap_hc(pt_x_min))
      {
        Eigen::Vector3i idx_x_min;
        posToIndex_hc(pt_x_min, idx_x_min);
        hcmd_->occupancy_inflate_buffer_hc_[toAddress_hc(idx_x_min)] = 1;
      }

      if (isInMap_hc(pt_x_max))
      {
        Eigen::Vector3i idx_x_max;
        posToIndex_hc(pt_x_max, idx_x_max);
        hcmd_->occupancy_inflate_buffer_hc_[toAddress_hc(idx_x_max)] = 1;
      }
    }
  }

  return;
}

void SDFMap::getGlobalHCMap(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  Eigen::Vector3d min_local = hcmp_->map_min_boundary_, max_local = hcmp_->map_max_boundary_;

  if (zFlag == true)
    min_local(2) = z_min;
  
  double sub_res = 1.0*hcmp_->resolution_;

  pcl::PointXYZ pt;
  for (double x = min_local(0); x <= max_local(0); x += sub_res)
    for (double y = min_local(1); y <= max_local(1); y += sub_res)
      for (double z = min_local(2); z <= max_local(2); z += sub_res)
      {
        Eigen::Vector3d pt_d(x, y, z);
        Eigen::Vector3i pt_idx;
        posToIndex_hc(pt_d, pt_idx);
        if (hcmd_->occupancy_buffer_hc_[toAddress_hc(pt_idx)] == 1)
        {
          pt.x = pt_d(0);
          pt.y = pt_d(1);
          pt.z = pt_d(2);
          cloud->points.push_back(pt);
        }
      }
  
  return;
}

void SDFMap::getLocalHCMap(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, Eigen::Vector3d& min, Eigen::Vector3d& max)
{
  Eigen::Vector3d min_local = min, max_local = max;

  if (zFlag == true)
    min_local(2) = z_min;

  min_local(0) = std::max(min_local(0), hcmp_->map_min_boundary_(0) + 1e-4);
  min_local(1) = std::max(min_local(1), hcmp_->map_min_boundary_(1) + 1e-4);
  min_local(2) = std::max(min_local(2), hcmp_->map_min_boundary_(2) + 1e-4);
  max_local(0) = std::min(max_local(0), hcmp_->map_max_boundary_(0) - 1e-4);
  max_local(1) = std::min(max_local(1), hcmp_->map_max_boundary_(1) - 1e-4);
  max_local(2) = std::min(max_local(2), hcmp_->map_max_boundary_(2) - 1e-4);

  Eigen::Vector3i min_idx, max_idx;
  posToIndex_hc(min_local, min_idx);
  posToIndex_hc(max_local, max_idx);

  int ds_factor = 1;

  int x_steps = (max_local(0) - min_local(0)) / (hcmp_->resolution_) + 1;
  int y_steps = (max_local(1) - min_local(1)) / (hcmp_->resolution_) + 1;
  int z_steps = (max_local(2) - min_local(2)) / (hcmp_->resolution_) + 1;

  size_t estimated_points = x_steps * y_steps * z_steps;
  Eigen::MatrixXd points(3, estimated_points);
  size_t current_size = 0;

  Eigen::Vector3d pt_d;
  Eigen::Vector3i pt_idx;
  pcl::PointXYZ pt;
  for (int x = min_idx(0); x <= max_idx(0); x+=ds_factor)
    for (int y = min_idx(1); y <= max_idx(1); y+=ds_factor)
      for (int z = min_idx(2); z <= max_idx(2); z+=ds_factor)
      {
        pt_idx << x, y, z;

        int adr = toAddress_hc(pt_idx);
        if (hcmd_->occupancy_buffer_hc_[adr] == 1)
        {
          if (this->en_user_)
          {
            if (hcmd_->user_obs_buffer_[adr] == 1)
            {
              if (hcmd_->count_observed_buffer_[adr] > this->belief_count_) continue;
              indexToPos_hc(pt_idx, pt_d);
              if (pt_d(2) < (this->ground_threshold_ + this->omit_height_) || pt_d(2) > (this->mp_->virtual_ceil_height_ - this->mp_->obs_inf_z_)) continue;
              points(0, current_size) = pt_d(0);
              points(1, current_size) = pt_d(1);
              points(2, current_size) = pt_d(2);
              current_size++;
            }
          }
          else
          {
            if (hcmd_->count_observed_buffer_[adr] > this->belief_count_) continue;
            indexToPos_hc(pt_idx, pt_d);
            if (pt_d(2) < (this->ground_threshold_ + this->omit_height_) || pt_d(2) > (this->mp_->virtual_ceil_height_ - this->mp_->obs_inf_z_)) continue;
            points(0, current_size) = pt_d(0);
            points(1, current_size) = pt_d(1);
            points(2, current_size) = pt_d(2);
            current_size++;
          }
        }
      }

  cloud->points.reserve(current_size);
  for (size_t i = 0; i < current_size; i++) 
  {
    pcl::PointXYZ pt;
    pt.x = points(0, i);
    pt.y = points(1, i);
    pt.z = points(2, i);
    cloud->points.push_back(pt);
  }
  
  return;
}

void SDFMap::setObserved(Eigen::Vector3d& p_w)
{
  Eigen::Vector3i idx;
  posToIndex_hc(p_w, idx);
  int adr = toAddress_hc(idx);
  hcmd_->count_observed_buffer_[adr] += 1;

  return;
}

vector<Eigen::Vector3d> SDFMap::points_in_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud, Eigen::Vector3d& point_on_plane, Eigen::Vector3d& plane_normal, double thickness)
{
  vector<Eigen::Vector3d> points;
  Eigen::Vector3d pt_vec;
  for (int i=0; i<(int)point_cloud->points.size(); ++i)
  {
    pt_vec << point_cloud->points[i].x, point_cloud->points[i].y, point_cloud->points[i].z;
    double distance_from_plane = (pt_vec - point_on_plane).dot(plane_normal);
    if (std::abs(distance_from_plane) <= thickness)
    {
      points.push_back(pt_vec);
      hcmd_->seg_occ_visited_buffer_[i] = true;
    }
  }

  return points;
}

void SDFMap::getLocalEnv(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, Eigen::Vector3d& min, Eigen::Vector3d& max)
{
  Eigen::Vector3d min_local = min, max_local = max;

  min_local(0) = std::max(min_local(0), mp_->map_min_boundary_(0) + 1e-4);
  min_local(1) = std::max(min_local(1), mp_->map_min_boundary_(1) + 1e-4);
  min_local(2) = std::max(min_local(2), mp_->map_min_boundary_(2) + 1e-4);
  max_local(0) = std::min(max_local(0), mp_->map_max_boundary_(0) - 1e-4);
  max_local(1) = std::min(max_local(1), mp_->map_max_boundary_(1) - 1e-4);
  max_local(2) = std::min(max_local(2), mp_->map_max_boundary_(2) - 1e-4);

  Eigen::Vector3i min_idx, max_idx;
  posToIndex(min_local, min_idx);
  posToIndex(max_local, max_idx);

  int ds_factor = 1;

  int x_steps = (max(0) - min(0)) / (mp_->resolution_) + 1;
  int y_steps = (max(1) - min(1)) / (mp_->resolution_) + 1;
  int z_steps = (max(2) - min(2)) / (mp_->resolution_) + 1;

  size_t estimated_points = x_steps * y_steps * z_steps;
  Eigen::MatrixXd points(3, estimated_points);
  size_t current_size = 0;

  Eigen::Vector3d pt_d;
  Eigen::Vector3i pt_idx;
  pcl::PointXYZ pt;
  for (int x = min_idx(0); x <= max_idx(0); x+=ds_factor)
    for (int y = min_idx(1); y <= max_idx(1); y+=ds_factor)
      for (int z = min_idx(2); z <= max_idx(2); z+=ds_factor)
      {
        pt_idx << x, y, z;

        if (md_->env_buffer_[toAddress(pt_idx)] == 1)
        {
          indexToPos(pt_idx, pt_d);
          if (pt_d(2) >= mp_->virtual_ceil_height_) continue;
          points(0, current_size) = pt_d(0);
          points(1, current_size) = pt_d(1);
          points(2, current_size) = pt_d(2);
          current_size++;
        }
      }
  
  cloud->points.reserve(current_size);
  for (size_t i = 0; i < current_size; i++) 
  {
    pcl::PointXYZ pt;
    pt.x = points(0, i);
    pt.y = points(1, i);
    pt.z = points(2, i);
    cloud->points.push_back(pt);
  }

  return;
}

void SDFMap::getLocalOccEnv(pcl::PointCloud<pcl::PointXYZ>::Ptr& occ_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& env_cloud, Eigen::Vector3d& min, Eigen::Vector3d& max)
{
  pcl::PointXYZ pt;
  std::vector<pcl::PointXYZ> occ_points, env_points;
  for (double x = min(0); x <= max(0); x += mp_->resolution_)
    for (double y = min(1); y <= max(1); y += mp_->resolution_)
      for (double z = min(2); z <= max(2); z += mp_->resolution_)
      {
        Eigen::Vector3d pt_d(x, y, z);
        if (getOccupancy(pt_d) == OCCUPIED)
        {
          pt.x = pt_d(0);
          pt.y = pt_d(1);
          pt.z = pt_d(2);
          occ_points.push_back(pt);

          Eigen::Vector3i pt_idx;
          posToIndex(pt_d, pt_idx);
          if (md_->env_buffer_[toAddress(pt_idx)] == 1)
          {
            env_points.push_back(pt);
          }
        }
      }
  
  occ_cloud->points.insert(occ_cloud->points.end(), occ_points.begin(), occ_points.end());
  env_cloud->points.insert(env_cloud->points.end(), env_points.begin(), env_points.end());

  return;
}

void SDFMap::resetBuffer() 
{
  resetBuffer(mp_->map_min_boundary_, mp_->map_max_boundary_);
  md_->local_bound_min_ = Eigen::Vector3i::Zero();
  md_->local_bound_max_ = mp_->map_voxel_num_ - Eigen::Vector3i::Ones();
}

void SDFMap::resetBuffer(const Eigen::Vector3d& min_pos, const Eigen::Vector3d& max_pos) {
  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z) {
        md_->occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
        md_->distance_buffer_[toAddress(x, y, z)] = mp_->default_dist_;
      }
}

bool SDFMap::getSafety(const Eigen::Vector3d& pos, const double x_range, const double y_range, const double z_range)
{
  Eigen::Vector3d min_pos = pos - Eigen::Vector3d(x_range, y_range, z_range);
  Eigen::Vector3d max_pos = pos + Eigen::Vector3d(x_range, y_range, z_range);
  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        if (getOccupancy(Eigen::Vector3i(x, y, z)) == OCCUPIED)
        {
          return false;
        }
      }
  
  return true;
}

bool SDFMap::getHCSafety(const Eigen::Vector3d& pos, const double x_range, const double y_range, const double z_range)
{
  Eigen::Vector3d min_pos = pos - Eigen::Vector3d(x_range, y_range, z_range);
  Eigen::Vector3d max_pos = pos + Eigen::Vector3d(x_range, y_range, z_range);
  Eigen::Vector3i min_id, max_id;
  posToIndex_hc(min_pos, min_id);
  posToIndex_hc(max_pos, max_id);
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        if (hcmd_->occupancy_buffer_hc_[toAddress_hc(Eigen::Vector3i(x, y, z))] == 1)
        {
          return false;
        }
      }

  return true;
} 

template <typename F_get_val, typename F_set_val>
void SDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) {
  int v[mp_->map_voxel_num_(dim)];
  double z[mp_->map_voxel_num_(dim) + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++) {
    k++;
    double s;

    do {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;

    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;

  for (int q = start; q <= end; q++) {
    while (z[k + 1] < q)
      k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}

void SDFMap::updateESDF3d() {
  Eigen::Vector3i min_esdf = md_->local_bound_min_;
  Eigen::Vector3i max_esdf = md_->local_bound_max_;

  if (mp_->optimistic_) {
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
      for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        fillESDF(
            [&](int z) {
              return md_->occupancy_buffer_inflate_[toAddress(x, y, z)] == 1 ?
                  0 :
                  std::numeric_limits<double>::max();
            },
            [&](int z, double val) { md_->tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
            max_esdf[2], 2);
      }
  } else {
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
      for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        fillESDF(
            [&](int z) {
              int adr = toAddress(x, y, z);
              return (md_->occupancy_buffer_inflate_[adr] == 1 ||
                      md_->occupancy_buffer_[adr] < mp_->clamp_min_log_ - 1e-3) ?
                  0 :
                  std::numeric_limits<double>::max();
            },
            [&](int z, double val) { md_->tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
            max_esdf[2], 2);
      }
  }

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF(
          [&](int y) { return md_->tmp_buffer1_[toAddress(x, y, z)]; },
          [&](int y, double val) { md_->tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
          max_esdf[1], 1);
    }
  for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF(
          [&](int x) { return md_->tmp_buffer2_[toAddress(x, y, z)]; },
          [&](int x, double val) {
            md_->distance_buffer_[toAddress(x, y, z)] = mp_->resolution_ * std::sqrt(val);
          },
          min_esdf[0], max_esdf[0], 0);
    }

  if (mp_->signed_dist_) 
  {
    // Compute negative distance
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
      for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        fillESDF(
            [&](int z) {
              return md_->occupancy_buffer_inflate_
                          [x * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2) +
                           y * mp_->map_voxel_num_(2) + z] == 0 ?
                  0 :
                  std::numeric_limits<double>::max();
            },
            [&](int z, double val) { md_->tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
            max_esdf[2], 2);
      }
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
      for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
        fillESDF(
            [&](int y) { return md_->tmp_buffer1_[toAddress(x, y, z)]; },
            [&](int y, double val) { md_->tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
            max_esdf[1], 1);
      }
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
      for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
        fillESDF(
            [&](int x) { return md_->tmp_buffer2_[toAddress(x, y, z)]; },
            [&](int x, double val) {
              md_->distance_buffer_neg_[toAddress(x, y, z)] = mp_->resolution_ * std::sqrt(val);
            },
            min_esdf[0], max_esdf[0], 0);
      }
    // Merge negative distance with positive
    for (int x = min_esdf(0); x <= max_esdf(0); ++x)
      for (int y = min_esdf(1); y <= max_esdf(1); ++y)
        for (int z = min_esdf(2); z <= max_esdf(2); ++z) {
          int idx = toAddress(x, y, z);
          if (md_->distance_buffer_neg_[idx] > 0.0)
            md_->distance_buffer_[idx] += (-md_->distance_buffer_neg_[idx] + mp_->resolution_);
        }
  }
}

void SDFMap::updateESDF3dHCMap()
{
  Eigen::Vector3i min_esdf = hcmp_->box_min_ + Eigen::Vector3i::Ones();
  Eigen::Vector3i max_esdf = hcmp_->box_max_ - Eigen::Vector3i::Ones();

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) 
    {
      fillESDF(
          [&](int z) {
            return hcmd_->occupancy_buffer_hc_[toAddress_hc(x, y, z)] == 1 ?
            0 :
            std::numeric_limits<double>::max();
          },
          [&](int z, double val) { hcmd_->tmp_buffer1_hc_[toAddress_hc(x, y, z)] = val; }, min_esdf[2],
          max_esdf[2], 2);
    }
  
  for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) 
    {
      fillESDF(
          [&](int y) { return hcmd_->tmp_buffer1_hc_[toAddress_hc(x, y, z)]; },
          [&](int y, double val) { hcmd_->tmp_buffer2_hc_[toAddress_hc(x, y, z)] = val; }, min_esdf[1],
          max_esdf[1], 1);
    }
  
  for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) 
    {
      fillESDF(
          [&](int x) { return hcmd_->tmp_buffer2_hc_[toAddress_hc(x, y, z)]; },
          [&](int x, double val) {
            hcmd_->distance_buffer_hc_[toAddress(x, y, z)] = hcmp_->resolution_ * std::sqrt(val);
          },
          min_esdf[0], max_esdf[0], 0);
    }
  
  if (mp_->signed_dist_)
  {
    // Compute negative distance
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
      for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        fillESDF(
            [&](int z) {
              return 0;
            },
            [&](int z, double val) { hcmd_->tmp_buffer1_hc_[toAddress_hc(x, y, z)] = val; }, min_esdf[2],
            max_esdf[2], 2);
      }
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
      for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
        fillESDF(
            [&](int y) { return hcmd_->tmp_buffer1_hc_[toAddress_hc(x, y, z)]; },
            [&](int y, double val) { hcmd_->tmp_buffer2_hc_[toAddress_hc(x, y, z)] = val; }, min_esdf[1],
            max_esdf[1], 1);
      }
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
      for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
        fillESDF(
            [&](int x) { return hcmd_->tmp_buffer2_hc_[toAddress_hc(x, y, z)]; },
            [&](int x, double val) {
              hcmd_->distance_buffer_neg_hc_[toAddress_hc(x, y, z)] = hcmp_->resolution_ * std::sqrt(val);
            },
            min_esdf[0], max_esdf[0], 0);
      }
    // Merge negative distance with positive
    for (int x = min_esdf(0); x <= max_esdf(0); ++x)
      for (int y = min_esdf(1); y <= max_esdf(1); ++y)
        for (int z = min_esdf(2); z <= max_esdf(2); ++z) {
          int idx = toAddress_hc(x, y, z);
          if (hcmd_->distance_buffer_neg_hc_[idx] > 0.0)
            hcmd_->distance_buffer_hc_[idx] += (-hcmd_->distance_buffer_neg_hc_[idx] + hcmp_->resolution_);
        }

  }

  return;
}

void SDFMap::setCacheOccupancy(const int& adr, const int& occ) 
{
  // Add to update list if first visited
  if (md_->count_hit_[adr] == 0 && md_->count_miss_[adr] == 0) md_->cache_voxel_.push(adr);

  if (occ == 0)
    md_->count_miss_[adr] += 1;
  else if (occ == 1)
    md_->count_hit_[adr] += 1;
}

void SDFMap::inputPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>& points, const int& point_num,
    const Eigen::Vector3d& camera_pos, bool air) 
{
  if (point_num == 0) return;
  md_->raycast_num_ += 1;

  Eigen::Vector3d update_min = camera_pos;
  Eigen::Vector3d update_max = camera_pos;
  if (md_->reset_updated_box_) 
  {
    md_->update_min_ = camera_pos;
    md_->update_max_ = camera_pos;
    md_->reset_updated_box_ = false;
  }

  double lowest = mp_->ground_height_;

  Eigen::Vector3d pt_w, tmp;
  Eigen::Vector3i idx;
  int vox_adr;
  double length;
  for (int i = 0; i < point_num; ++i) {
    auto& pt = points.points[i];
    pt_w << pt.x, pt.y, pt.z;
    int tmp_flag;
    // Set flag for projected point
    if (!isInMap(pt_w)) 
    {
      // Find closest point in map and set free
      pt_w = closetPointInMap(pt_w, camera_pos);
      length = (pt_w - camera_pos).norm();
      if (length > mp_->max_ray_length_)
        pt_w = (pt_w - camera_pos) / length * mp_->max_ray_length_ + camera_pos;
      if (pt_w[2] < lowest) continue;
      tmp_flag = 0;
    } 
    else 
    {
      length = (pt_w - camera_pos).norm();
      if (length > mp_->max_ray_length_) 
      {
        pt_w = (pt_w - camera_pos) / length * mp_->max_ray_length_ + camera_pos;
        if (pt_w[2] < lowest) continue;
        tmp_flag = 0;
      }
      else
        tmp_flag = 1;
    }
    posToIndex(pt_w, idx);
    vox_adr = toAddress(idx);
    if(!air) 
    {
      setCacheOccupancy(vox_adr, tmp_flag);
      for (int k = 0; k < 3; ++k) 
      {
        update_min[k] = min(update_min[k], pt_w[k]);
        update_max[k] = max(update_max[k], pt_w[k]);
      }
    }
    // Raycasting between camera center and point
    if (md_->flag_rayend_[vox_adr] == md_->raycast_num_)
      continue;
    else if (md_->flag_rayend_[vox_adr] != md_->raycast_num_)
      md_->flag_rayend_[vox_adr] = md_->raycast_num_;
 
    if (air)
    {
      caster_->input(camera_pos, pt_w);
      while (caster_->nextId(idx)) 
      {
        if(getInflateOccupancy(idx) == 1) break;
        if (isInMap(idx)) setCacheOccupancy(toAddress(idx), 0);
      }
    }
    else
    {
      caster_->input(camera_pos, pt_w);
      caster_->nextId(idx);
      while (caster_->nextId(idx)) 
      {
        if (isInMap(idx)) setCacheOccupancy(toAddress(idx), 0);
      }
    }
  }

  if (!air)
  {
    update_max[2] = mp_->box_maxd_[2];
    Eigen::Vector3d bound_inf(mp_->local_bound_inflate_, mp_->local_bound_inflate_, mp_->local_bound_inflate_);
    posToIndex(update_max + bound_inf, md_->local_bound_max_);
    posToIndex(update_min - bound_inf, md_->local_bound_min_);
    boundIndex(md_->local_bound_min_);
    boundIndex(md_->local_bound_max_);
    mr_->local_updated_ = true;

    // Bounding box for subsequent updating
    for (int k = 0; k < 3; ++k) 
    {
      md_->update_min_[k] = min(update_min[k], md_->update_min_[k]);
      md_->update_max_[k] = max(update_max[k], md_->update_max_[k]);
    }
  }

  while (!md_->cache_voxel_.empty()) 
  {
    int adr = md_->cache_voxel_.front();
    md_->cache_voxel_.pop();

    double log_odds_update = 0.0;
    log_odds_update = md_->count_hit_[adr] >= md_->count_miss_[adr] ? mp_->prob_hit_log_ : mp_->prob_miss_log_;

    md_->count_hit_[adr] = md_->count_miss_[adr] = 0;

    if (md_->occupancy_buffer_[adr] < mp_->clamp_min_log_ - 1e-3)
      md_->occupancy_buffer_[adr] = mp_->min_occupancy_log_;

    md_->occupancy_buffer_[adr] = std::min(
        std::max(md_->occupancy_buffer_[adr] + log_odds_update, mp_->clamp_min_log_),
        mp_->clamp_max_log_);


    if (this->en_attributes_)
    {
      Eigen::Vector3i idx;
      Eigen::Vector3d pt;
      this->invAddress(adr, idx);
      this->indexToPos(idx, pt);
      if (md_->target_buffer_[adr] == 1 || md_->env_buffer_[adr] == 1)
      {
        if (this->getOccupancy(idx) != OCCUPIED)
        {
          md_->target_buffer_[adr] = 0;
          md_->env_buffer_[adr] = 0;
        }
      }

      if (this->getOccupancy(idx) == OCCUPIED)
      {
        if (pt(2) > this->ground_threshold_)
        {
          double hc_esdf = this->getDistance_hc(pt);
          if (abs(hc_esdf) < this->attri_threshold_)
            this->md_->target_buffer_[adr] = 1;
          else
            this->md_->env_buffer_[adr] = 1;
        }
        else
        {
          md_->target_buffer_[adr] = 0;
          md_->env_buffer_[adr] = 0;
        }
      }
    }
    
  }
}

void SDFMap::updateMapAttributes()
{
  for (int x = mp_->box_min_(0); x < mp_->box_max_(0); ++x)
    for (int y = mp_->box_min_(1); y < mp_->box_max_(1); ++y)
      for (int z = mp_->box_min_(2); z < mp_->box_max_(2); ++z) 
      {
        Eigen::Vector3i idx(x, y, z);
        Eigen::Vector3d pt;
        this->indexToPos(idx, pt);
        if (pt(2) > this->ground_threshold_)
        {
          if (this->getOccupancy(idx) == OCCUPIED)
          {
            int id = this->toAddress(idx);
            double hc_esdf = this->getDistance_hc(pt);
            if (abs(hc_esdf) < this->attri_threshold_)
              this->md_->target_buffer_[id] = 1;
            else
              this->md_->env_buffer_[id] = 1;
          }
        } 
      }

  this->en_attributes_ = true;

  return;
}

void SDFMap::inputUserObs(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  this->en_user_ = true;
  int buffer_size = hcmp_->map_voxel_num_(0) * hcmp_->map_voxel_num_(1) * hcmp_->map_voxel_num_(2);
  hcmd_->user_obs_buffer_.resize(buffer_size, 0);
  for (int i=0; i<(int)cloud->points.size(); ++i)
  {
    Eigen::Vector3d pt_w;
    pt_w << cloud->points[i].x, cloud->points[i].y, cloud->points[i].z;
    if (!isInMap_hc(pt_w)) continue;
    Eigen::Vector3i idx;
    posToIndex_hc(pt_w, idx);
    int adr = toAddress_hc(idx);
    hcmd_->user_obs_buffer_[adr] = 1;
  }

  return;
}

Eigen::Vector3d SDFMap::closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt) {
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_->map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_->map_min_boundary_ - camera_pt;
  double min_t = 1000000;
  for (int i = 0; i < 3; ++i) {
    if (fabs(diff[i]) > 0) {
      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t) min_t = t1;
      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t) min_t = t2;
    }
  }
  return camera_pt + (min_t - 1e-3) * diff;
}

void SDFMap::clearAndInflateLocalMap() 
{
  int inf_step = ceil(mp_->obstacles_inflation_ / mp_->resolution_);
  int inf_z_step = ceil(mp_->obs_inf_z_ / mp_->resolution_);
  int target_inf_step = ceil(mp_->target_obstacles_inflation_ / mp_->resolution_);
  int target_inf_z_step = ceil(mp_->target_obs_inf_z_ / mp_->resolution_);
  vector<Eigen::Vector3i> inf_pts;

  // clear old inflated occupancy
  for (int x = md_->local_bound_min_(0); x <= md_->local_bound_max_(0); ++x)
    for (int y = md_->local_bound_min_(1); y <= md_->local_bound_max_(1); ++y)
      for (int z = md_->local_bound_min_(2); z <= md_->local_bound_max_(2); ++z) {
        if (isInMap(Eigen::Vector3i(x, y, z)))
          md_->occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }

  // inflate newest occpuied cells
  for (int x = md_->local_bound_min_(0); x <= md_->local_bound_max_(0); ++x)
    for (int y = md_->local_bound_min_(1); y <= md_->local_bound_max_(1); ++y)
      for (int z = md_->local_bound_min_(2); z <= md_->local_bound_max_(2); ++z) {
        if (!isInMap(Eigen::Vector3i(x, y, z))) continue; 
        int id1 = toAddress(x, y, z);
        if (md_->occupancy_buffer_[id1] > mp_->min_occupancy_log_) 
        {
          inf_pts.clear();
          if (md_->target_buffer_[id1] == 1)
            inflatePoint(Eigen::Vector3i(x, y, z), target_inf_step, target_inf_z_step, inf_pts);
          else
            inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_z_step, inf_pts);

          for (auto inf_pt : inf_pts) 
          {
            int idx_inf = toAddress(inf_pt);
            if (!isInMap(inf_pt)) continue;

            if (idx_inf >= 0 &&
                idx_inf < mp_->map_voxel_num_(0) * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2)) 
            {
              md_->occupancy_buffer_inflate_[idx_inf] = 1;
            }
          }
        }
      }

  // add virtual ceiling to limit flight height
  if (mp_->virtual_ceil_height_ > -0.5) 
  {
    int ceil_id = floor((mp_->virtual_ceil_height_ - mp_->map_origin_(2)) * mp_->resolution_inv_);
    for (int x = md_->local_bound_min_(0); x <= md_->local_bound_max_(0); ++x)
      for (int y = md_->local_bound_min_(1); y <= md_->local_bound_max_(1); ++y) {
        if (!isInMap(Eigen::Vector3i(x, y, ceil_id))) continue;
        md_->occupancy_buffer_[toAddress(x, y, ceil_id)] = mp_->clamp_max_log_;

        inf_pts.clear();
        inflatePoint(Eigen::Vector3i(x, y, ceil_id), inf_step, inf_z_step, inf_pts);
        for (auto inf_pt : inf_pts) 
        {
          int idx_inf = toAddress(inf_pt);
          if (!isInMap(inf_pt)) continue;

          if (idx_inf >= 0 &&
              idx_inf < mp_->map_voxel_num_(0) * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2)) 
          {
            md_->occupancy_buffer_inflate_[idx_inf] = 1;
          }
        }
      }
  }
}

void SDFMap::clearAndMapTarget(const pcl::PointCloud<pcl::PointXYZ>& points, const int& point_num, const Eigen::Vector3d& camera_pos)
{
  double lowest = mp_->ground_height_;

  // * clear old target points
  for (int i=0; i<(int)md_->target_buffer_.size(); ++i)
    md_->target_buffer_[i] = 0;
  
  // * map new target points
  if (point_num == 0) return;
  Eigen::Vector3d pt_w;

  for (int i=0; i<point_num; ++i)
  {
    pt_w << points.points[i].x, points.points[i].y, points.points[i].z;
    if (!isInMap(pt_w)) continue;

    if (pt_w(2) < lowest) continue;
    
    Eigen::Vector3i idx;
    posToIndex(pt_w, idx);
    if (md_->occupancy_buffer_[toAddress(idx)] > mp_->min_occupancy_log_)
      md_->target_buffer_[toAddress(idx)] = 1;
  }

  return;
}

void SDFMap::clearAndMapEnvironment(const pcl::PointCloud<pcl::PointXYZ>& points, const int& point_num, const Eigen::Vector3d& camera_pos)
{
  double lowest = mp_->ground_height_ + 0.3;
  
  // * clear old environment points
  for (int i=0; i<(int)md_->env_buffer_.size(); ++i)
    md_->env_buffer_[i] = 0;
  
  // * map new environment points
  if (point_num == 0) return;
  Eigen::Vector3d pt_w;

  for (int i=0; i<point_num; ++i)
  {
    pt_w << points.points[i].x, points.points[i].y, points.points[i].z;
    if (!isInMap(pt_w)) continue;

    if (pt_w(2) < lowest) continue;
    
    Eigen::Vector3i idx;
    posToIndex(pt_w, idx);
    if (md_->occupancy_buffer_[toAddress(idx)] > mp_->min_occupancy_log_)
      md_->env_buffer_[toAddress(idx)] = 1;
  }

  return;
}

void SDFMap::loadInheritMap()
{
  string map_path = ros::package::getPath("plan_env") + "/inherit_map";
  string occ_file = map_path + "/occupied.pcd";
  string free_file = map_path + "/free.pcd";

  pcl::PointCloud<pcl::PointXYZ>::Ptr occ_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr free_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile(occ_file, *occ_cloud);
  pcl::io::loadPCDFile(free_file, *free_cloud);

  cout << "inherited occupied voxels: " << occ_cloud->size() << endl;
  cout << "inherited free voxels: " << free_cloud->size() << endl;

  // occupied states
  for (auto pt : occ_cloud->points)
  {
    Eigen::Vector3d pt_w(pt.x, pt.y, pt.z);
    Eigen::Vector3i idx;
    posToIndex(pt_w, idx);
    if (isInMap(idx))
    {
      md_->occupancy_buffer_[toAddress(idx)] = mp_->clamp_max_log_;
    }
  }

  // free states
  for (auto pt : free_cloud->points)
  {
    Eigen::Vector3d pt_w(pt.x, pt.y, pt.z);
    Eigen::Vector3i idx;
    posToIndex(pt_w, idx);
    if (isInMap(idx))
    {
      md_->occupancy_buffer_[toAddress(idx)] = 0.5*(mp_->min_occupancy_log_ + mp_->clamp_min_log_);
    }
  }

  // inflate cur map
  int inf_step = ceil(mp_->obstacles_inflation_ / mp_->resolution_);
  int inf_z_step = ceil(mp_->obs_inf_z_ / mp_->resolution_);
  vector<Eigen::Vector3i> inf_pts;
  for (int x = mp_->box_min_(0) /* + 1 */; x < mp_->box_max_(0); ++x)
    for (int y = mp_->box_min_(1) /* + 1 */; y < mp_->box_max_(1); ++y)
      for (int z = mp_->box_min_(2) /* + 1 */; z < mp_->box_max_(2); ++z)
      {
        if (getOccupancy(Eigen::Vector3i(x, y, z)) == OCCUPANCY::OCCUPIED)
        {
          inf_pts.clear();
          inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_z_step, inf_pts);
          for (auto inf_pt : inf_pts)
          {
            int idx_inf = toAddress(inf_pt);
            if (idx_inf >= 0 && idx_inf < mp_->map_voxel_num_(0) * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2))
            {
              md_->occupancy_buffer_inflate_[idx_inf] = 1;
            }
          }
        }
      }

  ROS_INFO("\033[33m[Inheritance Map in FC-Vision] Imported! \033[32m");

  return;
}

void SDFMap::loadInheritOffset()
{
  string offset_file = ros::package::getPath("plan_env") + "/inherit_map/offset.yaml";

  YAML::Node config = YAML::LoadFile(offset_file);
  if (config["map_x_offset"] && config["map_y_offset"]) 
  {
    double inherited_x_offset = config["map_x_offset"].as<double>();
    double inherited_y_offset = config["map_y_offset"].as<double>();
    this->cur_x_ = inherited_x_offset;
    this->cur_y_ = inherited_y_offset;

    cout << "Map Offset: " << inherited_x_offset << ", " << inherited_y_offset << endl;

    ROS_INFO("\033[33m[Inheritance Offset in FC-Vision] Reset! \033[32m");
  } 
  else 
  {
    cerr << "Error: Missing map offsets!" << endl;
  }

  return;
}

void SDFMap::getOccPcd(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  pcl::PointXYZ pt;
  for (int x = mp_->box_min_(0) /* + 1 */; x < mp_->box_max_(0); ++x)
    for (int y = mp_->box_min_(1) /* + 1 */; y < mp_->box_max_(1); ++y)
      for (int z = mp_->box_min_(2) /* + 1 */; z < mp_->box_max_(2); ++z)
      {
        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);

        if (getOccupancy(pos) == SDFMap::OCCUPANCY::OCCUPIED)
        {
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud->points.push_back(pt);
        }
      }

  return;
}

void SDFMap::getFreePcd(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  pcl::PointXYZ pt;
  for (int x = mp_->box_min_(0) /* + 1 */; x < mp_->box_max_(0); ++x)
    for (int y = mp_->box_min_(1) /* + 1 */; y < mp_->box_max_(1); ++y)
      for (int z = mp_->box_min_(2) /* + 1 */; z < mp_->box_max_(2); ++z)
      {
        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);

        if (getOccupancy(pos) == SDFMap::OCCUPANCY::FREE)
        {
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud->points.push_back(pt);
        }
      }

  return;
}

double SDFMap::getResolution() 
{
  return mp_->resolution_;
}

int SDFMap::getVoxelNum() 
{
  return mp_->map_voxel_num_[0] * mp_->map_voxel_num_[1] * mp_->map_voxel_num_[2];
}

void SDFMap::getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size) 
{
  ori = mp_->map_origin_, size = mp_->map_size_;
}

void SDFMap::getBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax) 
{
  bmin = mp_->box_mind_;
  bmax = mp_->box_maxd_;
}

void SDFMap::getUpdatedBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax, bool reset) 
{
  bmin = md_->update_min_;
  bmax = md_->update_max_;
  if (reset) md_->reset_updated_box_ = true;
}

void SDFMap::getGlobalBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax)
{
  bmin = mp_->map_min_boundary_;
  bmax = mp_->map_max_boundary_;
  
  return;
}

void SDFMap::getOccMap(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  double lowest = mp_->ground_height_ + 1.0;
  
  pcl::PointXYZ pt;
  for (int x = mp_->box_min_(0); x < mp_->box_max_(0); ++x)
    for (int y = mp_->box_min_(1); y < mp_->box_max_(1); ++y)
      for (int z = mp_->box_min_(2); z < mp_->box_max_(2); ++z)
      {
        if (md_->occupancy_buffer_[toAddress(x, y, z)] > mp_->min_occupancy_log_)
        {
          Eigen::Vector3d pos;
          indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2)>lowest)
          {
            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
          }
          cloud->points.push_back(pt);
        }
      }

  return;
}

double SDFMap::getDistWithGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad) {
  if (!isInMap(pos)) {
    grad.setZero();
    return 0;
  }

  /* trilinear interpolation */
  Eigen::Vector3d pos_m = pos - 0.5 * mp_->resolution_ * Eigen::Vector3d::Ones();
  Eigen::Vector3i idx;
  posToIndex(pos_m, idx);
  Eigen::Vector3d idx_pos, diff;
  indexToPos(idx, idx_pos);
  diff = (pos - idx_pos) * mp_->resolution_inv_;

  double values[2][2][2];
  for (int x = 0; x < 2; x++)
    for (int y = 0; y < 2; y++)
      for (int z = 0; z < 2; z++) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        values[x][y][z] = getDistance(current_idx);
      }

  double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
  double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
  double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
  double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
  double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
  double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
  double dist = (1 - diff[2]) * v0 + diff[2] * v1;

  grad[2] = (v1 - v0) * mp_->resolution_inv_;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * mp_->resolution_inv_;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
  grad[0] *= mp_->resolution_inv_;

  return dist;
}

double SDFMap::getDistWithGradHCMap(const Eigen::Vector3d& pos, Eigen::Vector3d& grad)
{
  if (!isInMap_hc(pos)) 
  {
    grad.setZero();
    return 0;
  }

  /* trilinear interpolation */
  Eigen::Vector3d pos_m = pos - 0.5 * hcmp_->resolution_ * Eigen::Vector3d::Ones();
  Eigen::Vector3i idx;
  posToIndex_hc(pos_m, idx);
  Eigen::Vector3d idx_pos, diff;
  indexToPos_hc(idx, idx_pos);
  diff = (pos - idx_pos) * hcmp_->resolution_inv_;

  double values[2][2][2];
  for (int x = 0; x < 2; x++)
    for (int y = 0; y < 2; y++)
      for (int z = 0; z < 2; z++) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        values[x][y][z] = getDistance_hc(current_idx);
      }

  double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
  double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
  double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
  double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
  double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
  double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
  double dist = (1 - diff[2]) * v0 + diff[2] * v1;

  grad[2] = (v1 - v0) * hcmp_->resolution_inv_;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * hcmp_->resolution_inv_;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
  grad[0] *= hcmp_->resolution_inv_;

  return dist;
}

}  // namespace fc_vision
// SDFMap
