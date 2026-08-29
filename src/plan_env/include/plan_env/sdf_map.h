/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Sept. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the header file of SDFMap class, which implements
 *                   dual volumetric mapping module in FC-Vision.
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

#ifndef _SDF_MAP_H
#define _SDF_MAP_H

#include "plan_env/map_ros.h"
#include "plan_env/raycast.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/convex_hull.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <tuple>
#include <queue>
#include <chrono>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>

using namespace std;
using std::shared_ptr;
using std::unique_ptr;

namespace cv {
class Mat;
}

class RayCaster;

namespace fc_vision {
struct MapParam;
struct MapData;
struct HCMapParam;
struct HCMapData;
class MapROS;

class SDFMap {
public:
  SDFMap();
  ~SDFMap();

  enum OCCUPANCY { UNKNOWN, FREE, OCCUPIED, HC_INTERNAL };

  Eigen::Vector3d min_bound, max_bound;

  void initMap(ros::NodeHandle& nh);
  void setXYoffset(double x_offset, double y_offset);
  void getXYoffset(double& x_offset, double& y_offset);
  void inputPointCloud(const pcl::PointCloud<pcl::PointXYZ>& points, const int& point_num,
                       const Eigen::Vector3d& camera_pos, bool air);
  void updateMapAttributes();
  void inputUserObs(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  void inputFreePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& points);

  void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
  void posToIndex_hc(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
  void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
  void indexToPos_hc(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
  void boundIndex(Eigen::Vector3i& id);
  int toAddress(const Eigen::Vector3i& id);
  int toAddress(const int& x, const int& y, const int& z);
  int toAddress_hc(const Eigen::Vector3i& id);
  int toAddress_hc(const int& x, const int& y, const int& z);
  bool isInMap(const Eigen::Vector3d& pos);
  bool isInMap(const Eigen::Vector3i& idx);
  bool isInBox(const Eigen::Vector3i& id);
  bool isInBox(const Eigen::Vector3d& pos);
  void boundBox(Eigen::Vector3d& low, Eigen::Vector3d& up);
  int getOccupancy(const Eigen::Vector3d& pos);
  int getOccupancy(const Eigen::Vector3i& id);
  bool getEnv(const Eigen::Vector3d& pos);
  bool getEnv(const Eigen::Vector3i& id);
  int getInflateOccupancy(const Eigen::Vector3d& pos);
  int getInflateOccupancy(const Eigen::Vector3i& id);
  double getDistance(const Eigen::Vector3d& pos);
  double getDistance(const Eigen::Vector3i& id);
  double getDistWithGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad);
  void updateESDF3d();
  void resetBuffer();
  void resetBuffer(const Eigen::Vector3d& min, const Eigen::Vector3d& max);
  void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size);
  void getBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax);
  void getUpdatedBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax, bool reset = false);
  double getResolution();
  int getVoxelNum();
  void getOccMap(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  void invAddress(int& id, Eigen::Vector3i& idx);
  void getLocalEnv(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, Eigen::Vector3d& min, Eigen::Vector3d& max);
  void getLocalOccEnv(pcl::PointCloud<pcl::PointXYZ>::Ptr& occ_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& env_cloud, Eigen::Vector3d& min, Eigen::Vector3d& max);
  bool getSafety(const Eigen::Vector3d& pos, const double x_range, const double y_range, const double z_range);
  void getGlobalBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax);

  /* Utils */
  unique_ptr<MapParam> mp_;

  // ! /* mesh prior related */ --------------------------------------------
  bool zFlag; double zPos;
  int checkSize, inflate_num;
  void initHC(ros::NodeHandle& nh);
  void initHCMap(ros::NodeHandle& nh, pcl::PointCloud<pcl::PointXYZ>::Ptr& model);
  void InternalSpace(map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>& seg_cloud, Eigen::MatrixXd& vertices, map<int, vector<int>>& segments);
  vector<Eigen::Vector3d> points_in_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud, Eigen::Vector3d& point_on_plane, Eigen::Vector3d& plane_normal, double thickness);
  bool isInMap_hc(const Eigen::Vector3d& pos);
  bool isInMap_hc(const Eigen::Vector3i& idx);
  bool safety_check(Eigen::Vector3d& pos);
  bool safety_check(Eigen::Vector3i& id);
  bool getHCocc(Eigen::Vector3d& pos);
  bool getHCocc(Eigen::Vector3i& id);
  bool freeCheck(Eigen::Vector3d& pos);
  bool freeCheck(Eigen::Vector3i& id);
  int get_Internal(const Eigen::Vector3d& pos);
  int get_Internal(const Eigen::Vector3i& id);

  int getInflateOccupancy_hc(const Eigen::Vector3d& pos);
  int getInflateOccupancy_hc(const Eigen::Vector3i& id);

  void setObserved(Eigen::Vector3d& p_w);

  void setInternal(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const unordered_map<int, Eigen::Vector3d>& cor_inliers);
  void resetHCMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& model);
  void resetHCVirtualBound(const double min_x, const double max_x, const double min_y, const double max_y);
  void getGlobalHCMap(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  void getLocalHCMap(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, Eigen::Vector3d& min, Eigen::Vector3d& max);
  bool getHCSafety(const Eigen::Vector3d& pos, const double x_range, const double y_range, const double z_range);
  double getDistance_hc(const Eigen::Vector3d& pos);
  double getDistance_hc(const Eigen::Vector3i& id);
  void updateESDF3dHCMap();
  double getDistWithGradHCMap(const Eigen::Vector3d& pos, Eigen::Vector3d& grad);

  unique_ptr<HCMapParam> hcmp_;
  unique_ptr<HCMapData> hcmd_;
  unique_ptr<RayCaster> internal_cast_;
  // ! /* mesh prior related */ --------------------------------------------

  // ! /* debug */ --------------------------------------------

  bool isPointInConvexHull(const Eigen::Vector3d& point, const Eigen::MatrixX4d& H);
  Eigen::Vector4d computePlaneEquation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_hull, const pcl::Vertices& polygon);
  void setOccManual(const vector<Eigen::Vector3d>& pseudo_points);
  void setFreeManual();
  void setTargetManual(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  void updateESDF3dManual();

  // ! /* debug */ --------------------------------------------

  // * inheritance flight
  void loadInheritMap();
  void loadInheritOffset();
  void getOccPcd(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  void getFreePcd(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

private:
  void clearAndInflateLocalMap();
  void clearAndMapTarget(const pcl::PointCloud<pcl::PointXYZ>& points, const int& point_num, const Eigen::Vector3d& camera_pos);
  void clearAndMapEnvironment(const pcl::PointCloud<pcl::PointXYZ>& points, const int& point_num, const Eigen::Vector3d& camera_pos);
  void inflatePoint(const Eigen::Vector3i& pt, int step, int z_step, vector<Eigen::Vector3i>& pts);
  void setCacheOccupancy(const int& adr, const int& occ);
  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt);
  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);
  
  bool airsim_flag_ = false;
  double z_min = 0.0;
  double cur_x_ = 0.0, cur_y_ = 0.0;
  double ground_threshold_ = 0.0, omit_height_ = 0.15;
  double attri_threshold_ = 0.5;

  bool en_attributes_ = false;
  bool en_inherit_ = false;
  bool en_user_ = false;

  int belief_count_ = 1;

  unique_ptr<MapData> md_;
  unique_ptr<MapROS> mr_;
  unique_ptr<RayCaster> caster_;

  friend MapROS;

public:
  typedef std::shared_ptr<SDFMap> Ptr;
  typedef std::unique_ptr<SDFMap> HCPtr;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct MapParam {
  // map properties
  Eigen::Vector3d map_origin_, map_size_;
  Eigen::Vector3d map_min_boundary_, map_max_boundary_;
  Eigen::Vector3i map_voxel_num_;
  double resolution_, resolution_inv_;
  double obstacles_inflation_, target_obstacles_inflation_, obs_inf_z_, target_obs_inf_z_;
  double virtual_ceil_height_, ground_height_;
  Eigen::Vector3i box_min_, box_max_;
  Eigen::Vector3d box_mind_, box_maxd_;
  double default_dist_;
  bool optimistic_, signed_dist_;
  // map fusion
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;  // occupancy probability
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_, min_occupancy_log_;  // logit
  double max_ray_length_;
  double local_bound_inflate_;
  int local_map_margin_;
  double unknown_flag_;
  double x_origin_back_, y_origin_back_;
};

struct HCMapParam
{
  double resolution_, resolution_inv_, proj_interval, thickness, size_inflate;
  double checkScale;
  Eigen::Vector3d map_origin_, map_size_;
  Eigen::Vector3d map_min_boundary_, map_max_boundary_;
  Eigen::Vector3i map_voxel_num_;
  Eigen::Vector3i box_min_, box_max_;
  Eigen::Vector3d box_mind_, box_maxd_;
  Eigen::Vector3i map_origin_idx_;
  double x_size_, y_size_, z_size_, z_min_;
};

struct MapData {
  // Occupancy
  vector<double> occupancy_buffer_;
  vector<char> occupancy_buffer_inflate_;
  // target & environment flags
  vector<char> target_buffer_;
  vector<char> env_buffer_;
  // ESDF
  vector<double> distance_buffer_neg_;
  vector<double> distance_buffer_;
  vector<double> tmp_buffer1_;
  vector<double> tmp_buffer2_;
  // data for updating
  vector<short> count_hit_, count_miss_;
  vector<char> flag_rayend_;
  char raycast_num_;
  queue<int> cache_voxel_;
  Eigen::Vector3i local_bound_min_, local_bound_max_;
  Eigen::Vector3d update_min_, update_max_;
  bool reset_updated_box_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct HCMapData
{
  vector<char> occupancy_buffer_hc_;
  vector<char> occupancy_inflate_buffer_hc_;
  vector<char> occupancy_buffer_internal_;
  vector<char> user_obs_buffer_;
  vector<char> count_observed_buffer_;
  pcl::PointCloud<pcl::PointXYZ> occ_cloud;
  pcl::PointCloud<pcl::PointXYZ> internal_cloud;
  vector<bool> seg_occ_visited_buffer_;
  // ESDF
  vector<double> distance_buffer_neg_hc_;
  vector<double> distance_buffer_hc_;
  vector<double> tmp_buffer1_hc_;
  vector<double> tmp_buffer2_hc_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

inline void SDFMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i)
    id(i) = floor((pos(i) - mp_->map_origin_(i)) * mp_->resolution_inv_);
}

inline void SDFMap::posToIndex_hc(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i)
    id(i) = floor((pos(i) - hcmp_->map_origin_(i)) * hcmp_->resolution_inv_);
}

inline void SDFMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
  for (int i = 0; i < 3; ++i)
    pos(i) = (id(i) + 0.5) * mp_->resolution_ + mp_->map_origin_(i);
}

inline void SDFMap::indexToPos_hc(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
  for (int i = 0; i < 3; ++i)
    pos(i) = (id(i) + 0.5) * hcmp_->resolution_ + hcmp_->map_origin_(i);
}


inline void SDFMap::boundIndex(Eigen::Vector3i& id) {
  Eigen::Vector3i id1;
  id1(0) = max(min(id(0), mp_->map_voxel_num_(0) - 1), 0);
  id1(1) = max(min(id(1), mp_->map_voxel_num_(1) - 1), 0);
  id1(2) = max(min(id(2), mp_->map_voxel_num_(2) - 1), 0);
  id = id1;
}

inline int SDFMap::toAddress(const int& x, const int& y, const int& z) {
  return x * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2) + y * mp_->map_voxel_num_(2) + z;
}

inline int SDFMap::toAddress(const Eigen::Vector3i& id) {
  return toAddress(id[0], id[1], id[2]);
}

inline int SDFMap::toAddress_hc(const int& x, const int& y, const int& z) {
  int xi = x-hcmp_->map_origin_idx_(0); int yi = y-hcmp_->map_origin_idx_(1); int zi = z-hcmp_->map_origin_idx_(2); 
  return xi * hcmp_->map_voxel_num_(1) * hcmp_->map_voxel_num_(2) + yi * hcmp_->map_voxel_num_(2) + zi;
}

inline int SDFMap::toAddress_hc(const Eigen::Vector3i& id) {
  return toAddress_hc(id[0], id[1], id[2]);
}

inline void SDFMap::invAddress(int& id, Eigen::Vector3i& idx)
{
  int nx = mp_->map_voxel_num_(1);
  int ny = mp_->map_voxel_num_(2);

  int z = id % ny;
  int y = (id / ny) % nx;
  int x = id / (nx * ny);

  idx(0) = x;
  idx(1) = y;
  idx(2) = z;

  return;
}

inline bool SDFMap::isInMap(const Eigen::Vector3d& pos) {
  if (pos(0) < mp_->map_min_boundary_(0) + 1e-4 || pos(1) < mp_->map_min_boundary_(1) + 1e-4 ||
      pos(2) < mp_->map_min_boundary_(2) + 1e-4)
    return false;
  if (pos(0) > mp_->map_max_boundary_(0) - 1e-4 || pos(1) > mp_->map_max_boundary_(1) - 1e-4 ||
      pos(2) > mp_->map_max_boundary_(2) - 1e-4)
    return false;
  return true;
}

inline bool SDFMap::isInMap(const Eigen::Vector3i& idx) {
  if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0) return false;
  if (idx(0) > mp_->map_voxel_num_(0) - 1 || idx(1) > mp_->map_voxel_num_(1) - 1 ||
      idx(2) > mp_->map_voxel_num_(2) - 1)
    return false;
  return true;
}

inline bool SDFMap::isInBox(const Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i) {
    if (id[i] < mp_->box_min_[i] || id[i] >= mp_->box_max_[i]) {
      return false;
    }
  }
  return true;
}

inline bool SDFMap::isInBox(const Eigen::Vector3d& pos) {
  for (int i = 0; i < 3; ++i) {
    if (pos[i] <= mp_->box_mind_[i] || pos[i] >= mp_->box_maxd_[i]) {
      return false;
    }
  }
  return true;
}

inline void SDFMap::boundBox(Eigen::Vector3d& low, Eigen::Vector3d& up) {
  for (int i = 0; i < 3; ++i) {
    low[i] = max(low[i], mp_->box_mind_[i]);
    up[i] = min(up[i], mp_->box_maxd_[i]);
  }
}

inline int SDFMap::getOccupancy(const Eigen::Vector3i& id) {
  if (!isInMap(id)) return -1;
  double occ = md_->occupancy_buffer_[toAddress(id)];
  if (occ < mp_->clamp_min_log_ - 1e-3) return UNKNOWN;
  if (occ > mp_->min_occupancy_log_) return OCCUPIED;
  return FREE;
}

inline int SDFMap::getOccupancy(const Eigen::Vector3d& pos) {
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return getOccupancy(id);
}

inline bool SDFMap::getEnv(const Eigen::Vector3i& id) 
{
  if (!isInMap(id)) return false;
  return md_->env_buffer_[toAddress(id)] == 1;
}

inline bool SDFMap::getEnv(const Eigen::Vector3d& pos) 
{
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return getEnv(id);
}

inline bool SDFMap::isInMap_hc(const Eigen::Vector3d& pos) {
  if (pos(0) < hcmp_->map_min_boundary_(0) + 1e-4 || pos(1) < hcmp_->map_min_boundary_(1) + 1e-4 ||
      pos(2) < hcmp_->map_min_boundary_(2) + 1e-4)
    return false;
  if (pos(0) > hcmp_->map_max_boundary_(0) - 1e-4 || pos(1) > hcmp_->map_max_boundary_(1) - 1e-4 ||
      pos(2) > hcmp_->map_max_boundary_(2) - 1e-4)
    return false;
  return true;
}

inline bool SDFMap::isInMap_hc(const Eigen::Vector3i& idx) {
  if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0) return false;
  if (idx(0) > hcmp_->map_voxel_num_(0) - 1 || idx(1) > hcmp_->map_voxel_num_(1) - 1 ||
      idx(2) > hcmp_->map_voxel_num_(2) - 1)
    return false;
  return true;
}

inline bool SDFMap::safety_check(Eigen::Vector3i& id)
{
  if (!isInMap_hc(id)) return false;
  
  if (hcmd_->occupancy_buffer_hc_[toAddress_hc(id)] == 1 || hcmd_->occupancy_buffer_internal_[toAddress_hc(id)] == 1 || hcmd_->occupancy_inflate_buffer_hc_[toAddress_hc(id)] == 1)
    return false;

  return true;
}

inline bool SDFMap::safety_check(Eigen::Vector3d& pos)
{
  // if (zFlag == true)
  // {
  //   if (pos(2) < zPos)
  //     return false;
  // }

  Eigen::Vector3i id;
  posToIndex_hc(pos, id);
  return safety_check(id);
}

inline bool SDFMap::getHCocc(Eigen::Vector3i& id)
{
  if (!isInMap_hc(id)) return false;

  if (hcmd_->occupancy_buffer_hc_[toAddress_hc(id)] == 1)
    return true;
  
  return false;
}

inline bool SDFMap::getHCocc(Eigen::Vector3d& pos)
{
  Eigen::Vector3i id;
  posToIndex_hc(pos, id);
  return getHCocc(id);
}

inline bool SDFMap::freeCheck(Eigen::Vector3i& id)
{
  // HCMap inspection helpers
  if (!isInMap_hc(id)) return false;
  if (hcmd_->occupancy_buffer_hc_[toAddress_hc(id)] == 2)
    return true;
  
  return false;
}

inline bool SDFMap::freeCheck(Eigen::Vector3d& pos)
{
  // HCMap inspection helpers
  Eigen::Vector3i id;
  posToIndex_hc(pos, id);
  return getHCocc(id);
}

inline int SDFMap::get_Internal(const Eigen::Vector3i& id)
{
  if (!isInMap_hc(id)) return -1;
  if (hcmd_->occupancy_buffer_internal_[toAddress_hc(id)] == 1) 
    return HC_INTERNAL;
  
  return -1;
}

inline int SDFMap::get_Internal(const Eigen::Vector3d& pos)
{
  Eigen::Vector3i id;
  posToIndex_hc(pos, id);
  return get_Internal(id);
}

inline int SDFMap::getInflateOccupancy(const Eigen::Vector3i& id) {
  if (!isInMap(id)) return -1;
  return int(md_->occupancy_buffer_inflate_[toAddress(id)]);
}

inline int SDFMap::getInflateOccupancy(const Eigen::Vector3d& pos) {
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return getInflateOccupancy(id);
}

inline int SDFMap::getInflateOccupancy_hc(const Eigen::Vector3i& id) {
  if (!isInMap_hc(id)) return -1;
  return int(hcmd_->occupancy_inflate_buffer_hc_[toAddress_hc(id)]);
}

inline int SDFMap::getInflateOccupancy_hc(const Eigen::Vector3d& pos) {
  Eigen::Vector3i id;
  posToIndex_hc(pos, id);
  return getInflateOccupancy_hc(id);
}

inline double SDFMap::getDistance(const Eigen::Vector3i& id) {
  if (!isInMap(id)) return -1;
  return md_->distance_buffer_[toAddress(id)];
}

inline double SDFMap::getDistance(const Eigen::Vector3d& pos) {
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return getDistance(id);
}

inline double SDFMap::getDistance_hc(const Eigen::Vector3i& id) {
  if (!isInMap_hc(id)) return -1;
  return hcmd_->distance_buffer_hc_[toAddress_hc(id)];
}

inline double SDFMap::getDistance_hc(const Eigen::Vector3d& pos) {
  Eigen::Vector3i id;
  posToIndex_hc(pos, id);
  return getDistance_hc(id);
}

inline void SDFMap::inflatePoint(const Eigen::Vector3i& pt, int step, int z_step, vector<Eigen::Vector3i>& pts) {
  int num = 0;

  /* ---------- + shape inflate ---------- */
  // for (int x = -step; x <= step; ++x)
  // {
  //   if (x == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
  // }
  // for (int y = -step; y <= step; ++y)
  // {
  //   if (y == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
  // }
  // for (int z = -1; z <= 1; ++z)
  // {
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
  // }

  /* ---------- all inflate ---------- */
  for (int x = -step; x <= step; ++x)
    for (int y = -step; y <= step; ++y)
      for (int z = -z_step; z <= z_step; ++z) {
        if (x == 0 && y == 0 && z == 0)
          continue;
        pts.push_back(Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z));
      }
}

// ! /* debug */ --------------------------------------------
inline bool SDFMap::isPointInConvexHull(const Eigen::Vector3d& point, const Eigen::MatrixX4d& H)
{ 
  for (int i = 0; i < H.rows(); ++i)
  {
    double dist = point(0)*H(i, 0) + point(1)*H(i, 1) + point(2)*H(i, 2) + H(i, 3);
    if (dist > 0)
      return false;
  }

  return true;
}

inline Eigen::Vector4d SDFMap::computePlaneEquation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_hull, const pcl::Vertices& polygon)
{
  // Each row of hPoly is defined by h0, h1, h2, h3 as
  // h0*x + h1*y + h2*z + h3 <= 0

  Eigen::Vector3d v0 = cloud_hull->points[polygon.vertices[0]].getVector3fMap().cast<double>();
  Eigen::Vector3d v1 = cloud_hull->points[polygon.vertices[1]].getVector3fMap().cast<double>();
  Eigen::Vector3d v2 = cloud_hull->points[polygon.vertices[2]].getVector3fMap().cast<double>();

  Eigen::Vector3d normal = (v1 - v0).cross(v2 - v0);
  double d = -normal.dot(v0);

  Eigen::Vector3d test_point;
  for (int i=0; i<(int)cloud_hull->points.size(); ++i)
  {
    if (i == polygon.vertices[0] || i == polygon.vertices[1] || i == polygon.vertices[2])
      continue;
    
    pcl::PointXYZ pt = cloud_hull->points[i];
    Eigen::Vector3d temp(pt.x, pt.y, pt.z);
    if (normal.dot(temp) + d > -1e-3 && normal.dot(temp) + d < 1e-3) continue;

    test_point = pt.getVector3fMap().cast<double>();
    break;
  }

  if (normal.dot(test_point) + d > 0) {
      normal = -normal;
      d = -d;
  }

  return Eigen::Vector4d(normal[0], normal[1], normal[2], d);
}

inline void SDFMap::setOccManual(const vector<Eigen::Vector3d>& pseudo_points)
{
  // 1. generate convex hull
  if ((int)pseudo_points.size() < 4)
  {
    ROS_ERROR("The number of pseudo points is less than 4.");
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  for (const auto& p : pseudo_points)
    cloud->points.emplace_back(p.x(), p.y(), p.z());
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::ConvexHull<pcl::PointXYZ> chull;
  vector<pcl::Vertices> polygons;
  chull.setInputCloud(cloud);
  chull.setDimension(3);
  chull.reconstruct(*cloud_hull, polygons);

  if (cloud_hull->points.empty()) 
  {
    std::cerr << "Error: Convex hull could not be computed." << std::endl;
  }

  Eigen::MatrixX4d H(polygons.size(), 4);
  for (size_t i = 0; i < polygons.size(); ++i)
    H.row(i) = computePlaneEquation(cloud_hull, polygons[i]);
  
  // 2. calculate bbox
  Eigen::Vector3d min_pt = pseudo_points[0], max_pt = pseudo_points[0];
  for (const auto& pt : pseudo_points)
  {
    min_pt = min_pt.cwiseMin(pt);
    max_pt = max_pt.cwiseMax(pt);
  }

  // 3. generate uniform points
  int num = 0;
  vector<Eigen::Vector3d> grid_points;
  for (double x = min_pt.x(); x <= max_pt.x(); x += 0.5*mp_->resolution_)
    for (double y = min_pt.y(); y <= max_pt.y(); y += 0.5*mp_->resolution_)
      for (double z = min_pt.z(); z <= max_pt.z(); z += 0.5*mp_->resolution_)
      {
        num++;
        Eigen::Vector3d grid_point(x, y, z);
        if (isPointInConvexHull(grid_point, H) == true)
          grid_points.push_back(grid_point);
      }

  for (auto& pt : grid_points)
  {
    if (!isInMap(pt)) continue;
    Eigen::Vector3i id;
    posToIndex(pt, id);
    md_->occupancy_buffer_[toAddress(id)] = mp_->min_occupancy_log_+0.01;
    md_->occupancy_buffer_inflate_[toAddress(id)] = 1;
    // * set this voxel as environment
    md_->env_buffer_[toAddress(id)] = 1;

    int inf_num = ceil(mp_->obstacles_inflation_ / mp_->resolution_);
    int inf_num_z = ceil(mp_->obs_inf_z_ / mp_->resolution_);

    for (int i=-inf_num; i<inf_num+1; i=i+1)
        for (int j=-inf_num; j<inf_num+1; j=j+1)
          for (int k=-inf_num_z; k<inf_num_z+1; k=k+1)
          {
            if (i == 0 && j == 0 && k == 0) {
              continue;
            }
            Eigen::Vector3i neighbor;
            neighbor(0)=id.x() + i; neighbor(1)=id.y() + j; neighbor(2)=id.z() + k;

            if (!isInMap(neighbor)) continue;
            md_->occupancy_buffer_inflate_[toAddress(neighbor)] = 1;
          }
  }
}

inline void SDFMap::setFreeManual()
{
  int voxel_size = md_->occupancy_buffer_.size();
  double free_log = 0.5 * (mp_->clamp_min_log_ + mp_->min_occupancy_log_);
  md_->occupancy_buffer_ = vector<double>(voxel_size, free_log);

  return;
}

inline void SDFMap::setTargetManual(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  for (int i=0; i<(int)cloud->points.size(); ++i)
  {
    Eigen::Vector3d pt(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
    if (!isInMap(pt)) continue;
    Eigen::Vector3i id;
    posToIndex(pt, id);
    md_->occupancy_buffer_[toAddress(id)] = mp_->min_occupancy_log_+0.01;
    md_->occupancy_buffer_inflate_[toAddress(id)] = 1;
    md_->target_buffer_[toAddress(id)] = 1;

    int inf_num = ceil(mp_->target_obstacles_inflation_ / mp_->resolution_);
    int inf_num_z = ceil(mp_->target_obs_inf_z_ / mp_->resolution_);

    for (int i=-inf_num; i<inf_num+1; i=i+1)
        for (int j=-inf_num; j<inf_num+1; j=j+1)
          for (int k=-inf_num_z; k<inf_num_z+1; k=k+1)
          {
            if (i == 0 && j == 0 && k == 0) {
              continue;
            }
            Eigen::Vector3i neighbor;
            neighbor(0)=id.x() + i; neighbor(1)=id.y() + j; neighbor(2)=id.z() + k;

            if (!isInMap(neighbor)) continue;
            md_->occupancy_buffer_inflate_[toAddress(neighbor)] = 1;
          }
  }

  return;
}

inline void SDFMap::updateESDF3dManual()
{
  md_->local_bound_min_ = mp_->box_min_ + Eigen::Vector3i(2, 2, 2);
  md_->local_bound_max_ = mp_->box_max_ - Eigen::Vector3i(2, 2, 2);
  this->updateESDF3d();

  return;
}

}
#endif
