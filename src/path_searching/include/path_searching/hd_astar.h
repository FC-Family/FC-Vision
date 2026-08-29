/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2025
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the header file of high-dim A* class, which implements
 *                   safe & visible path searching in FC-Vision.
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

#ifndef _HD_ASTAR_H
#define _HD_ASTAR_H

#include "active_perception/perception_utils.h"
#include "plan_env/sdf_map.h"
#include "plan_env/raycast.h"
#include "path_searching/matrix_hash.h"
#include "path_searching/path_tools.hpp"
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <string>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <queue>
#include <sstream>

using namespace std;
using std::unique_ptr;

class RayCaster;

namespace fc_vision {

class SDFMap;
class PerceptionUtils;

class HDNode 
{
public:
  Eigen::Vector3i index;
  Eigen::Vector3d position;
  Eigen::VectorXd pose; // x y z pitch yaw
  double g_score, f_score;
  HDNode* parent;

  /* -------------------- */
  HDNode() {
    parent = NULL;
  }
  ~HDNode(){};
};
typedef HDNode* HDNodePtr;

class HDNodeComparator 
{
public:
  bool operator()(HDNodePtr node1, HDNodePtr node2) 
  {
    return node1->f_score > node2->f_score;
  }
};

class HDAstar
{
public:
  HDAstar();
  ~HDAstar();
  enum { SUCCEED = 1, FAIL = 2 };

  void init(ros::NodeHandle& nh);
  void setMap(shared_ptr<SDFMap> map);
  void setOcclusion(Eigen::Matrix4Xd& occlusion);
  void reset();
  void setSearchStep(const int step);
  void setPathInterval(const double interval);
  bool preCheck(const Eigen::VectorXd& start_pt, const Eigen::VectorXd& end_pt);
  bool casVisEqSearch(const Eigen::VectorXd& start_pt, const Eigen::VectorXd& end_pt);
  bool casSafeSearch(const Eigen::VectorXd& start_pt, const Eigen::VectorXd& end_pt);
  int visEqSearch(const Eigen::VectorXd& start_pt, const Eigen::VectorXd& end_pt);
  int safeSearch(const Eigen::VectorXd& start_pt, const Eigen::VectorXd& end_pt);
  int hcSafeSearch(const Eigen::VectorXd& start_pt, const Eigen::VectorXd& end_pt, bool high_tolerance);
  vector<Eigen::VectorXd> getPath();
  static double pathLength(const vector<Eigen::VectorXd>& path);
  double pathOcclusionRate(const vector<Eigen::VectorXd>& path);

private:
  /* Func */
  void backTrack(const HDNodePtr& end_node, const Eigen::VectorXd& end);
  void posToIndex(const Eigen::Vector3d& pt, Eigen::Vector3i& idx);
  void indexToPos(const Eigen::Vector3i& idx, Eigen::Vector3d& pt);
  double getDiagHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2);
  void getAttitudeGap();
  struct NeighborOffset
  {
    Eigen::Vector3i idx_offset;
    Eigen::Vector3d pos_offset;
    double step_len;
  };
  void buildNeighborOffsets(int step);
  bool interpolatePitchYaw(Eigen::Vector3d& cur_node_pos, bool en_vis, Eigen::Vector2d& pitch_yaw);
  bool linePreCheck(bool en_visibility, bool en_hc);
  /* Utils */
  shared_ptr<SDFMap> map_;
  unique_ptr<PerceptionUtils> percep_utils_ = nullptr;
  unique_ptr<RayCaster> raycaster_ = nullptr;
  /* Param */
  int search_step_;
  double path_interval_;
  double resolution_, lambda_heu_, safe_height_;
  double max_vis_search_time_, max_safe_search_time_, max_hc_search_time_;
  int allocate_num_;
  double tie_breaker_, inv_resolution_;
  int use_node_num_, iter_num_;
  double early_terminate_cost_;
  Eigen::Vector3d origin_;
  Eigen::VectorXd start_pose_, end_pose_;
  double pitch_gap_, yaw_gap_;
  /* Data */
  vector<HDNodePtr> path_node_pool_;
  priority_queue<HDNodePtr, vector<HDNodePtr>, HDNodeComparator> open_set_;
  unordered_map<Eigen::Vector3i, HDNodePtr, matrix_hash<Eigen::Vector3i>> open_set_map_;
  unordered_map<Eigen::Vector3i, int, matrix_hash<Eigen::Vector3i>> close_set_map_;
  unordered_map<Eigen::Vector3i, std::pair<bool, Eigen::Vector2d>, matrix_hash<Eigen::Vector3i>> vis_cache_;
  std::vector<NeighborOffset> vis_neighbor_offsets_;
  int vis_neighbor_step_cache_ = -1;
  vector<Eigen::VectorXd> path_nodes_;
  Eigen::Matrix<double, 4, Eigen::Dynamic> occlusion_mat_;
  Eigen::Matrix<double, 5, Eigen::Dynamic> occlusion_results_;

};

} // namespace fc_vision

#endif
