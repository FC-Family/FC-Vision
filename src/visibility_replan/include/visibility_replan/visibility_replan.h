/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Mar. 2025
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the header file of visibility_replan class, which is 
 *                   visibility-aware replanning in FC-Vision.
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

#ifndef _VISIBILITY_REPLAN_H_
#define _VISIBILITY_REPLAN_H_

#include "active_perception/perception_utils.h"
#include "plan_env/raycast.h"
#include "plan_env/sdf_map.h"
#include "path_searching/geometry_utils.hpp"
#include "path_searching/path_tools.hpp"
#include "vis_utils/planning_visualization.h"
#include "sop/sop.h"
#include "path_searching/hd_astar.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <limits>
#include <chrono>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <yaml-cpp/yaml.h>

using namespace std;
using std::unique_ptr;
using std::shared_ptr;

class RayCaster;

namespace fc_vision
{

class SDFMap;
class PerceptionUtils;
class PlanningVisualization;
class SOP;
class HDAstar;

class VisibilityReplan
{

struct VectorXdCompare 
{
  bool operator()(const Eigen::VectorXd& v1, const Eigen::VectorXd& v2) const 
  {
    int size = v1.size();
    if (v1.size() != v2.size()) 
    {
      return v1.size() < v2.size();
    }

    for (int i = 0; i < size; ++i) 
    {
      if (v1[i] < v2[i]) return true;
      if (v1[i] > v2[i]) return false;
    }
    
    return false;
  }
};

struct PolarCoord
{
  double r, theta, phi;
  Eigen::Vector3d vec_xyz;
  Eigen::Vector3d vec_dir;
  Eigen::VectorXd vec_vp;
  double push_dist;
  double visible_proportion;
};

struct PlanDataWrapper
{
  // * Extended Path
  vector<Eigen::VectorXd> ext_path;
  vector<bool> ext_indi;
  map<Eigen::VectorXd, bool, VectorXdCompare> ext_table;

  // * Optimization Bounds
  Eigen::Vector3d min_bound, max_bound;
  
  // * Perception
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, env_cloud;
  Eigen::Matrix4Xd target_mat, env_mat; 

  // * Planning
  vector<bool> qualified_states;
  vector<double> vis_rates;
  vector<Eigen::Matrix4Xd> vis_st_set;
  vector<vector<int>> vis_st_set_idx;
  vector<Eigen::VectorXd> opted_vps, qual_vps;
  vector<bool> target_cover_states;
  vector<Eigen::VectorXd> all_vps;

  // * Viewpoint Optimization
  vector<PolarCoord> local_samples_template;
  vector<PolarCoord> independent_samples_group;
  Eigen::VectorXd dl_vector;

  // * Set Covering Problem
  vector<int> uncovered_idx;
  vector<PolarCoord> open_pool;
  vector<bool> U_states, S_states;
  Eigen::MatrixXi U_S_mat;
  vector<Eigen::VectorXd> S_set;

  // * Reset
  void reset()
  {
    ext_path.clear();
    ext_indi.clear();
    ext_table.clear();
    min_bound = Eigen::Vector3d::Constant(numeric_limits<double>::max());
    max_bound = Eigen::Vector3d::Constant(numeric_limits<double>::lowest());
    target_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    env_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    target_mat.resize(4, 0);
    env_mat.resize(4, 0);
    qualified_states.clear();
    vis_rates.clear();
    vis_st_set.clear();
    vis_st_set_idx.clear();
    opted_vps.clear();
    qual_vps.clear();
    target_cover_states.clear();
    all_vps.clear();
    local_samples_template.clear();
    independent_samples_group.clear();
    dl_vector.resize(0);
    open_pool.clear();
    uncovered_idx.clear();
    U_states.clear();
    S_states.clear();
    U_S_mat.resize(0, 0);
    S_set.clear();
  }
};

public:
    VisibilityReplan(){
    }
    ~VisibilityReplan(){
    }
    /* Func */
    void init(ros::NodeHandle& nh);
    void setMap(shared_ptr<SDFMap> map);
    void setInputPath(vector<Eigen::VectorXd>& path);
    void replan();
    void getOutputPath(vector<Eigen::VectorXd>& path, vector<bool>& waypts_indi);
    void reset();
    bool en_vis_ = false;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    /* Func */
    void getReplanArea();
    void addAnchors();
    void getReplanConstraint();
    void checkStates();
    void viewpointOpt();
    void pathReordering();
    void segmentOpt();
    /* Tools */
    void fps(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int num_sample);
    bool vpOpt(int& idx, Eigen::VectorXd& vp, Eigen::Matrix4Xd& vis_st, vector<int>& vis_st_idx, bool start);
    Eigen::Vector3d findMedianPoint(const Eigen::Matrix4Xd& points);
    void localSamplesTemplate();
    void genSamplesGroup(PolarCoord& local_frame_sample);
    bool optPosAlongLine(const Eigen::VectorXd& input_vp, Eigen::VectorXd& vp, double& opt_dist, Eigen::Vector3d& line_dir);
    bool calVisibleP(PolarCoord& sample, Eigen::Matrix4Xd& vis_st, double& init_dist);
    void maximizeVis(PolarCoord& sample, Eigen::Matrix<double, 5, Eigen::Dynamic>& results, double& init_dist);
    void stepUpdateResults(Eigen::Matrix<double, 5, Eigen::Dynamic>& A, double l, Eigen::Matrix<double, 5, Eigen::Dynamic>& B);
    bool getBestSample(vector<PolarCoord>& opt_samples, int& idx, Eigen::VectorXd& best_vp);
    void updateCoverageStates(vector<PolarCoord>& opt_samples, Eigen::VectorXd& best_vp, vector<int>& vis_st_idx, bool start);
    void addNewVpsFromOpenPool();
    double calCoverGain(int& idx_in_open_pool, double& unit_gain);
    /* Param */
    double half_fov_top_angle_, half_fov_left_angle_;
    double vis_inf_, opt_inf_;
    int env_fps_size_;
    double pitch_upper_, pitch_lower_;
    double theta_step_, phi_step_;
    double grid_inf_, drone_radius_, safe_height_;
    double vmax_, wmax_;
    double path_interval_;
    int search_step_;
    int total_target_, uncovered_before_, uncovered_now_;
    double open_pool_range_;
    bool no_need_opt_;
    int k_samples_;
    double alpha_dist_;
    double ceil_;
    /* Utils */
    unique_ptr<RayCaster> raycaster_ = nullptr;
    shared_ptr<SDFMap> map_ = nullptr;
    unique_ptr<PerceptionUtils> percep_utils_ = nullptr;
    unique_ptr<PlanningVisualization> vis_utils_ = nullptr;
    unique_ptr<SOP> sop_= nullptr;
    unique_ptr<HDAstar> hd_astar_ = nullptr;
    /* Data */
    vector<Eigen::VectorXd> input_path_;
    vector<Eigen::VectorXd> output_path_;
    vector<bool> waypts_indi_;
    PlanDataWrapper plan_data_;
    bool first_temp_ = true;
    /* ROS Service */
    ros::Timer visTimer_;
    void visCallback(const ros::TimerEvent& e);

    // ! Debug
    vector<vector<double>> sample_regions_;
    vector<vector<Eigen::Vector3d>> world_samples_;
    Eigen::Matrix<double, 5, 4> debug_H_ = Eigen::Matrix<double, 5, 4>::Zero();
    void debugFunc();
};

} // namespace fc_vision

#endif
