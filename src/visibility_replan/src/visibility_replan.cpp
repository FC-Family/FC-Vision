/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Mar. 2025
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the main algorithm of visibility-aware replanning in FC-Vision.
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

#include "visibility_replan/visibility_replan.h"

namespace fc_vision
{

void VisibilityReplan::init(ros::NodeHandle& nh)
{
  // * Module Initialization
  this->raycaster_.reset(new RayCaster);
  this->percep_utils_.reset(new PerceptionUtils);
  this->vis_utils_.reset(new PlanningVisualization(nh));
  this->sop_.reset(new SOP);
  this->hd_astar_.reset(new HDAstar);
  this->plan_data_.reset();

  this->percep_utils_->init(nh);
  this->percep_utils_->preComputeLocalH();
  this->hd_astar_->init(nh);

  // * ROS Service
  visTimer_ = nh.createTimer(ros::Duration(0.5), &VisibilityReplan::visCallback, this);

  // * Param Initialization 
  nh.param("perception_utils/top_angle",  this->half_fov_top_angle_, 0.0);
  nh.param("perception_utils/left_angle", this->half_fov_left_angle_, 0.0);
  nh.param("sdf_map/obstacles_inflation", this->grid_inf_, 0.0);
  nh.param("perception_utils/vis_inf",    this->vis_inf_, 0.4);
  nh.param("replanning/opt_inflation",    this->opt_inf_, 5.0);
  nh.param("replanning/env_fps_size",     this->env_fps_size_, 1000);
  nh.param("replanning/pitch_upper",      this->pitch_upper_, 1e-5);
  nh.param("replanning/pitch_lower",      this->pitch_lower_, -1e-5);
  nh.param("replanning/theta_step",       this->theta_step_, 1.0);
  nh.param("replanning/phi_step",         this->phi_step_, 1.0);
  nh.param("replanning/drone_radius",     this->drone_radius_, 0.0);
  nh.param("replanning/safe_height",      this->safe_height_, 0.0);
  nh.param("replanning/max_v",            this->vmax_, 1.0);
  nh.param("replanning/max_w",            this->wmax_, 1.0);
  nh.param("replanning/path_interval",    this->path_interval_, 1.0);
  nh.param("replanning/vis_search_step",  this->search_step_, 1);
  nh.param("replanning/open_pool_range",  this->open_pool_range_, 1.0);
  nh.param("replanning/top_k",            this->k_samples_, 30);
  nh.param("replanning/alpha_dist",       this->alpha_dist_, 10.0);
  nh.param("replanning/ceil",             this->ceil_, 1000.0);
  this->no_need_opt_ = false;

  this->pitch_upper_ = this->pitch_upper_ * M_PI / 180.0;
  this->pitch_lower_ = this->pitch_lower_ * M_PI / 180.0;
  this->theta_step_ = this->theta_step_ * M_PI / 180.0;
  this->phi_step_ = this->phi_step_ * M_PI / 180.0;
  this->sop_->setParams(this->vmax_, this->wmax_);
  this->total_target_ = 1, this->uncovered_before_ = 0, this->uncovered_now_ = 0;

  return;
}

void VisibilityReplan::setMap(shared_ptr<SDFMap> map)
{
  this->map_ = map;
  this->raycaster_->setParams(this->map_->mp_->resolution_, this->map_->mp_->map_origin_);
  this->hd_astar_->setMap(this->map_);

  return;
}

void VisibilityReplan::setInputPath(vector<Eigen::VectorXd>& path)
{
  this->input_path_ = path;

  return;
}

void VisibilityReplan::replan()
{
  ROS_INFO("\033[34m[Replanning] Visibility-equivalent Replanning Starts! \033[34m");

  auto t1 = chrono::high_resolution_clock::now();

  // * 1. get replan area -> perception
  this->getReplanArea();

  // * 2. add anchors -> facilitate optimization
  this->addAnchors();

  // * 3. get replan constraint -> visibility area
  this->getReplanConstraint();

  // * 4. check states -> need optimization or not
  this->checkStates();

  // * 5. viewpoint visibility optimization
  this->viewpointOpt();

  // * 6. path reordering
  this->pathReordering();

  // * 7. occlusion-free segment searching
  this->segmentOpt();

  stringstream stream_crb, stream_crn;
  double covered_rate_before = (1.0 - (double)this->uncovered_before_ / (double)this->total_target_) * 100.0;
  double covered_rate_now = (1.0 - (double)this->uncovered_now_ / (double)this->total_target_) * 100.0;
  stream_crb << fixed << setprecision(2) << covered_rate_before;
  stream_crn << fixed << setprecision(2) << covered_rate_now;
  string crb_str = stream_crb.str();
  string crn_str = stream_crn.str();

  auto t2 = chrono::high_resolution_clock::now();
  chrono::duration<double, milli> algo_ms = t2 - t1;
  double algo_time = (double)algo_ms.count(); 
  ROS_INFO("\033[34m[Replanning] Visibility-equivalent Path Replanning Uses %lf ms, Coverage Rate from %s%% to %s%%.\033[34m", algo_time, crb_str.c_str(), crn_str.c_str());

  // ! debug
  // this->debugFunc();

  return;
}

void VisibilityReplan::getOutputPath(vector<Eigen::VectorXd>& path, vector<bool>& waypts_indi)
{
  path = this->output_path_;
  waypts_indi = this->waypts_indi_;

  return;
}

void VisibilityReplan::reset()
{
  this->input_path_.clear();
  this->output_path_.clear();
  this->waypts_indi_.clear();
  this->plan_data_.reset();
  this->total_target_ = 1;
  this->uncovered_before_ = 0;
  this->uncovered_now_ = 0;

  return;
}

// ! -------------------------- ROS Functions -------------------------- ! //
void VisibilityReplan::visCallback(const ros::TimerEvent& e)
{
  if (!this->en_vis_) return;

  if (!this->input_path_.empty())
  {
    // input path
    vector<vector<Eigen::Vector3d>> list_1, list_2;
    Eigen::Vector3d position;
    double pitch, yaw;
    for (int i=0; i<(int)this->input_path_.size(); ++i)
    {
      vector<Eigen::Vector3d> l1, l2;
      position(0) = this->input_path_[i](0);
      position(1) = this->input_path_[i](1);
      position(2) = this->input_path_[i](2);
      pitch = this->input_path_[i](3);
      yaw = this->input_path_[i](4);
      this->percep_utils_->setPose_PY(position, pitch, yaw);
      this->percep_utils_->getFOVShrink_PY(l1, l2);

      list_1.push_back(l1);
      list_2.push_back(l2);
    }

    this->vis_utils_->publishCurPath(this->input_path_, list_1, list_2);

    // input visible area
    // pcl::PointCloud<pcl::PointXYZ>::Ptr input_visible_st(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr input_visible_actual(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointXYZ pt;
    // for (int x = this->map_->hcmp_->box_min_(0) /* + 1 */; x < this->map_->hcmp_->box_max_(0); ++x)
    //   for (int y = this->map_->hcmp_->box_min_(1) /* + 1 */; y < this->map_->hcmp_->box_max_(1); ++y)
    //     for (int z = this->map_->hcmp_->box_min_(2) /* + 1 */; z < this->map_->hcmp_->box_max_(2); ++z)
    //     {
    //       Eigen::Vector3d pt_pos;
    //       this->map_->indexToPos_hc(Eigen::Vector3i(x, y, z), pt_pos);
    //       pt.x = pt_pos(0);
    //       pt.y = pt_pos(1);
    //       pt.z = pt_pos(2);
            
    //       if (this->map_->hcmd_->occupancy_buffer_hc_[this->map_->toAddress_hc(x, y, z)] == 1) 
    //       {
    //         bool visible_st = false, visible_actual = false;
    //         for (int i=0; i<(int)this->input_path_.size(); ++i)
    //         {
    //           Eigen::VectorXd vp = this->input_path_[i];
    //           Eigen::Vector3d vp_pos = vp.head(3);
    //           double vp_pitch = vp(3), vp_yaw = vp(4);
    //           this->percep_utils_->setPose_PY(vp_pos, vp_pitch, vp_yaw);

    //           {
    //             if (visible_st && visible_actual) continue;

    //             visible_st = true;
    //             visible_actual = true;

    //             Eigen::Vector3d ray_dir = (pt_pos - vp_pos).normalized();
    //             double dist = (pt_pos - vp_pos).norm();
    //             Eigen::Vector3d pt_inf = vp_pos + (dist-this->vis_inf_) * ray_dir;

    //             Eigen::Vector3i idx;
    //             this->raycaster_->input(vp_pos, pt_inf);
    //             while (this->raycaster_->nextId(idx)) 
    //             {
    //               if (this->map_->getEnv(idx)) 
    //               {
    //                 visible_actual = false;
    //                 break;
    //               }
    //             }
    //           }
    //         }

    //         if (visible_st) input_visible_st->points.push_back(pt);
    //         if (visible_actual) input_visible_actual->points.push_back(pt);
    //       }
    //     }

    // this->vis_utils_->publishLocalVST(input_visible_st, input_visible_actual);
  }

  if (!this->output_path_.empty())
  {
    // input path
    vector<vector<Eigen::Vector3d>> list_1, list_2;
    Eigen::Vector3d position;
    double pitch, yaw;
    for (int i=0; i<(int)this->output_path_.size(); ++i)
    {
      vector<Eigen::Vector3d> l1, l2;
      position(0) = this->output_path_[i](0);
      position(1) = this->output_path_[i](1);
      position(2) = this->output_path_[i](2);
      pitch = this->output_path_[i](3);
      yaw = this->output_path_[i](4);
      this->percep_utils_->setPose_PY(position, pitch, yaw);
      this->percep_utils_->getFOV_PY(l1, l2);

      list_1.push_back(l1);
      list_2.push_back(l2);
    }

    this->vis_utils_->publishUpdatedCurPath(this->output_path_, list_1, list_2);
  }

  this->vis_utils_->publishLocalRegion(this->plan_data_.target_cloud, this->plan_data_.env_cloud, this->plan_data_.min_bound, this->plan_data_.max_bound);

  if (!this->sample_regions_.empty())
  {
    this->vis_utils_->publishSampleRegions(this->sample_regions_);
  }

  if (!this->world_samples_.empty() && !this->plan_data_.local_samples_template.empty())
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr world_samples(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_samples(new pcl::PointCloud<pcl::PointXYZ>);

    for (auto x : this->world_samples_.back())
    {
      pcl::PointXYZ pt;
      pt.x = x(0);
      pt.y = x(1);
      pt.z = x(2);
      world_samples->points.push_back(pt);
    }

    for (auto x : this->plan_data_.local_samples_template)
    {
      pcl::PointXYZ pt;
      pt.x = x.vec_xyz(0);
      pt.y = x.vec_xyz(1);
      pt.z = x.vec_xyz(2);
      local_samples->points.push_back(pt);
    }
    this->vis_utils_->publishSamples(world_samples, local_samples);
  }

  if (!this->debug_H_.isZero())
  {
    this->vis_utils_->publishFOVHRep(this->debug_H_);
  }

  return;
}

} // namespace fc_vision
