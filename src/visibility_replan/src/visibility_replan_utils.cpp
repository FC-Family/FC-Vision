/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Mar. 2025
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the utils functions of visibility-aware replanning in FC-Vision.
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

void VisibilityReplan::getReplanArea()
{
  auto t1 = chrono::high_resolution_clock::now();
  
  // * 1. get replan range
  for (int i=0; i<(int)this->input_path_.size(); ++i)
  {
    this->plan_data_.min_bound(0) = min(this->plan_data_.min_bound(0), this->input_path_[i](0));
    this->plan_data_.min_bound(1) = min(this->plan_data_.min_bound(1), this->input_path_[i](1));
    this->plan_data_.min_bound(2) = min(this->plan_data_.min_bound(2), this->input_path_[i](2));
    this->plan_data_.max_bound(0) = max(this->plan_data_.max_bound(0), this->input_path_[i](0));
    this->plan_data_.max_bound(1) = max(this->plan_data_.max_bound(1), this->input_path_[i](1));
    this->plan_data_.max_bound(2) = max(this->plan_data_.max_bound(2), this->input_path_[i](2));
  }

  this->plan_data_.min_bound -= Eigen::Vector3d(this->opt_inf_, this->opt_inf_, this->opt_inf_);
  this->plan_data_.max_bound += Eigen::Vector3d(this->opt_inf_, this->opt_inf_, this->opt_inf_);

  this->plan_data_.min_bound(0) = max(this->plan_data_.min_bound(0), this->map_->mp_->map_min_boundary_(0));
  this->plan_data_.min_bound(1) = max(this->plan_data_.min_bound(1), this->map_->mp_->map_min_boundary_(1));
  this->plan_data_.min_bound(2) = max(this->plan_data_.min_bound(2), this->map_->mp_->map_min_boundary_(2));
  this->plan_data_.max_bound(0) = min(this->plan_data_.max_bound(0), this->map_->mp_->map_max_boundary_(0));
  this->plan_data_.max_bound(1) = min(this->plan_data_.max_bound(1), this->map_->mp_->map_max_boundary_(1));
  this->plan_data_.max_bound(2) = min(this->plan_data_.max_bound(2), this->map_->mp_->map_max_boundary_(2));

  // * 2. get replan local map
  this->map_->getLocalHCMap(this->plan_data_.target_cloud, this->plan_data_.min_bound, this->plan_data_.max_bound);
  // this->map_->getLocalEnv(this->plan_data_.env_cloud, this->plan_data_.min_bound, this->plan_data_.max_bound);

  // * get local environment with cube corridoration
  double max_interval = -1.0;
  for (int i=0; i<(int)this->input_path_.size()-1; ++i)
  {
    max_interval = max(max_interval, (this->input_path_[i+1].head(3) - this->input_path_[i].head(3)).norm());
  }
  max_interval = 0.5 * max_interval;

  for (int i=0; i<(int)this->input_path_.size(); ++i)
  {
    Eigen::Vector3d pos = this->input_path_[i].head(3);
    double pitch = this->input_path_[i](3);
    double yaw = this->input_path_[i](4);
    this->percep_utils_->setPose_PY(pos, pitch, yaw);
    Eigen::Vector3d min_vp_b, max_vp_b;
    this->percep_utils_->getFOVAABB(min_vp_b, max_vp_b, max_interval);
    pcl::PointCloud<pcl::PointXYZ>::Ptr vp_fov_aabb(new pcl::PointCloud<pcl::PointXYZ>);
    this->map_->getLocalEnv(vp_fov_aabb, min_vp_b, max_vp_b);
    *(this->plan_data_.env_cloud) += *vp_fov_aabb;
  }

  // ? a. farthest point sampling in CPU
  // if ((int)this->plan_data_.target_cloud->points.size() > this->env_fps_size_) this->fps(this->plan_data_.target_cloud, this->env_fps_size_);
  // if ((int)this->plan_data_.env_cloud->points.size() > this->env_fps_size_) this->fps(this->plan_data_.env_cloud, this->env_fps_size_);
  
  // ? b. random sampling
  if ((int)this->plan_data_.target_cloud->points.size() > this->env_fps_size_)
  {
    pcl::RandomSample<pcl::PointXYZ> random_sampler_tar;
    random_sampler_tar.setSample(this->env_fps_size_);
    random_sampler_tar.setInputCloud(this->plan_data_.target_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    random_sampler_tar.filter(*target_filtered);
    this->plan_data_.target_cloud = target_filtered;
  }

  if ((int)this->plan_data_.env_cloud->points.size() > this->env_fps_size_)
  {
    pcl::RandomSample<pcl::PointXYZ> random_sampler_env;
    random_sampler_env.setSample(this->env_fps_size_);
    random_sampler_env.setInputCloud(this->plan_data_.env_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr env_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    random_sampler_env.filter(*env_filtered);
    this->plan_data_.env_cloud = env_filtered;
  }

  visibility_utils::pointCloudToEigen(this->plan_data_.target_cloud, this->plan_data_.target_mat);
  visibility_utils::pointCloudToEigen(this->plan_data_.env_cloud, this->plan_data_.env_mat);

  auto t2 = chrono::high_resolution_clock::now();
  chrono::duration<double, milli> algo_ms = t2 - t1;
  double algo_time = (double)algo_ms.count(); 
  ROS_INFO("\033[32m[Replanning][Preparation] get replan area time -> %lf ms.\033[32m", algo_time);

  return;
}

void VisibilityReplan::addAnchors()
{
  auto t1 = chrono::high_resolution_clock::now();

  // * 1. pre check each segment
  this->hd_astar_->setOcclusion(this->plan_data_.env_mat);
  this->hd_astar_->setSearchStep(this->search_step_);
  this->hd_astar_->setPathInterval(this->path_interval_);

  double max_dist = 2 * this->path_interval_;
  this->plan_data_.ext_path.push_back(this->input_path_.front());
  this->plan_data_.ext_indi.push_back(false);
  this->plan_data_.ext_table[this->input_path_.front()] = false;
  for (int i=1; i<(int)this->input_path_.size(); ++i)
  {
    Eigen::VectorXd prev = this->input_path_[i-1];
    Eigen::VectorXd curr = this->input_path_[i];

    this->hd_astar_->reset();
    if (!this->hd_astar_->preCheck(prev, curr))
    {
      double dist = (curr.head(3) - prev.head(3)).norm();
      if (dist > max_dist)
      {
        double seg_dist = 0.8*dist;
        vector<Eigen::VectorXd> seg;
        path_tools::pieceInterpolate(prev, curr, seg_dist, seg);
        Eigen::VectorXd anchor = seg.front();
        this->plan_data_.ext_path.push_back(anchor);
        this->plan_data_.ext_indi.push_back(true);
        this->plan_data_.ext_table[anchor] = true;
      }
    }

    this->plan_data_.ext_path.push_back(curr);
    this->plan_data_.ext_indi.push_back(false);
    this->plan_data_.ext_table[curr] = false;
  }

  // * 2. update the input path
  this->input_path_.clear();
  this->input_path_ = this->plan_data_.ext_path;

  auto t2 = chrono::high_resolution_clock::now();
  chrono::duration<double, milli> algo_ms = t2 - t1;
  double algo_time = (double)algo_ms.count(); 
  ROS_INFO("\033[32m[Replanning][Preparation] add anchors in original path time -> %lf ms.\033[32m", algo_time);
}

void VisibilityReplan::getReplanConstraint()
{
  auto t1 = chrono::high_resolution_clock::now();

  // * 1. initialize containers
  this->plan_data_.qualified_states.resize(this->input_path_.size(), false);
  this->plan_data_.vis_rates.resize(this->input_path_.size(), 0.0);
  this->plan_data_.vis_st_set.resize(this->input_path_.size());
  this->plan_data_.vis_st_set_idx.resize(this->input_path_.size());

  // * 2. get expected visible area as visibility constraint
  vector<bool> vis_st(this->plan_data_.target_mat.cols(), false);
  for (int i=0; i<(int)this->input_path_.size(); ++i)
  {
    Eigen::VectorXd vp = this->input_path_[i];
    Eigen::Vector3d vp_pos = vp.head(3);
    double vp_pitch = vp(3), vp_yaw = vp(4);
    this->percep_utils_->setPose_PY(vp_pos, vp_pitch, vp_yaw);

    vector<bool> independent_vis_st(vis_st.size(), false);
    for (int j=0; j<(int)this->plan_data_.target_mat.cols(); ++j)
    {
      Eigen::Vector3d pt = this->plan_data_.target_mat.col(j).head(3);
      if (this->percep_utils_->insideFOV(pt)) 
      {
        bool visible = true;
        vector<Eigen::Vector3d> line_samples = visibility_utils::sampleLine(vp_pos, pt, this->vis_inf_, this->map_->getResolution());
        for (Eigen::Vector3d& sample_pt : line_samples)
        {
          double hc_esdf = this->map_->getDistance_hc(sample_pt);
          if (abs(hc_esdf) < 0.2)
          {
            visible = false;
            break;
          }
        }

        if (visible)
        {
          vis_st[j] = true;
          independent_vis_st[j] = true; 
        }
      }
    }
    size_t independent_vis_num = count(independent_vis_st.begin(), independent_vis_st.end(), true);
    Eigen::Matrix4Xd vis_st_mat(4, independent_vis_num);
    vector<int> vis_st_idx;
    int count = 0;
    for (int k=0; k<(int)independent_vis_st.size(); ++k)
    {
      if (!independent_vis_st[k]) continue;
      vis_st_mat.col(count) = this->plan_data_.target_mat.col(k);
      vis_st_idx.push_back(k);
      count++;
    }
    this->plan_data_.vis_st_set[i] = vis_st_mat;
    this->plan_data_.vis_st_set_idx[i] = vis_st_idx;
  }

  this->plan_data_.target_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ pt;
  map<int, int> target_idx_map;
  int count = 0;
  for (int i=0; i<(int)this->plan_data_.target_mat.cols(); ++i)
  {
    pt.x = this->plan_data_.target_mat(0, i);
    pt.y = this->plan_data_.target_mat(1, i);
    pt.z = this->plan_data_.target_mat(2, i);
    if (vis_st[i]) 
    {
      this->plan_data_.target_cloud->points.push_back(pt);
      target_idx_map[i] = count;
      count++;
    }
  }
  visibility_utils::pointCloudToEigen(this->plan_data_.target_cloud, this->plan_data_.target_mat);
  this->total_target_ = this->plan_data_.target_mat.cols();
  this->plan_data_.target_cover_states.resize(this->plan_data_.target_mat.cols(), false);
  for (int i=0; i<(int)this->plan_data_.vis_st_set_idx.size(); ++i)
  {
    for (int j=0; j<(int)this->plan_data_.vis_st_set_idx[i].size(); ++j)
    {
      int idx = this->plan_data_.vis_st_set_idx[i][j];
      if (target_idx_map.find(idx) != target_idx_map.end())
      {
        this->plan_data_.vis_st_set_idx[i][j] = target_idx_map[idx];
      }
    }
  }

  auto t2 = chrono::high_resolution_clock::now();
  chrono::duration<double, milli> algo_ms = t2 - t1;
  double algo_time = (double)algo_ms.count(); 
  ROS_INFO("\033[32m[Replanning][Preparation] get replan vis constraint time -> %lf ms.\033[32m", algo_time);

  return;
}

void VisibilityReplan::checkStates()
{
  auto t1 = chrono::high_resolution_clock::now();

  // * check each viewpoint -> collision & occlusion
  for (int i=0; i<(int)this->input_path_.size(); ++i)
  {
    Eigen::Vector3d vp_pos = this->input_path_[i].head(3);

    // collision checking -> NOT in inflation area
    bool collision = false;
    if (this->map_->getInflateOccupancy(vp_pos) == 1) collision = true;

    // raycasting-based occlusion checking
    Eigen::Matrix4Xd vis_st = this->plan_data_.vis_st_set[i];
    int occlusion_num = 0;
    for (int j=0; j<vis_st.cols(); ++j)
    {
      Eigen::Vector3d pt_pos = vis_st.col(j).head(3);
      Eigen::Vector3d ray_dir = (pt_pos - vp_pos).normalized();
      double dist = (pt_pos - vp_pos).norm();
      Eigen::Vector3d pt_inf = vp_pos + (dist-this->vis_inf_) * ray_dir;

      Eigen::Vector3i idx;
      this->raycaster_->input(vp_pos, pt_inf);
      while (this->raycaster_->nextId(idx)) 
      {
        if (this->map_->getEnv(idx)) 
        {
          occlusion_num++;
          break;
        }
      }
    }

    if (collision || occlusion_num > 0 || (int)vis_st.cols() == 0)
    {
      // cout << "vp " << i << " occlusion rate -> " << (double)occlusion_num / (double)vis_st.cols() << ", collision -> " << collision << endl;
      
      this->plan_data_.qualified_states[i] = false;
      this->plan_data_.vis_rates[i] = 1.0 - (double)occlusion_num / (double)vis_st.cols();
    }
    else
    {
      this->plan_data_.qualified_states[i] = true;
      this->plan_data_.vis_rates[i] = 1.0;
      for (auto idx : this->plan_data_.vis_st_set_idx[i])
      {
        this->plan_data_.target_cover_states[idx] = true;
      }
    }
  }
  this->plan_data_.qualified_states[0] = true; // * ensure the start state is always qualified

  auto t2 = chrono::high_resolution_clock::now();
  chrono::duration<double, milli> algo_ms = t2 - t1;
  double algo_time = (double)algo_ms.count(); 
  ROS_INFO("\033[32m[Replanning][Preparation] check occlusion state time -> %lf ms.\033[32m", algo_time);

  return;
}

void VisibilityReplan::viewpointOpt()
{
  auto t1 = chrono::high_resolution_clock::now();

  // * 1. independent optimization from initial viewpoint
  this->localSamplesTemplate();
  this->plan_data_.dl_vector.resize(5);
  this->plan_data_.dl_vector << sin(this->half_fov_top_angle_), sin(this->half_fov_top_angle_), sin(this->half_fov_left_angle_), sin(this->half_fov_left_angle_), -1.0;

  this->first_temp_ = true;

  bool opt_suc = false, is_start = false;
  for (int i=0; i<(int)this->plan_data_.qualified_states.size(); ++i)
  {
    Eigen::VectorXd cur_vp = this->input_path_[i];
    if (!this->plan_data_.qualified_states[i])
    {
      is_start = i == 0 ? true : false; 
 
      if (this->plan_data_.vis_st_set[i].cols() == 0)
      {
        opt_suc = false;
        ROS_WARN("\033[35m[Replanning][Warning] viewpoint %d has NO visible target points, skip optimization.\033[35m", i);
      }
      else
      {
        opt_suc = this->vpOpt(i, cur_vp, this->plan_data_.vis_st_set[i], this->plan_data_.vis_st_set_idx[i], is_start);
      }
      if (is_start) 
      {
        this->plan_data_.qual_vps.push_back(cur_vp);
        this->plan_data_.all_vps.push_back(cur_vp);
      }
      else
      {
        if (opt_suc) 
        {
          this->plan_data_.opted_vps.push_back(cur_vp);
          this->plan_data_.all_vps.push_back(cur_vp);
        }
      }
    }
    else
    {
      this->plan_data_.qual_vps.push_back(cur_vp);
      this->plan_data_.all_vps.push_back(cur_vp);
    }
  }

  if (this->plan_data_.opted_vps.empty()) this->no_need_opt_ = true;
  else this->no_need_opt_ = false;

  // * 2. find other viewpoints from open_pool for complete coverage
  if (!this->no_need_opt_)
  {
    this->addNewVpsFromOpenPool();
    // if (!this->plan_data_.S_set.empty())
    // {
    //   this->plan_data_.all_vps.insert(this->plan_data_.all_vps.end(), this->plan_data_.S_set.begin(), this->plan_data_.S_set.end());
    //   this->plan_data_.opted_vps.insert(this->plan_data_.opted_vps.end(), this->plan_data_.S_set.begin(), this->plan_data_.S_set.end());
    // }
  }

  auto t2 = chrono::high_resolution_clock::now();
  chrono::duration<double, milli> algo_ms = t2 - t1;
  double algo_time = (double)algo_ms.count();
  ROS_INFO("\033[33m[Replanning][Process] visibility-equivalent viewpoint optimization time -> %lf ms.\033[33m", algo_time);

  return;
}

void VisibilityReplan::pathReordering()
{
  auto t1 = chrono::high_resolution_clock::now();

  if (!this->no_need_opt_)
  {
    // * 1. construct cost matrix
    vector<Eigen::VectorXd> temp_vps;
    temp_vps.insert(temp_vps.end(), this->plan_data_.qual_vps.begin(), this->plan_data_.qual_vps.end());
    temp_vps.insert(temp_vps.end(), this->plan_data_.opted_vps.begin(), this->plan_data_.opted_vps.end());
    Eigen::MatrixXi cost_mat = this->sop_->constructCostMat(this->plan_data_.qual_vps, this->plan_data_.opted_vps);

    // vector<Eigen::VectorXd> fix = {this->plan_data_.qual_vps.front()};
    // vector<Eigen::VectorXd> rem;
    // rem.insert(rem.end(), this->plan_data_.qual_vps.begin()+1, this->plan_data_.qual_vps.end());
    // rem.insert(rem.end(), this->plan_data_.opted_vps.begin(), this->plan_data_.opted_vps.end());
    // Eigen::MatrixXi cost_mat = this->sop_->constructCostMat(fix, rem);

    // * 2. solve Seq Ordering Problem (SOP)
    vector<int> solution;
    this->sop_->sopSolve(cost_mat, solution);

    // * 3. read solution
    for (auto idx : solution)
    {
      this->output_path_.push_back(temp_vps[idx]);
    }
  }
  else
  {
    this->output_path_ = this->plan_data_.qual_vps;
  }

  auto t2 = chrono::high_resolution_clock::now();
  chrono::duration<double, milli> algo_ms = t2 - t1;
  double algo_time = (double)algo_ms.count();
  ROS_INFO("\033[33m[Replanning][Process] path reordering time -> %lf ms.\033[33m", algo_time);

  return;
}

void VisibilityReplan::segmentOpt()
{
  auto t1 = chrono::high_resolution_clock::now();

  // * 1. search occlusion-free segments for each piece
  vector<Eigen::VectorXd> updated_output_path, segment_path;
  vector<bool> indicators;
  Eigen::VectorXd s_pt, e_pt;
  for (int i=0; i<(int)this->output_path_.size()-1; ++i)
  {
    updated_output_path.push_back(this->output_path_[i]);
    indicators.push_back(false);
    s_pt = this->output_path_[i];
    e_pt = this->output_path_[i+1];

    if ((s_pt.head(3)-e_pt.head(3)).norm() < this->path_interval_) continue;

    if (this->hd_astar_->casVisEqSearch(s_pt, e_pt))
    {
      segment_path = this->hd_astar_->getPath();
      if ((int)segment_path.size() > 2)
      {
        updated_output_path.insert(updated_output_path.end(), segment_path.begin()+1, segment_path.end()-1);
        indicators.insert(indicators.end(), segment_path.size()-2, true);
      }
    }
    else
    {
      ROS_INFO("\033[35m[Replanning][Debug] high-dim astar search FAILED between two viewpoints. Try safe search or piecewise interpolation. \033[35m");
      this->hd_astar_->reset();
      if (this->hd_astar_->casSafeSearch(s_pt, e_pt))
      {
        segment_path = this->hd_astar_->getPath();
        if ((int)segment_path.size() > 2)
        {
          updated_output_path.insert(updated_output_path.end(), segment_path.begin()+1, segment_path.end()-1);
          indicators.insert(indicators.end(), segment_path.size()-2, true);
        }
      }
      else
      {
        segment_path.clear();
        path_tools::pieceInterpolate(s_pt, e_pt, this->path_interval_, segment_path);
        if (!segment_path.empty())
        {
          updated_output_path.insert(updated_output_path.end(), segment_path.begin(), segment_path.end());
          indicators.insert(indicators.end(), segment_path.size(), true);
        }
      }
    }
  }
  updated_output_path.push_back(this->output_path_.back());
  indicators.push_back(false);

  for (int i=0; i<(int)updated_output_path.size(); ++i)
  {
    Eigen::VectorXd cur_vp = updated_output_path[i];
    if (this->plan_data_.ext_table.find(cur_vp) != this->plan_data_.ext_table.end())
    {
      indicators[i] = this->plan_data_.ext_table[cur_vp];
    }
  }

  this->output_path_.clear();
  this->output_path_ = updated_output_path;
  this->waypts_indi_.clear();
  this->waypts_indi_ = indicators;

  auto t2 = chrono::high_resolution_clock::now();
  chrono::duration<double, milli> algo_ms = t2 - t1;
  double algo_time = (double)algo_ms.count();
  ROS_INFO("\033[33m[Replanning][Process] visibility-equivalent segment searching time -> %lf ms.\033[33m", algo_time);

  // * 2. statistics
  int covered_before = 0;
  Eigen::Vector3d t_pt;
  Eigen::VectorXd q_wp(5);
  for (int i=0; i<(int)this->plan_data_.target_mat.cols(); ++i)
  {
    t_pt = this->plan_data_.target_mat.col(i).head(3);
    for (int j=0; j<(int)this->plan_data_.qual_vps.size(); ++j)
    {
      q_wp = this->plan_data_.qual_vps[j];
      this->percep_utils_->setPose_PY(q_wp.head(3), q_wp(3), q_wp(4));
      if (this->percep_utils_->insideFOV(t_pt))
      {
        bool visible = true;
        vector<Eigen::Vector3d> line_samples = visibility_utils::sampleLine(q_wp.head(3), t_pt, this->vis_inf_, this->map_->getResolution());
        for (Eigen::Vector3d& sample_pt : line_samples)
        {
          double hc_esdf = this->map_->getDistance_hc(sample_pt);
          if (abs(hc_esdf) < 0.2)
          {
            visible = false;
            break;
          }
        }

        if (visible)
        {
          covered_before++;
          break;
        }
      }
    }
  }
  this->uncovered_before_ = this->total_target_ - covered_before;

  int u_id;
  Eigen::Vector3d u_pt;
  Eigen::VectorXd o_wp(5);
  for (int i=0; i<(int)this->plan_data_.uncovered_idx.size(); ++i)
  {
    u_id = this->plan_data_.uncovered_idx[i];
    if (this->plan_data_.target_cover_states[u_id]) continue;
    u_pt = this->plan_data_.target_mat.col(u_id).head(3);

    for (int j=0; j<(int)this->output_path_.size(); ++j)
    {
      o_wp = this->output_path_[j];
      this->percep_utils_->setPose_PY(o_wp.head(3), o_wp(3), o_wp(4));
      if (this->percep_utils_->insideFOV(u_pt))
      {
        bool visible = true;
        vector<Eigen::Vector3d> line_samples = visibility_utils::sampleLine(o_wp.head(3), u_pt, this->vis_inf_, this->map_->getResolution());
        for (Eigen::Vector3d& sample_pt : line_samples)
        {
          double hc_esdf = this->map_->getDistance_hc(sample_pt);
          if (abs(hc_esdf) < 0.2)
          {
            visible = false;
            break;
          }
        }

        if (visible)
        {
          this->plan_data_.target_cover_states[u_id] = true;
          break;
        }
      }
    }
  }
  this->uncovered_now_ = count(this->plan_data_.target_cover_states.begin(), this->plan_data_.target_cover_states.end(), false);

  // auto t3 = chrono::high_resolution_clock::now();

  // Eigen::VectorXd test_vp = this->output_path_.front();
  // Eigen::Vector3d test_vp_pos = test_vp.head(3);
  // double test_vp_pitch = test_vp(3), test_vp_yaw = test_vp(4);
  // this->percep_utils_->setPose_PY(test_vp_pos, test_vp_pitch, test_vp_yaw);
  // Eigen::Vector3d min_vp_b, max_vp_b;
  // this->percep_utils_->getFOVShrinkAABB_PY(min_vp_b, max_vp_b);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr vp_fov_aabb(new pcl::PointCloud<pcl::PointXYZ>);
  // this->map_->getLocalHCMap(vp_fov_aabb, min_vp_b, max_vp_b);

  // auto t4 = chrono::high_resolution_clock::now();
  // chrono::duration<double, milli> algo_ms2 = t4 - t3;
  // double algo_time2 = (double)algo_ms2.count();
  // ROS_INFO("\033[33m[Replanning][Process] FOV-AABB local map time -> %lf ms.\033[33m", algo_time2);
  // cout << "vp_fov_aabb size: " << vp_fov_aabb->points.size() << endl;

  return;
}

} // namespace fc_vision
