/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2025
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the main algorithm of safe & visible path searching
 *                   using high-dim A* algorithm.
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

#include "path_searching/hd_astar.h"

namespace fc_vision {

HDAstar::HDAstar() {
}

HDAstar::~HDAstar() {
  for (int i = 0; i < allocate_num_; i++)
  {
    if (path_node_pool_[i] != nullptr) 
    {
      delete path_node_pool_[i];
      path_node_pool_[i] = nullptr;
    }
  }
}

void HDAstar::init(ros::NodeHandle& nh)
{
  nh.param("hdastar/resolution",           this->resolution_, -1.0);
  nh.param("hdastar/lambda_heu",           this->lambda_heu_, -1.0);
  nh.param("hdastar/allocate_num",         this->allocate_num_, -1);
  nh.param("hdastar/max_vis_search_time",  this->max_vis_search_time_, -1.0);
  nh.param("hdastar/max_safe_search_time", this->max_safe_search_time_, -1.0);
  nh.param("hdastar/max_hc_search_time",   this->max_hc_search_time_, -1.0);
  nh.param("hdastar/safe_height",          this->safe_height_, -1.0);
  this->tie_breaker_ = 1.0 + 1.0 / 1000;
  this->inv_resolution_ = 1.0 / this->resolution_;

  this->path_node_pool_.clear();
  this->path_node_pool_.resize(this->allocate_num_);
  for (int i = 0; i < this->allocate_num_; i++) this->path_node_pool_[i] = new HDNode;
  this->use_node_num_ = 0;
  this->iter_num_ = 0;
  this->early_terminate_cost_ = 0.0;

  this->percep_utils_.reset(new PerceptionUtils);
  this->percep_utils_->init(nh);
  this->percep_utils_->preComputeLocalH();

  return;
}

void HDAstar::setMap(shared_ptr<SDFMap> map)
{
  this->map_ = map;
  this->origin_ = this->map_->mp_->map_origin_;
  this->raycaster_.reset(new RayCaster);
  this->raycaster_->setParams(this->resolution_, this->origin_);

  return;
}

void HDAstar::setOcclusion(Eigen::Matrix4Xd& occlusion)
{
  this->occlusion_mat_.resize(4, occlusion.cols());
  this->occlusion_mat_ = occlusion;
  this->occlusion_results_.resize(5, occlusion.cols());

  return;
}

void HDAstar::reset()
{
  this->open_set_map_.clear();
  this->close_set_map_.clear();
  this->vis_cache_.clear();
  this->path_nodes_.clear();

  priority_queue<HDNodePtr, vector<HDNodePtr>, HDNodeComparator> empty_queue;
  this->open_set_.swap(empty_queue);
  for (int i = 0; i < this->use_node_num_; i++) this->path_node_pool_[i]->parent = NULL;
  this->use_node_num_ = 0;
  this->iter_num_ = 0;

  return;
}

void HDAstar::setSearchStep(const int step)
{
  this->search_step_ = step;
  this->buildNeighborOffsets(step);
  
  return;
}

void HDAstar::setPathInterval(const double interval)
{
  this->path_interval_ = interval;

  return;
}

bool HDAstar::preCheck(const Eigen::VectorXd& start_pt, const Eigen::VectorXd& end_pt)
{
  this->start_pose_ = start_pt;
  this->end_pose_ = end_pt;
  this->getAttitudeGap();
  bool line_check = this->linePreCheck(true, false);

  return line_check;
}

bool HDAstar::casVisEqSearch(const Eigen::VectorXd& start_pt, const Eigen::VectorXd& end_pt)
{
  int original_step = this->search_step_;
  vector<int> cas_res = {this->search_step_, 2*this->search_step_, 3*this->search_step_};

  for (auto step : cas_res)
  {
    this->search_step_ = step;
    this->reset();
    bool suc = this->visEqSearch(start_pt, end_pt) == SUCCEED;
    if (suc)
    {
      ROS_INFO("\033[34mHD-A* [Cascaded Visibility-equivalent Search] -> Succeed with step size : %d. \033[34m", step);
      this->search_step_ = original_step;
      return true;
    }
  }

  this->search_step_ = original_step;
  return false;
}

bool HDAstar::casSafeSearch(const Eigen::VectorXd& start_pt, const Eigen::VectorXd& end_pt)
{
  int original_step = 1;
  vector<int> cas_res = {1, 2, 3};

  for (auto step : cas_res)
  {
    this->search_step_ = step;
    this->reset();
    bool suc = this->safeSearch(start_pt, end_pt) == SUCCEED;
    if (suc)
    {
      ROS_INFO("\033[34mHD-A* [Cascaded Safe Search] -> Succeed with step size : %d. \033[34m", step);
      this->search_step_ = original_step;
      return true;
    }
  }

  this->search_step_ = original_step;
  return false;
}

int HDAstar::visEqSearch(const Eigen::VectorXd& start_pt, const Eigen::VectorXd& end_pt)
{
  this->start_pose_ = start_pt;
  this->end_pose_ = end_pt;
  this->getAttitudeGap();

  bool line_check = this->linePreCheck(true, false);
  if (line_check) return SUCCEED;

  HDNodePtr cur_node = this->path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->position = start_pt.head(3);
  cur_node->pose = start_pt;
  this->posToIndex(cur_node->position, cur_node->index);
  cur_node->g_score = 0.0;
  cur_node->f_score = this->lambda_heu_ * this->getDiagHeu(cur_node->position, end_pt.head(3));

  Eigen::Vector3d end_pos = end_pt.head(3);
  Eigen::Vector3i end_index;
  this->posToIndex(end_pos, end_index);

  this->open_set_.push(cur_node);
  this->open_set_map_.insert(make_pair(cur_node->index, cur_node));
  this->use_node_num_ += 1;

  const auto t1 = ros::Time::now();
  if (this->vis_neighbor_offsets_.empty() || this->vis_neighbor_step_cache_ != this->search_step_)
    this->buildNeighborOffsets(this->search_step_);
  /* ---------- search loop ---------- */
  while (!this->open_set_.empty())
  {
    cur_node = this->open_set_.top();
    bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 && abs(cur_node->index(1) - end_index(1)) <= 1 && abs(cur_node->index(2) - end_index(2)) <= 1;
    if (reach_end)
    {
      this->backTrack(cur_node, end_pt);
      return SUCCEED;
    }

    // early termination if time up
    if ((ros::Time::now() - t1).toSec() > this->max_vis_search_time_)
    {
      this->early_terminate_cost_ = cur_node->g_score + this->getDiagHeu(cur_node->position, end_pt.head(3));
      ROS_WARN("HD-A* [Visibility-equivalent Search] @ %lf -> Early termination", this->search_step_ * this->resolution_);
      return FAIL;
    }

    this->open_set_.pop();
    this->open_set_map_.erase(cur_node->index);
    this->close_set_map_.insert(make_pair(cur_node->index, 1));
    this->iter_num_ += 1;

    Eigen::Vector3d cur_pos = cur_node->position;
    Eigen::Vector3d nbr_pos;
    Eigen::Vector3i nbr_idx;
    Eigen::VectorXd nbr_pose(5);
    Eigen::Vector2d py;
    const Eigen::Vector3i cur_idx = cur_node->index;
    const double safe_height = this->safe_height_;

    for (const auto& offset : this->vis_neighbor_offsets_)
    {
      nbr_idx = cur_idx + offset.idx_offset;
      nbr_pos = cur_pos + offset.pos_offset;

      // quick close/occupancy checks
      if (this->close_set_map_.find(nbr_idx) != close_set_map_.end()) continue;
      if (this->map_->getInflateOccupancy(nbr_idx) != 0) continue;
      if (nbr_pos(2) < safe_height) continue;

      // visibility
      bool occlusion = this->interpolatePitchYaw(nbr_pos, true, py);
      if (occlusion) continue;
      nbr_pose << nbr_pos, py(0), py(1);

      HDNodePtr neighbor;
      double tmp_g_score = offset.step_len + cur_node->g_score;
      auto node_iter = this->open_set_map_.find(nbr_idx);
      if (node_iter == this->open_set_map_.end()) 
      {
        neighbor = this->path_node_pool_[use_node_num_];
        use_node_num_ += 1;
        if (use_node_num_ == allocate_num_) 
        {
          ROS_ERROR("run out of node pool.");
          return FAIL;
        }
        neighbor->index = nbr_idx;
        neighbor->position = nbr_pos;
        neighbor->pose = nbr_pose;
      } 
      else if (tmp_g_score < node_iter->second->g_score)
       {
        neighbor = node_iter->second;
      } 
      else continue;

      neighbor->parent = cur_node;
      neighbor->g_score = tmp_g_score;
      neighbor->f_score = tmp_g_score + this->lambda_heu_ * this->getDiagHeu(nbr_pos, end_pos);
      this->open_set_.push(neighbor);
      this->open_set_map_[nbr_idx] = neighbor;
    }
  }

  return FAIL;
}

int HDAstar::safeSearch(const Eigen::VectorXd& start_pt, const Eigen::VectorXd& end_pt)
{
  this->start_pose_ = start_pt;
  this->end_pose_ = end_pt;
  this->getAttitudeGap();

  bool line_check = this->linePreCheck(false, false);
  if (line_check) return SUCCEED;

  HDNodePtr cur_node = this->path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->position = start_pt.head(3);
  cur_node->pose = start_pt;
  this->posToIndex(cur_node->position, cur_node->index);
  cur_node->g_score = 0.0;
  cur_node->f_score = this->lambda_heu_ * this->getDiagHeu(cur_node->position, end_pt.head(3));

  Eigen::Vector3d end_pos = end_pt.head(3);
  Eigen::Vector3i end_index;
  this->posToIndex(end_pos, end_index);

  this->open_set_.push(cur_node);
  this->open_set_map_.insert(make_pair(cur_node->index, cur_node));
  this->use_node_num_ += 1;

  const auto t1 = ros::Time::now();
  /* ---------- search loop ---------- */
  while (!this->open_set_.empty())
  {
    cur_node = this->open_set_.top();
    bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 && abs(cur_node->index(1) - end_index(1)) <= 1 && abs(cur_node->index(2) - end_index(2)) <= 1;
    if (reach_end)
    {
      this->backTrack(cur_node, end_pt);
      return SUCCEED;
    }

    // early termination if time up
    if ((ros::Time::now() - t1).toSec() > this->max_safe_search_time_)
    {
      this->early_terminate_cost_ = cur_node->g_score + this->getDiagHeu(cur_node->position, end_pt.head(3));
      ROS_WARN("HD-A* [Safe Search] -> Early termination");
      return FAIL;
    }

    this->open_set_.pop();
    this->open_set_map_.erase(cur_node->index);
    this->close_set_map_.insert(make_pair(cur_node->index, 1));
    this->iter_num_ += 1;

    Eigen::Vector3d cur_pos = cur_node->position;
    Eigen::Vector3d nbr_pos, step;
    Eigen::Vector3i nbr_idx, step_idx;
    Eigen::VectorXd nbr_pose(5);
    Eigen::Vector2d py;

    for (int dx = -1; dx <= 1; dx += 1)
      for (int dy = -1; dy <= 1; dy += 1)
        for (int dz = -1; dz <= 1; dz += 1)
        {
          if (dx == 0 && dy == 0 && dz == 0) continue;

          step_idx << dx, dy, dz;
          step << dx * this->resolution_, dy * this->resolution_, dz * this->resolution_;
          nbr_pos = cur_pos;
          this->posToIndex(nbr_pos, nbr_idx);
          nbr_idx += step_idx;
          this->indexToPos(nbr_idx, nbr_pos);

          // check safety
          if (nbr_pos(2) < this->safe_height_) continue;
          // if (this->map_->getDistance(nbr_pos) < 0.3 || this->map_->getInflateOccupancy_hc(nbr_pos) != 0 || this->map_->getOccupancy(nbr_pos) != SDFMap::FREE) continue;
          if (this->map_->getInflateOccupancy(nbr_pos) != 0) continue;

          bool safe = true;
          Eigen::Vector3d dir = nbr_pos - cur_pos;
          double len = dir.norm();
          dir.normalize();
          for (double l = this->resolution_; l < len; l += this->resolution_)
          {
            Eigen::Vector3d ckpt = cur_pos + l * dir;
            if (ckpt(2) < this->safe_height_)
            {
              safe = false;
              break;
            }
            // if (this->map_->getDistance(ckpt) < 0.3 || this->map_->getInflateOccupancy_hc(ckpt) != 0 || this->map_->getOccupancy(ckpt) != SDFMap::FREE)
            if (this->map_->getInflateOccupancy(ckpt) != 0)
            {
              safe = false;
              break;
            }
          }
          if (!safe) continue;

          // check not in close set
          if (this->close_set_map_.find(nbr_idx) != close_set_map_.end()) continue;

          bool occlusion = this->interpolatePitchYaw(nbr_pos, false, py);
          if (!occlusion) nbr_pose << nbr_pos, py(0), py(1);

          HDNodePtr neighbor;
          double tmp_g_score = step.norm() + cur_node->g_score;
          auto node_iter = this->open_set_map_.find(nbr_idx);
          if (node_iter == this->open_set_map_.end()) 
          {
            neighbor = this->path_node_pool_[use_node_num_];
            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_) 
            {
              ROS_ERROR("run out of node pool.");
              return FAIL;
            }
            neighbor->index = nbr_idx;
            neighbor->position = nbr_pos;
            neighbor->pose = nbr_pose;
          } 
          else if (tmp_g_score < node_iter->second->g_score)
           {
            neighbor = node_iter->second;
          } 
          else continue;

          neighbor->parent = cur_node;
          neighbor->g_score = tmp_g_score;
          neighbor->f_score = tmp_g_score + this->lambda_heu_ * this->getDiagHeu(nbr_pos, end_pos);
          this->open_set_.push(neighbor);
          this->open_set_map_[nbr_idx] = neighbor;
        } 
  }

  return FAIL;
}

int HDAstar::hcSafeSearch(const Eigen::VectorXd& start_pt, const Eigen::VectorXd& end_pt, bool high_tolerance)
{
  double max_time = high_tolerance == true ? this->max_hc_search_time_ : this->max_safe_search_time_;

  this->start_pose_ = start_pt;
  this->end_pose_ = end_pt;
  this->getAttitudeGap();

  bool line_check = this->linePreCheck(false, true);
  if (line_check) return SUCCEED;

  HDNodePtr cur_node = this->path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->position = start_pt.head(3);
  cur_node->pose = start_pt;
  this->map_->posToIndex_hc(cur_node->position, cur_node->index);
  cur_node->g_score = 0.0;
  cur_node->f_score = this->lambda_heu_ * this->getDiagHeu(cur_node->position, end_pt.head(3));

  Eigen::Vector3d end_pos = end_pt.head(3);
  Eigen::Vector3i end_index;
  this->map_->posToIndex_hc(end_pos, end_index);

  this->open_set_.push(cur_node);
  this->open_set_map_.insert(make_pair(cur_node->index, cur_node));
  this->use_node_num_ += 1;

  const auto t1 = ros::Time::now();
  /* ---------- search loop ---------- */
  while (!this->open_set_.empty())
  {
    cur_node = this->open_set_.top();
    bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 && abs(cur_node->index(1) - end_index(1)) <= 1 && abs(cur_node->index(2) - end_index(2)) <= 1;
    if (reach_end)
    {
      this->backTrack(cur_node, end_pt);
      return SUCCEED;
    }

    // early termination if time up
    if ((ros::Time::now() - t1).toSec() > max_time)
    {
      this->early_terminate_cost_ = cur_node->g_score + this->getDiagHeu(cur_node->position, end_pt.head(3));
      ROS_WARN("HD-A* [HC Safe Search] -> Early termination");
      return FAIL;
    }

    this->open_set_.pop();
    this->open_set_map_.erase(cur_node->index);
    this->close_set_map_.insert(make_pair(cur_node->index, 1));
    this->iter_num_ += 1;

    Eigen::Vector3d cur_pos = cur_node->position;
    Eigen::Vector3d nbr_pos, step;
    Eigen::Vector3i nbr_idx, step_idx;
    Eigen::VectorXd nbr_pose(5);
    Eigen::Vector2d py;

    for (int dx = -1; dx <= 1; dx += 1)
      for (int dy = -1; dy <= 1; dy += 1)
        for (int dz = -1; dz <= 1; dz += 1)
        {
          if (dx == 0 && dy == 0 && dz == 0) continue;

          step_idx << dx, dy, dz;
          step << dx * this->resolution_, dy * this->resolution_, dz * this->resolution_;
          nbr_pos = cur_pos;
          this->posToIndex(nbr_pos, nbr_idx);
          nbr_idx += step_idx;
          this->indexToPos(nbr_idx, nbr_pos);

          // check safety
          if (nbr_pos(2) < this->safe_height_) continue;
          if (this->map_->getInflateOccupancy_hc(nbr_pos) != 0) continue;
          // if (this->map_->getDistance_hc(nbr_pos) < 1.0) continue;

          bool safe = true;
          Eigen::Vector3d dir = nbr_pos - cur_pos;
          double len = dir.norm();
          dir.normalize();
          for (double l = this->resolution_; l < len; l += this->resolution_)
          {
            Eigen::Vector3d ckpt = cur_pos + l * dir;
            if (ckpt(2) < this->safe_height_)
            {
              safe = false;
              break;
            }
            if (this->map_->getInflateOccupancy_hc(ckpt) != 0)
            // if (this->map_->getDistance_hc(ckpt) < 1.0)
            {
              safe = false;
              break;
            }
          }
          if (!safe) continue;

          // check not in close set
          if (this->close_set_map_.find(nbr_idx) != close_set_map_.end()) continue;

          bool occlusion = this->interpolatePitchYaw(nbr_pos, false, py);
          if (!occlusion) nbr_pose << nbr_pos, py(0), py(1);

          HDNodePtr neighbor;
          double tmp_g_score = step.norm() + cur_node->g_score;
          auto node_iter = this->open_set_map_.find(nbr_idx);
          if (node_iter == this->open_set_map_.end()) 
          {
            neighbor = this->path_node_pool_[use_node_num_];
            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_) 
            {
              ROS_ERROR("run out of node pool.");
              return FAIL;
            }
            neighbor->index = nbr_idx;
            neighbor->position = nbr_pos;
            neighbor->pose = nbr_pose;
          } 
          else if (tmp_g_score < node_iter->second->g_score)
           {
            neighbor = node_iter->second;
          } 
          else continue;

          neighbor->parent = cur_node;
          neighbor->g_score = tmp_g_score;
          neighbor->f_score = tmp_g_score + this->lambda_heu_ * this->getDiagHeu(nbr_pos, end_pos);
          this->open_set_.push(neighbor);
          this->open_set_map_[nbr_idx] = neighbor;
        } 
  }

  return FAIL;
}

vector<Eigen::VectorXd> HDAstar::getPath()
{
  // path : [start, wp1, wp2, ..., end]
  return this->path_nodes_;
}

double HDAstar::pathLength(const vector<Eigen::VectorXd>& path) 
{
  double length = 0.0;
  if (path.size() < 2) return length;
  for (int i = 0; i < (int)path.size() - 1; ++i)
    length += (path[i + 1].head(3) - path[i].head(3)).norm();

  return length;
}

double HDAstar::pathOcclusionRate(const vector<Eigen::VectorXd>& path)
{
  if (path.size() < 2) return 0.0;
  if (this->occlusion_mat_.cols() == 0) return 0.0;

  auto wrap_angle = [](double a) {
    while (a < -M_PI) a += 2 * M_PI;
    while (a >  M_PI) a -= 2 * M_PI;
    return a;
  };

  const double step = this->resolution_;
  int total_samples = 0;
  int occluded_samples = 0;

  for (size_t i = 0; i + 1 < path.size(); ++i)
  {
    const Eigen::VectorXd& p0 = path[i];
    const Eigen::VectorXd& p1 = path[i + 1];
    const Eigen::Vector3d start = p0.head(3);
    const Eigen::Vector3d end = p1.head(3);
    const double seg_len = (end - start).norm();
    if (seg_len < 1e-6) continue;

    const double pitch0 = p0(3), pitch1 = p1(3);
    const double yaw0 = p0(4), yaw1 = p1(4);

    Eigen::Vector3d dir = (end - start).normalized();
    for (double d = 0.0; d <= seg_len; d += step)
    {
      double ratio = d / seg_len;
      Eigen::Vector3d pos = start + ratio * seg_len * dir;
      Eigen::Vector2d py;
      py(0) = wrap_angle(pitch0 + ratio * (pitch1 - pitch0));
      py(1) = wrap_angle(yaw0 + ratio * (yaw1 - yaw0));

      // visibility check
      this->percep_utils_->setPose_PY(pos, py(0), py(1));
      double h_dist = std::min(this->map_->getDistance_hc(pos), this->percep_utils_->max_dist_);
      Eigen::Matrix<double, 5, 4> H;
      this->percep_utils_->getHRepFov(H, h_dist, true);
      path_tools::fovMatMul(this->occlusion_results_, H, this->occlusion_mat_);
      bool occlusion = path_tools::checkNegativeColumns(this->occlusion_results_);

      ++total_samples;
      if (occlusion) ++occluded_samples;
    }
  }

  if (total_samples == 0) return 0.0;
  return static_cast<double>(occluded_samples) / static_cast<double>(total_samples);
}

// ! ------------------------------------- Tools ------------------------------------- ! //

void HDAstar::backTrack(const HDNodePtr& end_node, const Eigen::VectorXd& end)
{
  this->path_nodes_.push_back(end);
  this->path_nodes_.push_back(end_node->pose);
  HDNodePtr cur_node = end_node;
  while (cur_node->parent != NULL) 
  {
    cur_node = cur_node->parent;
    this->path_nodes_.push_back(cur_node->pose);
  }
  reverse(this->path_nodes_.begin(), this->path_nodes_.end());

  if((int)this->path_nodes_.size() > 2)
  {
    // raycasting -> shorten path
    vector<Eigen::VectorXd> shortened_path = {this->path_nodes_.front()};

    Eigen::Vector3i idx;
    for (int i=1; i<(int)this->path_nodes_.size(); ++i)
    {
      if ((this->path_nodes_[i].head(3) - shortened_path.back().head(3)).norm() > this->path_interval_)
        shortened_path.push_back(this->path_nodes_[i]);
      else
      {
        this->raycaster_->input(shortened_path.back().head(3), this->path_nodes_[i].head(3));
        while (this->raycaster_->nextId(idx)) 
        {
          if (this->map_->getInflateOccupancy(idx) != 0) 
          {
            shortened_path.push_back(this->path_nodes_[i]);
            break;
          }
        }
      }
    }
    shortened_path.push_back(this->path_nodes_.back());

    if ((int)shortened_path.size() > 2)
    {
      if ((shortened_path.back().head(3)-shortened_path[(int)shortened_path.size()-2].head(3)).norm() < this->path_interval_)
        shortened_path.erase(shortened_path.end()-2);
    }

    this->path_nodes_.clear();
    this->path_nodes_ = shortened_path;
  }

  return;
}

void HDAstar::posToIndex(const Eigen::Vector3d& pt, Eigen::Vector3i& idx)
{
  for (int i = 0; i < 3; ++i) idx(i) = floor((pt(i) - this->origin_(i)) * this->inv_resolution_);

  return;
}

void HDAstar::indexToPos(const Eigen::Vector3i& idx, Eigen::Vector3d& pt)
{
  for (int i = 0; i < 3; ++i) pt(i) = (idx(i) + 0.5) * this->resolution_ + this->origin_(i);

  return;
}

double HDAstar::getDiagHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2)
{
  double dx = fabs(x1(0) - x2(0));
  double dy = fabs(x1(1) - x2(1));
  double dz = fabs(x1(2) - x2(2));
  double h = 0.0;
  double diag = min(min(dx, dy), dz);
  dx -= diag;
  dy -= diag;
  dz -= diag;

  if (dx < 1e-4) 
  {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
  }
  if (dy < 1e-4) 
  {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
  }
  if (dz < 1e-4) 
  {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
  }

  return this->tie_breaker_ * h;
}

void HDAstar::getAttitudeGap()
{
  double pitch_start = this->start_pose_(3), pitch_end = this->end_pose_(3);
  double yaw_start = this->start_pose_(4), yaw_end = this->end_pose_(4);

  while(pitch_start < -M_PI) pitch_start += 2 * M_PI;
  while(pitch_start > M_PI) pitch_start -= 2 * M_PI;
  while(pitch_end < -M_PI) pitch_end += 2 * M_PI;
  while(pitch_end > M_PI) pitch_end -= 2 * M_PI;
  while(yaw_start < -M_PI) yaw_start += 2 * M_PI;
  while(yaw_start > M_PI) yaw_start -= 2 * M_PI;
  while(yaw_end < -M_PI) yaw_end += 2 * M_PI;
  while(yaw_end > M_PI) yaw_end -= 2 * M_PI;

  double pitch_diff = pitch_end - pitch_start;
  if (abs(pitch_diff) > M_PI)
  {
    int pitch_indi = pitch_diff > 0 ? -1 : 1;
    pitch_diff = pitch_indi*(2*M_PI - abs(pitch_diff));
  }
  this->pitch_gap_ = pitch_diff;

  double yaw_diff = yaw_end - yaw_start;
  if (abs(yaw_diff) > M_PI)
  {
    int yaw_indi = yaw_diff > 0 ? -1 : 1;
    yaw_diff = yaw_indi*(2*M_PI - abs(yaw_diff));
  }
  this->yaw_gap_ = yaw_diff;

  return;
}

void HDAstar::buildNeighborOffsets(int step)
{
  vis_neighbor_offsets_.clear();
  vis_neighbor_offsets_.reserve(26); // at most 3^3 - 1
  if (step <= 0) return;
  vis_neighbor_step_cache_ = step;
  const double res = this->resolution_;
  for (int dx = -step; dx <= step; dx += step)
    for (int dy = -step; dy <= step; dy += step)
      for (int dz = -step; dz <= step; dz += step)
      {
        if (dx == 0 && dy == 0 && dz == 0) continue;
        NeighborOffset off;
        off.idx_offset << dx, dy, dz;
        off.pos_offset = off.idx_offset.cast<double>() * res;
        off.step_len = off.pos_offset.norm();
        vis_neighbor_offsets_.push_back(off);
      }
}

bool HDAstar::interpolatePitchYaw(Eigen::Vector3d& cur_node_pos, bool en_vis, Eigen::Vector2d& pitch_yaw)
{
  bool occlusion = false;

  // cache hit can skip most work
  Eigen::Vector3i cache_idx;
  this->posToIndex(cur_node_pos, cache_idx);
  auto cache_it = this->vis_cache_.find(cache_idx);
  if (cache_it != this->vis_cache_.end())
  {
    pitch_yaw = cache_it->second.second;
    return cache_it->second.first;
  }

  double l_s = (cur_node_pos - this->start_pose_.head(3)).norm();
  double l_e = (cur_node_pos - this->end_pose_.head(3)).norm();
  double ratio = l_s / (l_s + l_e);

  pitch_yaw(0) = this->start_pose_(3) + ratio * this->pitch_gap_;
  pitch_yaw(1) = this->start_pose_(4) + ratio * this->yaw_gap_;

  while (pitch_yaw(0) < -M_PI) pitch_yaw(0) += 2 * M_PI;
  while (pitch_yaw(0) > M_PI) pitch_yaw(0) -= 2 * M_PI;
  while (pitch_yaw(1) < -M_PI) pitch_yaw(1) += 2 * M_PI;
  while (pitch_yaw(1) > M_PI) pitch_yaw(1) -= 2 * M_PI;

  if (!en_vis)
  {
    this->vis_cache_[cache_idx] = std::make_pair(false, pitch_yaw);
    return occlusion;
  }
  // Only run expensive visibility check when cache missed.
  // Use position-based query so we get the correct hc index; cache_idx comes from the
  // regular grid and does not align with hc resolution.
  double h_dist = this->map_->getDistance_hc(cur_node_pos);
  if (h_dist < 0) h_dist = this->percep_utils_->max_dist_;  // fallback when outside hc map
  h_dist = min(h_dist, this->percep_utils_->max_dist_);
  this->percep_utils_->setPose_PY(cur_node_pos, pitch_yaw(0), pitch_yaw(1));
  Eigen::Matrix<double, 5, 4> H;
  this->percep_utils_->getHRepFov(H, h_dist, true);
  path_tools::fovMatMul(this->occlusion_results_, H, this->occlusion_mat_);
  occlusion = path_tools::checkNegativeColumns(this->occlusion_results_);

  if (occlusion)
  {
    Eigen::RowVectorXd max_per_col = this->occlusion_results_.colwise().maxCoeff();
    vector<int> occlusion_idx;
    Eigen::Vector3d occlusion_pt;
    double cur2occlusion;

    double left_d_yaw, right_d_yaw, top_d_pitch, bottom_d_pitch;
    double max_left_d_yaw = 1e5, max_right_d_yaw = 1e-5, max_top_d_pitch = 1e5, max_bottom_d_pitch = -1e5;
    double d_yaw, d_pitch;

    for (int i=0; i<(int)this->occlusion_results_.cols(); ++i)
    {
      if (max_per_col(i) > 0) continue;
      occlusion_idx.push_back(i);
      occlusion_pt = this->occlusion_mat_.col(i).head(3);

      cur2occlusion = (cur_node_pos - occlusion_pt).norm();
      top_d_pitch = -asin(abs(this->occlusion_results_(0, i))/cur2occlusion);
      bottom_d_pitch = asin(abs(this->occlusion_results_(1, i))/cur2occlusion);
      left_d_yaw = -asin(abs(this->occlusion_results_(2, i))/cur2occlusion);
      right_d_yaw = asin(abs(this->occlusion_results_(3, i))/cur2occlusion);

      max_left_d_yaw = min(max_left_d_yaw, left_d_yaw);
      max_right_d_yaw = max(max_right_d_yaw, right_d_yaw);
      max_top_d_pitch = min(max_top_d_pitch, top_d_pitch);
      max_bottom_d_pitch = max(max_bottom_d_pitch, bottom_d_pitch);
    }

    d_yaw = abs(max_left_d_yaw) < abs(max_right_d_yaw) ? max_left_d_yaw : max_right_d_yaw;
    d_pitch = abs(max_top_d_pitch) < abs(max_bottom_d_pitch) ? max_top_d_pitch : max_bottom_d_pitch;

    double allowed_pitch_adjust = (d_pitch * this->pitch_gap_) > 0 ? (1.0-ratio)*this->pitch_gap_ : ratio*this->pitch_gap_;
    double allowed_yaw_adjust = (d_yaw * this->yaw_gap_) > 0 ? (1.0-ratio)*this->yaw_gap_ : ratio*this->yaw_gap_;

    if (abs(d_pitch) < abs(allowed_pitch_adjust) && abs(d_yaw) < abs(allowed_yaw_adjust))
    {
      double temp_pitch = pitch_yaw(0) + d_pitch;
      double temp_yaw = pitch_yaw(1) + d_yaw;
      this->percep_utils_->setPose_PY(cur_node_pos, temp_pitch, temp_yaw);
      Eigen::Matrix<double, 5, 4> H_d;
      this->percep_utils_->getHRepFov(H_d, h_dist, true);
      path_tools::fovMatMul(this->occlusion_results_, H_d, this->occlusion_mat_);
      occlusion = path_tools::checkNegativeColumns(this->occlusion_results_);
      if (!occlusion)
      {
        pitch_yaw(0) = temp_pitch;
        pitch_yaw(1) = temp_yaw;
      }
    }
  }

  // store result in cache
  this->vis_cache_[cache_idx] = std::make_pair(occlusion, pitch_yaw);

  return occlusion;
}

bool HDAstar::linePreCheck(bool en_visibility, bool en_hc)
{
  bool qualified = true;

  vector<Eigen::VectorXd> dense_samples = {this->start_pose_};
  Eigen::Vector3d dir = (this->end_pose_.head(3) - this->start_pose_.head(3)).normalized();
  double length = (this->end_pose_.head(3) - this->start_pose_.head(3)).norm();
  double step = en_visibility == true ? this->search_step_ * this->resolution_ : this->resolution_;
  Eigen::Vector3d cur_pos;
  Eigen::Vector2d cur_py;
  Eigen::VectorXd cur_pose(5);

  for (double dl = step; dl < length; dl += step)
  {
    cur_pos = this->start_pose_.head(3) + dl * dir;
    int inf = en_hc == true ? this->map_->getInflateOccupancy_hc(cur_pos) : this->map_->getInflateOccupancy(cur_pos);
    if (cur_pos(2) < this->safe_height_ || inf != 0)
    {
      qualified = false;
      break;
    }

    bool occlusion = this->interpolatePitchYaw(cur_pos, en_visibility, cur_py);
    if (occlusion)
    {
      qualified = false;
      break;
    }

    cur_pose << cur_pos, cur_py(0), cur_py(1);
    dense_samples.push_back(cur_pose);
  }
  dense_samples.push_back(this->end_pose_);

  if (!qualified) return qualified;

  vector<Eigen::VectorXd> sparse_samples = {dense_samples.front()};
  for (int i = 1; i < (int)dense_samples.size(); ++i)
  {
    if ((dense_samples[i].head(3) - sparse_samples.back().head(3)).norm() > this->path_interval_)
      sparse_samples.push_back(dense_samples[i]);
  }
  sparse_samples.push_back(dense_samples.back());

  if ((int)sparse_samples.size() > 2)
  {
    if ((sparse_samples.back().head(3)-sparse_samples[(int)sparse_samples.size()-2].head(3)).norm() < this->path_interval_)
      sparse_samples.erase(sparse_samples.end()-2);
  }

  this->path_nodes_.clear();
  this->path_nodes_ = sparse_samples;

  // cout << "line check success, " << this->path_nodes_.size() << " samples." << endl;

  return qualified;
}

} // namespace fc_vision
