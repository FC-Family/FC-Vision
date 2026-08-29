/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Mar. 2025
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the tools functions of visibility-aware replanning in FC-Vision.
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

#include <limits>

namespace fc_vision
{
void VisibilityReplan::fps(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int num_sample)
{
  if (!cloud || cloud->empty() || num_sample <= 0) return;
  if (static_cast<int>(cloud->size()) <= num_sample) return;

  const size_t point_count = cloud->size();
  std::vector<double> min_squared_distances(point_count, std::numeric_limits<double>::infinity());
  pcl::PointCloud<pcl::PointXYZ>::Ptr sampled(new pcl::PointCloud<pcl::PointXYZ>);
  sampled->reserve(num_sample);

  size_t selected_index = 0;
  for (int sample_index = 0; sample_index < num_sample; ++sample_index)
  {
    const pcl::PointXYZ& selected = cloud->points[selected_index];
    sampled->push_back(selected);

    double farthest_distance = -1.0;
    size_t farthest_index = selected_index;
    for (size_t i = 0; i < point_count; ++i)
    {
      const double dx = cloud->points[i].x - selected.x;
      const double dy = cloud->points[i].y - selected.y;
      const double dz = cloud->points[i].z - selected.z;
      const double squared_distance = dx * dx + dy * dy + dz * dz;
      min_squared_distances[i] = std::min(min_squared_distances[i], squared_distance);
      if (min_squared_distances[i] > farthest_distance)
      {
        farthest_distance = min_squared_distances[i];
        farthest_index = i;
      }
    }
    selected_index = farthest_index;
  }

  cloud = sampled;
}

bool VisibilityReplan::vpOpt(int& idx, Eigen::VectorXd& vp, Eigen::Matrix4Xd& vis_st, vector<int>& vis_st_idx, bool start)
{
  // auto t1 = chrono::high_resolution_clock::now();

  bool opt_success = false;

  // * 0. only collision
  if ((int)vis_st.cols() == 0)
  {
    Eigen::Vector3d vp_pos = vp.head(3);
    Eigen::Vector2d vp_py = vp.tail(2);
    Eigen::Vector3d vp_dir = path_tools::pyToVec(vp_py);

    bool safe = true;
    if (this->map_->getOccupancy(vp_pos) == SDFMap::OCCUPANCY::UNKNOWN || this->map_->getInflateOccupancy(vp_pos) == 1)
    {
      Eigen::Vector3d cur_pos;
      double hc_esdf = this->map_->getDistance_hc(vp_pos);
      double search_range = max(hc_esdf-this->grid_inf_-this->drone_radius_, 0.0);
      for (double dl=this->drone_radius_; dl<search_range; dl+=this->drone_radius_)
      {
        cur_pos = vp_pos + dl * vp_dir;
        if (this->map_->getOccupancy(cur_pos) == SDFMap::OCCUPANCY::FREE && this->map_->getInflateOccupancy(cur_pos) != 1)
        {
          safe = true;
          vp.head(3) = vp_pos;
          break;
        }
      }
    }

    ROS_INFO("[Replanning][VP-Opt] VP (%lf, %lf, %lf) only collision check, safe: %d.", vp_pos(0), vp_pos(1), vp_pos(2), safe);

    if(safe) opt_success = true;

    return opt_success;
  }

  // * 1. optimization space preparation
  Eigen::Vector3d median_vis_st = this->findMedianPoint(vis_st);
  Eigen::Vector3d vis_dir = (vp.head(3) - median_vis_st).normalized();
  double dist = (median_vis_st - vp.head(3)).norm();
  double radius = 0.0;
  for (int i=0; i<(int)vis_st.cols(); ++i)
  {
    Eigen::Vector3d pt = vis_st.col(i).head(3);
    double temp_r = (pt - median_vis_st).norm();
    radius = max(radius, temp_r);
  }
  double sample_angle_half = atan(radius / dist);
  double fov_max = max(this->half_fov_top_angle_, this->half_fov_left_angle_) + 1e-3;

  // sample_angle_half = min(sample_angle_half, fov_max);

  sample_angle_half = fov_max;

  vector<double> sample_region = {median_vis_st(0), median_vis_st(1), median_vis_st(2), vis_dir(0), vis_dir(1), vis_dir(2), dist, this->percep_utils_->max_dist_, 2*sample_angle_half};
  this->sample_regions_.push_back(sample_region);

  // * 2. get initial guesses
  Eigen::Vector3d x_axis(1, 0, 0);
  Eigen::Matrix3d rot_mat = path_tools::getRotMat(x_axis, vis_dir);
  Eigen::Vector3d init_vec = -vis_dir;
  Eigen::Vector2d init_py = path_tools::vecToPy(init_vec);

  vector<PolarCoord> initial_guesses;
  vector<Eigen::Vector3d> initial_guesses_pos;
  Eigen::Vector3d sample_world_pos, sample_world_dir;
  Eigen::Vector2d sample_world_py;
  PolarCoord guess_vp;
  Eigen::VectorXd guess_vp_vec(5);
  for (auto sample : this->plan_data_.local_samples_template)
  {
    if (abs(sample.theta) > (sample_angle_half) || abs(sample.phi) > (sample_angle_half)) continue;

    this->genSamplesGroup(sample);
    for (auto x : this->plan_data_.independent_samples_group)
    {
      double scale = this->percep_utils_->max_dist_ / x.r;
      sample_world_pos = rot_mat * scale * x.vec_xyz + median_vis_st;
      sample_world_py(1) = init_py(1) + x.theta;
      sample_world_py(0) = init_py(0) - x.phi;

      if (sample_world_pos(2) > this->ceil_) 
      {
        sample_world_pos(2) = this->ceil_ - 1;
        sample_world_py(0) = 8.0 / 180.0 * M_PI;
      }

      sample_world_dir = path_tools::pyToVec(sample_world_py);
      if (sample_world_py(0) > this->pitch_upper_) sample_world_py(0) = this->pitch_upper_;
      if (sample_world_py(0) < this->pitch_lower_) sample_world_py(0) = this->pitch_lower_;
      guess_vp_vec << sample_world_pos(0), sample_world_pos(1), sample_world_pos(2), sample_world_py(0), sample_world_py(1);

      guess_vp.r = scale * x.r;
      guess_vp.theta = sample_world_py(1);
      guess_vp.phi = sample_world_py(0);
      guess_vp.vec_xyz = sample_world_pos;
      guess_vp.vec_dir = sample_world_dir;
      guess_vp.vec_vp = guess_vp_vec;
      guess_vp.push_dist = 0.0;

      initial_guesses.push_back(guess_vp);
      initial_guesses_pos.push_back(sample_world_pos);
    }
  }

  cout << "initial independent viewpoint candidates : " << initial_guesses.size() << endl;

  this->world_samples_.push_back(initial_guesses_pos);

  // * 3. optimize position considering collision & occlusion
  vector<PolarCoord> opted_candidates;
  vector<Eigen::Vector3d> opted_candidates_pos;

  int num_opt = 0;

  Eigen::VectorXd opt_vp_vec(5);
  double opt_dist = 0.0;
  bool is_opt_pos = false;
  for (int i=0; i<(int)initial_guesses.size(); ++i)
  {
    opt_vp_vec = initial_guesses[i].vec_vp;
    is_opt_pos = this->optPosAlongLine(vp, opt_vp_vec, opt_dist, initial_guesses[i].vec_dir);

    if (is_opt_pos)
    {
      num_opt++;

      initial_guesses[i].vec_xyz = opt_vp_vec.head(3);
      initial_guesses[i].vec_vp.head(3) = opt_vp_vec.head(3);
      initial_guesses[i].push_dist = opt_dist;
      initial_guesses_pos[i] = opt_vp_vec.head(3);

      if (this->first_temp_)
      {
        opted_candidates.push_back(initial_guesses[i]);
        opted_candidates_pos.push_back(initial_guesses_pos[i]);
        this->first_temp_ = false;
      }
      else
      {
        if (this->map_->getOccupancy(initial_guesses[i].vec_xyz) == SDFMap::OCCUPANCY::FREE)
        {
          opted_candidates.push_back(initial_guesses[i]);
          opted_candidates_pos.push_back(initial_guesses_pos[i]);
        }
      }
    }
  }

  cout << "collision & occlusion free independent viewpoint candidates : " << num_opt << endl;
  cout << "FREE optimized independent viewpoint candidates : " << opted_candidates.size() << endl;

  // * 4. update position via maximizing visibility
  Eigen::Vector3d updated_pos;
  vector<PolarCoord> updated_opted_candidates;
  vector<Eigen::Vector3d> updated_opted_candidates_pos;
  for (int i=0; i<(int)opted_candidates.size(); ++i)
  {
    bool positive_contribution = this->calVisibleP(opted_candidates[i], vis_st, dist);
    if (positive_contribution && opted_candidates[i].vec_xyz(2) < this->ceil_)
    // if (positive_contribution)
    {
      updated_opted_candidates.push_back(opted_candidates[i]);
      updated_opted_candidates_pos.push_back(opted_candidates[i].vec_xyz);
    }
  }
  opted_candidates = updated_opted_candidates;
  opted_candidates_pos = updated_opted_candidates_pos;

  cout << "visibility positive independent viewpoint candidates : " << opted_candidates.size() << endl;

  // * 5. determine the best viewpoint sample
  Eigen::VectorXd best_sample(5);
  if (start) best_sample = vp;
  else 
  {
    bool find_best = this->getBestSample(opted_candidates, idx, best_sample);
    if (!find_best)
    {
      ROS_INFO("\033[33m[Replanning][Process] independent viewpoint opt FAILED.\033[33m");
      return opt_success;
    }
  }

  // * 6. update global coverage states & filter out useless samples
  this->updateCoverageStates(opted_candidates, best_sample, vis_st_idx, start);

  opt_success = true;
  vp = best_sample;

  opted_candidates_pos = {best_sample.head(3)};
  // this->world_samples_.push_back(opted_candidates_pos);

  // auto t2 = chrono::high_resolution_clock::now();
  // chrono::duration<double, milli> algo_ms = t2 - t1;
  // double algo_time = (double)algo_ms.count();
  // ROS_INFO("\033[33m[Replanning][Process] independent viewpoint opt time -> %lf ms.\033[33m", algo_time);

  return opt_success;
}

/* Weiszfeld algorithm */
Eigen::Vector3d VisibilityReplan::findMedianPoint(const Eigen::Matrix4Xd& points)
{
//   auto t1 = chrono::high_resolution_clock::now();

  Eigen::Vector3d median_point;
  int max_iter = 50;
  double tol = 1e-3;
  const int N = points.cols();

  if (N == 0) return Eigen::Vector3d::Zero();
    
  Eigen::Matrix3Xd xyz = points.topRows<3>();
  Eigen::Vector3d median = xyz.rowwise().mean();
    
  for (int iter = 0; iter < max_iter; ++iter) 
  {
    Eigen::Vector3d new_median = Eigen::Vector3d::Zero();
    double total_weight = 0;
    
    for (int i = 0; i < N; ++i) 
    {
      double dist = (xyz.col(i) - median).norm();
      if (dist < tol) continue;
        
      double weight = 1.0 / dist;
      new_median += weight * xyz.col(i);
      total_weight += weight;
    }
    
    if (total_weight > 0) new_median /= total_weight;
    
    if ((new_median - median).norm() < tol) break;
    
    median = new_median;
  }

  int best_idx = 0;
  double min_dist = (xyz.col(0) - median).norm();
  for (int i = 1; i < N; ++i) 
  {
    double dist = (xyz.col(i) - median).norm();
    if (dist < min_dist) 
    {
      min_dist = dist;
      best_idx = i;
    }
  }

  median_point = xyz.col(best_idx);

//   auto t2 = chrono::high_resolution_clock::now();
//   chrono::duration<double, milli> algo_ms = t2 - t1;
//   double algo_time = (double)algo_ms.count();
//   ROS_INFO("\033[32m[Replanning][Preparation] find median time -> %lf ms with %d pts.\033[32m", algo_time, N);

  return median_point;
}
/* Samples in local frame : direction +x-axis */
void VisibilityReplan::localSamplesTemplate()
{
  /* theta θ : diff with +x-axis within xoy plane, phi φ : diff with +x-axis within θoz plane */
  double radius = 1.0, d_theta = this->theta_step_, d_phi = this->phi_step_;
  PolarCoord temp_sample;

  // add axis sample
  temp_sample.r = radius;
  temp_sample.theta = 0.0;
  temp_sample.phi = 0.0;
  temp_sample.vec_xyz = path_tools::polarToCartesian(temp_sample.r, temp_sample.theta, temp_sample.phi);
  this->plan_data_.local_samples_template.push_back(temp_sample);

  for (double cur_theta = 0; cur_theta < M_PI; cur_theta += d_theta)
  {
    for (double cur_phi = 0; cur_phi < M_PI; cur_phi += d_phi)
    {
      if (cur_theta == 0 && cur_phi == 0) continue;

      const double theta_variants[2] = {cur_theta, -cur_theta};
      const double phi_variants[2] = {cur_phi, -cur_phi};

      for (int i = 0; i < 2; ++i) 
      {
        for (int j = 0; j < 2; ++j)
        {
          temp_sample.r = radius;
          temp_sample.theta = theta_variants[i];
          temp_sample.phi = phi_variants[j];
          temp_sample.vec_xyz = path_tools::polarToCartesian(temp_sample.r, temp_sample.theta, temp_sample.phi);
          this->plan_data_.local_samples_template.push_back(temp_sample);
        }
      }
    }
  }

  return;
}

void VisibilityReplan::genSamplesGroup(PolarCoord& local_frame_sample)
{
  this->plan_data_.independent_samples_group.clear();

  // * 1. determine the size of sample group
  int num_theta, num_phi;
  double t_step, p_step;

  if (abs(local_frame_sample.theta) != 0)
  {
    num_theta = ceil(abs(local_frame_sample.theta) / this->theta_step_) + 1;
    t_step = abs(local_frame_sample.theta) / (num_theta - 1);
  }
  else
  {
    num_theta = 1;
    t_step = 0.0;
  }

  if (abs(local_frame_sample.phi) != 0)
  {
    num_phi = ceil(abs(local_frame_sample.phi) / this->phi_step_) + 1;
    p_step = abs(local_frame_sample.phi) / (num_phi - 1);
  }
  else
  {
    num_phi = 1;
    p_step = 0.0;
  }
  int num_group = num_theta * num_phi;
  this->plan_data_.independent_samples_group.resize(num_group);

  // * 2. generate sample group
  int theta_sign = local_frame_sample.theta > 0 ? -1 : 1;
  int phi_sign = local_frame_sample.phi > 0 ? -1 : 1;
  PolarCoord temp_sample = local_frame_sample;
  for (int i=0; i<num_theta; ++i)
    for (int j=0; j<num_phi; ++j)
    {
      temp_sample.theta = local_frame_sample.theta + theta_sign * i * t_step;
      temp_sample.phi = local_frame_sample.phi + phi_sign * j * p_step;
      temp_sample.vec_xyz = local_frame_sample.vec_xyz;
      this->plan_data_.independent_samples_group[i*num_phi+j] = temp_sample;
    }

  return;
}

bool VisibilityReplan::optPosAlongLine(const Eigen::VectorXd& input_vp, Eigen::VectorXd& vp, double& opt_dist, Eigen::Vector3d& line_dir)
{
  bool is_suc = false;
  opt_dist = 0.0;

  Eigen::Matrix<double, 4, Eigen::Dynamic> env_4d(4, this->plan_data_.env_mat.cols());
  env_4d = this->plan_data_.env_mat;
  Eigen::Vector3d vp_pos = vp.head(3), ori_vp_pos = vp.head(3), input_vp_pos = input_vp.head(3);
  this->percep_utils_->setPose_PY(vp_pos, vp(3), vp(4));
  Eigen::Matrix<double, 5, 4> H;
  double hc_esdf_vp = this->map_->getDistance_hc(vp_pos);
  this->percep_utils_->getHRepFov(H, hc_esdf_vp, false);
  Eigen::Matrix<double, 5, Eigen::Dynamic> results(5, env_4d.cols());
  path_tools::fovMatMul(results, H, env_4d);

  double safe_push = 1.2*this->grid_inf_;

  bool occlusion = path_tools::checkNegativeColumns(results);
  if (occlusion)
  {
    // * 1. occlusion-aware operation
    // top, bottom, left, right, far
    Eigen::RowVectorXd max_per_col = results.colwise().maxCoeff();
    double occlusion_move_dist = -1e5;
    double td_dist = 0.0, lr_dist = 0.0, far_dist = 0.0;
    double min_out_dist = 0.0;

    for (int i=0; i<(int)results.cols(); ++i)
    {
      if (max_per_col(i) > 0) continue;
      td_dist = (min(abs(results(0, i)), abs(results(1, i))) + sqrt(3) * safe_push)/sin(this->half_fov_top_angle_);
      lr_dist = (min(abs(results(2, i)), abs(results(3, i))) + sqrt(3) * safe_push)/sin(this->half_fov_left_angle_);
      far_dist = hc_esdf_vp - abs(results(4, i)) + sqrt(3) * safe_push;

      min_out_dist = min(td_dist, min(lr_dist, far_dist));
      occlusion_move_dist = max(occlusion_move_dist, min_out_dist);
    }
    occlusion_move_dist += 1e-1;
    vp_pos += occlusion_move_dist * line_dir;
    opt_dist = occlusion_move_dist;
  }

  // * 2. collision-aware operation
  bool collision = false;
  if (this->map_->getOccupancy(vp_pos) != SDFMap::OCCUPANCY::UNKNOWN)
  {
    double cur_esdf_vp = this->map_->getDistance(vp_pos); 
    if (cur_esdf_vp < 0)
    {
    Eigen::Vector3d push_pos = vp_pos + (abs(cur_esdf_vp) + sqrt(3) * safe_push) * line_dir;
    cur_esdf_vp = this->map_->getDistance(push_pos);
    if (cur_esdf_vp < 1e-1 && this->map_->getInflateOccupancy(push_pos) != 1) 
    {
      collision = true;
      vp_pos = push_pos;
    }
    }
  }

  // * 3. bound-aware : within map & safe flight height
  bool bound = true;
  if (!this->map_->isInMap(vp_pos) || vp_pos(2) < this->safe_height_ || this->map_->getDistance_hc(vp_pos) < this->drone_radius_) bound = false;

  if (!collision && bound) is_suc = true; 

  // * 4. reject cross-surface movement
  double resolution = this->map_->getResolution();

  double length = (vp_pos - ori_vp_pos).norm();
  for (double dl=resolution; dl<length; dl+=resolution)
  {
    Eigen::Vector3d inter_pos = ori_vp_pos + dl * line_dir;
    double step_hc_esdf = this->map_->getDistance_hc(inter_pos);
    if (abs(step_hc_esdf) < this->drone_radius_)
    {
      is_suc = false;
      break;
    }
  }

  double to_input_dist = (vp_pos - input_vp_pos).norm();
  Eigen::Vector3d to_input_dir = (vp_pos - input_vp_pos).normalized();
  for (double dl=resolution; dl<to_input_dist; dl+=resolution)
  {
    Eigen::Vector3d inter_pos = input_vp_pos + dl * to_input_dir;
    double step_hc_esdf = this->map_->getDistance_hc(inter_pos);
    if (abs(step_hc_esdf) < this->drone_radius_)
    {
      is_suc = false;
      break;
    }
  }

  // * 5. final update
  if (is_suc) vp.head(3) = vp_pos;

  return is_suc;
}

bool VisibilityReplan::calVisibleP(PolarCoord& sample, Eigen::Matrix4Xd& vis_st, double& init_dist)
{
  Eigen::VectorXd vp = sample.vec_vp;
  Eigen::Matrix<double, 4, Eigen::Dynamic> vis_st_4d(4, vis_st.cols());
  vis_st_4d = vis_st;
  this->percep_utils_->setPose_PY(vp.head(3), vp(3), vp(4));
  Eigen::Matrix<double, 5, 4> H;
  this->percep_utils_->getHRepFov(H, this->percep_utils_->max_dist_, false);
  Eigen::Matrix<double, 5, Eigen::Dynamic> results(5, vis_st.cols());
  path_tools::fovMatMul(results, H, vis_st);

  if (sample.push_dist == 0) this->maximizeVis(sample, results, init_dist);

//   Eigen::RowVectorXd max_per_col = results.colwise().maxCoeff();
//   Eigen::VectorXi bool_mask = (max_per_col.array() < 0).cast<int>();
//   int count = bool_mask.sum();

//   double visible_p = (double)count / (double)vis_st.cols();
//   sample.visible_proportion = visible_p;

  int count = 0;
  Eigen::Vector3d pt_pos;
  for (int i=0; i<(int)vis_st.cols(); ++i)
  {
    pt_pos = vis_st.col(i).head(3);
    if (this->percep_utils_->insideFOV(pt_pos))
    {
      bool visible = true;
      vector<Eigen::Vector3d> line_samples = visibility_utils::sampleLine(vp.head(3), pt_pos, this->vis_inf_, this->map_->getResolution());
      for (Eigen::Vector3d& sample_pt : line_samples)
      {
        double hc_esdf = this->map_->getDistance_hc(sample_pt);
        if (abs(hc_esdf) < 0.2 || this->map_->getOccupancy(sample_pt) == SDFMap::OCCUPANCY::OCCUPIED)
        {
          visible = false;
          break;
        }
      }

      if (visible) 
        count++;
    }
  }
  sample.visible_proportion = (double)count / (double)vis_st.cols();

  if (count == 0) return false;
  else return true;
}

void VisibilityReplan::maximizeVis(PolarCoord& sample, Eigen::Matrix<double, 5, Eigen::Dynamic>& results, double& init_dist)
{
  double max2far = results.row(results.rows() - 1).maxCoeff();
  double opt_range = max(this->percep_utils_->max_dist_ - init_dist, max2far);
  double opt_step = sqrt(3) * this->map_->getResolution();
  
  double push_value = 0.0, max_vis = -1e5, cur_vis = 0.0;
  Eigen::Vector3d cur_pos = sample.vec_xyz;
  Eigen::Matrix<double, 5, Eigen::Dynamic> updated_results(5, results.cols());
  for (double dl=0.0; dl<opt_range; dl+=opt_step)
  {
    cur_pos = sample.vec_xyz + dl * sample.vec_dir;
    if (this->map_->getOccupancy(cur_pos) != SDFMap::OCCUPANCY::UNKNOWN)
    {
      if (this->map_->getDistance(cur_pos) < 1e-1) continue;
    }

    Eigen::Matrix<double, 5, Eigen::Dynamic> temp_results = results;
    this->stepUpdateResults(results, dl, temp_results);
    Eigen::RowVectorXd max_per_col = temp_results.colwise().maxCoeff();
    Eigen::VectorXi bool_mask = (max_per_col.array() < 0).cast<int>();
    cur_vis = (double)bool_mask.sum() / (double)temp_results.cols();

    if (cur_vis > max_vis)
    {
      max_vis = cur_vis;
      push_value = dl;
      updated_results = temp_results;
    }
  }

  sample.push_dist = push_value;
  sample.vec_xyz = cur_pos;
  sample.vec_vp.head(3) = cur_pos;
  results = updated_results;

  return;
}

void VisibilityReplan::stepUpdateResults(Eigen::Matrix<double, 5, Eigen::Dynamic>& A, double l, Eigen::Matrix<double, 5, Eigen::Dynamic>& B)
{
  Eigen::Matrix<double, 5, Eigen::Dynamic> delta = (l * this->plan_data_.dl_vector).replicate(1, A.cols());
  B = A + delta;

  return;
}

bool VisibilityReplan::getBestSample(vector<PolarCoord>& opt_samples, int& idx, Eigen::VectorXd& best_vp)
{
  bool find_best = false;

  if (opt_samples.size() == 0) return find_best;

  int top_k = 0;
  // double vis_p_thresh = this->plan_data_.vis_rates[idx];
  // for (int i=0; i<(int)opt_samples.size(); ++i)
  // {
  //   if (opt_samples[i].visible_proportion >= vis_p_thresh) top_k++;
  // }
  // top_k = min(top_k, this->k_samples_);

  // if (top_k == 0) top_k = (int)opt_samples.size() > this->k_samples_ ? this->k_samples_ : (int)opt_samples.size();
  top_k = (int)opt_samples.size() > this->k_samples_ ? this->k_samples_ : (int)opt_samples.size();

  vector<PolarCoord> top_samples;
  vector<int> top_samples_idx;
  top_samples.reserve(top_k);
  top_samples_idx.reserve(top_k);

  vector<pair<PolarCoord, int>> indexed_samples;
  for (int i = 0; i < (int)opt_samples.size(); ++i) indexed_samples.emplace_back(opt_samples[i], i);

  for (int i = 0; i < top_k; ++i) 
  {
    int max_id = 0;
    double max_val = indexed_samples[0].first.visible_proportion;
    
    for (int j = 1; j < (int)indexed_samples.size(); ++j) 
    {
      if (indexed_samples[j].first.visible_proportion > max_val) 
      {
        max_val = indexed_samples[j].first.visible_proportion;
        max_id = j;
      }
    }
    
    top_samples.push_back(indexed_samples[max_id].first);
    top_samples_idx.push_back(indexed_samples[max_id].second);
    
    indexed_samples.erase(indexed_samples.begin() + max_id);
  }

  // cout << "Top " << top_k << " visible proportions : ";
  // for (auto s : top_samples) cout << s.visible_proportion << " , ";
  // cout << endl;

  if (top_k == 1)
  {
    best_vp = opt_samples[top_samples_idx.front()].vec_vp;
    opt_samples.erase(opt_samples.begin() + top_samples_idx.front());
  }
  else
  {
    vector<double> gains;
    for (int i=0; i<(int)top_samples.size(); ++i)
    {
      Eigen::Vector3d pt = top_samples[i].vec_xyz;
      double dist = (pt - this->input_path_[idx].head(3)).norm();
      double vis_p = top_samples[i].visible_proportion;

      double gain = vis_p + this->alpha_dist_ * (1.0 / exp(dist));

      gains.push_back(gain);
    }

    auto max_it = max_element(gains.begin(), gains.end());
    int max_index = distance(gains.begin(), max_it);

    // cout << "Visible proportions of max gain : ";
    // cout << top_samples[max_index].visible_proportion << " , dist to target : " << gains[max_index] << endl;

    best_vp = opt_samples[top_samples_idx[max_index]].vec_vp;
    opt_samples.erase(opt_samples.begin() + top_samples_idx[max_index]);
  }
  find_best = true;

  return find_best;
}

void VisibilityReplan::updateCoverageStates(vector<PolarCoord>& opt_samples, Eigen::VectorXd& best_vp, vector<int>& vis_st_idx, bool start)
{
  vector<Eigen::Vector4d> remaining_pts;
  Eigen::Vector4d pt;
  Eigen::Vector3d pt_3d;
  bool inside = false;
  Eigen::Vector3d vp_pos = best_vp.head(3);
  this->percep_utils_->setPose_PY(vp_pos, best_vp(3), best_vp(4));
  for(int i=0; i<(int)vis_st_idx.size(); ++i)
  {
    inside = false;
    pt = this->plan_data_.target_mat.col(vis_st_idx[i]);
    pt_3d = pt.head(3);

    if (this->plan_data_.target_cover_states[vis_st_idx[i]]) continue;

    if (this->percep_utils_->insideFOV(pt_3d)) 
    {
      bool visible = true;
      vector<Eigen::Vector3d> line_samples = visibility_utils::sampleLine(vp_pos, pt_3d, this->vis_inf_, this->map_->getResolution());
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
        inside = true;
        this->plan_data_.target_cover_states[vis_st_idx[i]] = true;

        if(start)
        {
          Eigen::Vector3d ray_dir = (pt_3d - vp_pos).normalized();
          double dist = (pt_3d - vp_pos).norm();
          Eigen::Vector3d pt_inf = vp_pos + (dist-this->vis_inf_) * ray_dir;

          Eigen::Vector3i idx;
          this->raycaster_->input(vp_pos, pt_inf);
          while (this->raycaster_->nextId(idx)) 
          {
            if (this->map_->getEnv(idx)) 
            {
              inside = false;
              this->plan_data_.target_cover_states[vis_st_idx[i]] = false;
              break;
            }
          }
        }
      }
    }

    if (!inside) remaining_pts.push_back(pt);
  }

  Eigen::Matrix<double, 4, Eigen::Dynamic> remaining_vis_st(4, remaining_pts.size());
  for (int i=0; i<(int)remaining_pts.size(); ++i) remaining_vis_st.col(i) = remaining_pts[i];

  for(int i=0; i<(int)opt_samples.size(); ++i)
  {
    this->percep_utils_->setPose_PY(opt_samples[i].vec_vp.head(3), opt_samples[i].vec_vp(3), opt_samples[i].vec_vp(4));
    Eigen::Matrix<double, 5, 4> H;
    this->percep_utils_->getHRepFov(H, this->percep_utils_->max_dist_, false);
    Eigen::Matrix<double, 5, Eigen::Dynamic> results(5, remaining_vis_st.cols());
    path_tools::fovMatMul(results, H, remaining_vis_st);

    bool vis = path_tools::checkNegativeColumns(results);

    if (vis) 
    {
      double min_dist = 1e4;
      for (auto p : this->plan_data_.all_vps)
      {
        double dist = (opt_samples[i].vec_xyz - p.head(3)).norm();
        min_dist = min(min_dist, dist);
      }
      if (min_dist < this->open_pool_range_) this->plan_data_.open_pool.push_back(opt_samples[i]);
    }
  }
  
  return;
}

void VisibilityReplan::addNewVpsFromOpenPool()
{
  // auto t1 = chrono::high_resolution_clock::now();

  // * 1. initialize set covering problem -> U: uncovered points, S: open pool
  for (int i=0; i<(int)this->plan_data_.target_cover_states.size(); ++i)
  {
    if (!this->plan_data_.target_cover_states[i]) this->plan_data_.uncovered_idx.push_back(i);
  }
  this->plan_data_.U_states.resize(this->plan_data_.uncovered_idx.size(), false);
  this->plan_data_.S_states.resize(this->plan_data_.open_pool.size(), true);
  this->plan_data_.U_S_mat.resize(this->plan_data_.uncovered_idx.size(), this->plan_data_.open_pool.size());
  this->plan_data_.U_S_mat.setZero();

  int min_num = 0;
  int u_id;
  Eigen::Vector3d u_pt;
  Eigen::VectorXd open_vp(5);
  for (int i=0; i<(int)this->plan_data_.open_pool.size(); ++i)
  {
    open_vp = this->plan_data_.open_pool[i].vec_vp;
    int cur_cover = 0;
    this->percep_utils_->setPose_PY(open_vp.head(3), open_vp(3), open_vp(4));
    for (int j=0; j<(int)this->plan_data_.uncovered_idx.size(); ++j)
    {
      u_id = this->plan_data_.uncovered_idx[j];
      u_pt = this->plan_data_.target_mat.col(u_id).head(3);
      bool visible = true;
      const vector<Eigen::Vector3d> line_samples = visibility_utils::sampleLine(
          open_vp.head(3), u_pt, this->vis_inf_, this->map_->getResolution());
      for (const Eigen::Vector3d& sample : line_samples)
      {
        if (std::abs(this->map_->getDistance_hc(sample)) < 0.2)
        {
          visible = false;
          break;
        }
      }
      if (this->percep_utils_->insideFOV(u_pt) && visible)
      {
        this->plan_data_.U_S_mat(j, i) += 1;
        cur_cover++;
      }
    }
    if (cur_cover < min_num) this->plan_data_.S_states[i] = false;
  }

  int uncovered_in_sample = 0;
  for (int i=0; i<(int)this->plan_data_.U_S_mat.rows(); ++i)
  {
    if (this->plan_data_.U_S_mat.row(i).sum() == 0) uncovered_in_sample++;
  }

  // * 2. greedy selection
  int termination = uncovered_in_sample;
  int cur_max_s_idx = -1;
  int cur_uncovered = count(this->plan_data_.U_states.begin(), this->plan_data_.U_states.end(), false);

  double remaining_rate = (double)cur_uncovered / (double)this->plan_data_.target_cover_states.size();
  // cout << "cur uncovered : " << cur_uncovered << ", remaining rate : " << remaining_rate << endl;

  if (remaining_rate < 0.05) return;
  
  vector<double> S_set_unit_gain;
  int add_num = 0;
  while (cur_uncovered > termination && add_num == 0)
  {
    vector<double> S_count(this->plan_data_.S_states.size(), 0);
    vector<double> S_unit_gain(this->plan_data_.S_states.size(), 0.0);
    for (int i=0; i<(int)this->plan_data_.S_states.size(); ++i)
    {
      if (!this->plan_data_.S_states[i]) continue;
      S_count[i] = this->calCoverGain(i, S_unit_gain[i]);
    }

    // * if all gains are 0 -> break
    if (*std::max_element(S_count.begin(), S_count.end()) == 0) break;

    double max_val = *std::max_element(S_count.begin(), S_count.end());
    vector<int> max_indices;
    for (int i = 0; i < (int)S_count.size(); ++i) 
    {
      if (S_count[i] == max_val) max_indices.push_back(i);
    }
    if ((int)max_indices.size() == 1) cur_max_s_idx = max_indices.front();
    else
    {
      vector<double> dist2all(max_indices.size(), 0.0);
      for (int i=0; i<(int)max_indices.size(); ++i)
      {
        Eigen::Vector3d pt = this->plan_data_.open_pool[max_indices[i]].vec_xyz;
        double min2all = 1e5;
        for (int i=0; i<(int)this->plan_data_.all_vps.size(); ++i)
        {
          for (int j=0; j<(int)this->plan_data_.all_vps.size(); ++j)
          {
            if (i == j) continue;
            double dist_i = (this->plan_data_.all_vps[i].head(3) - pt).norm();
            double dist_j = (this->plan_data_.all_vps[j].head(3) - pt).norm();
            min2all = min(min2all, dist_i + dist_j - (this->plan_data_.all_vps[i].head(3) - this->plan_data_.all_vps[j].head(3)).norm());
          }
        }
        dist2all[i] = min2all;
      }

      auto min_it = min_element(dist2all.begin(), dist2all.end());
      int min_index = distance(dist2all.begin(), min_it);
      cur_max_s_idx = max_indices[min_index];
    }

    this->plan_data_.S_set.push_back(this->plan_data_.open_pool[cur_max_s_idx].vec_vp);
    S_set_unit_gain.push_back(S_unit_gain[cur_max_s_idx]);
    add_num++;

    this->plan_data_.S_states[cur_max_s_idx] = false;
    for (int j=0; j<(int)this->plan_data_.U_S_mat.rows(); ++j)
    {
      if (this->plan_data_.U_S_mat(j, cur_max_s_idx) > 0)
      {
        this->plan_data_.U_S_mat.row(j).setZero();
        this->plan_data_.U_states[j] = true;
        this->plan_data_.target_cover_states[this->plan_data_.uncovered_idx[j]] = true;
      }
    }

    cur_uncovered = count(this->plan_data_.U_states.begin(), this->plan_data_.U_states.end(), false);
  }

  vector<bool> good_new_vps(this->plan_data_.S_set.size(), true);
  for (int i=0; i<(int)this->plan_data_.S_set.size(); ++i)
  {
    double unit_gain = S_set_unit_gain[i];
    if (unit_gain < 0.05) good_new_vps[i] = false;
  }
  for (int i=0; i<(int)good_new_vps.size(); ++i)
  {
    if (!good_new_vps[i]) this->plan_data_.S_set.erase(this->plan_data_.S_set.begin() + i);
  }

  // auto t2 = chrono::high_resolution_clock::now();
  // chrono::duration<double, milli> algo_ms = t2 - t1;
  // double algo_time = (double)algo_ms.count();
  // ROS_INFO("\033[33m[Replanning][Process] add new viewpoints time -> %lf ms.\033[33m", algo_time);

  return;
}
// TODO : fit more stable gain function in set covering problem
double VisibilityReplan::calCoverGain(int& idx_in_open_pool, double& unit_gain)
{
  double gain = 0.0;
  Eigen::Vector3d pt = this->plan_data_.open_pool[idx_in_open_pool].vec_xyz;
  
  // * 1. coverage gain
  double c_num = (double)this->plan_data_.U_S_mat.col(idx_in_open_pool).sum();
  double c_gain_total = c_num / (double)this->plan_data_.target_cover_states.size();
  double c_gain = c_num / (double)this->plan_data_.U_S_mat.rows();

  // * 2. position gain -> insertion cost
  double p_gain = 1e5;
  for (int i=0; i<(int)this->plan_data_.all_vps.size(); ++i)
  {
    for (int j=0; j<(int)this->plan_data_.all_vps.size(); ++j)
    {
      if (i == j) continue;
      double dist_i = (this->plan_data_.all_vps[i].head(3) - pt).norm();
      double dist_j = (this->plan_data_.all_vps[j].head(3) - pt).norm();
      p_gain = min(p_gain, dist_i + dist_j - (this->plan_data_.all_vps[i].head(3) - this->plan_data_.all_vps[j].head(3)).norm());
    }
  }
  unit_gain = p_gain < 0.5 ? 0 : c_gain_total / (p_gain + 1e-3);
  p_gain = 1.0 / exp(p_gain);

  // * 3. weighted sum
  gain = c_gain == 0 ? 0 : (c_gain + this->alpha_dist_ * p_gain);

  return gain;
}
// ! ------------------- Debug ------------------- ! //
void VisibilityReplan::debugFunc()
{
  // ! test
  Eigen::VectorXd viewpoint = this->input_path_[1];
  Eigen::Vector3d vp_pos = viewpoint.head(3);
  double vp_pitch = viewpoint(3), vp_yaw = viewpoint(4);

  // ? get hc esdf
  Eigen::Vector3d hc_esdf_grad;
  double hc_esdf_vp = this->map_->getDistWithGradHCMap(vp_pos, hc_esdf_grad);

  Eigen::Vector2d vp_pitch_yaw(vp_pitch, vp_yaw);
  Eigen::Vector3d vp_dir = path_tools::pyToVec(vp_pitch_yaw);
  double diff_angle = abs(acos(vp_dir.dot(hc_esdf_grad.normalized())));
  diff_angle = min(diff_angle, M_PI - diff_angle) * 180.0 / M_PI;
  ROS_INFO("\033[35m[Replanning][Debug] hc esdf diff vp dir -> %lf.\033[35m", diff_angle);

//   hc_esdf_vp = hc_esdf_vp * cos(diff_angle * M_PI / 180.0);

  ROS_INFO("\033[35m[Replanning][Debug] hc esdf vp -> %lf.\033[35m", hc_esdf_vp);

  double fov_depth = min(hc_esdf_vp, this->percep_utils_->max_dist_);

  auto h_t1 = chrono::high_resolution_clock::now();
  
  // ? fast func to get H of fov
  debug_H_.setZero();
  this->percep_utils_->setPose_PY(vp_pos, vp_pitch, vp_yaw);
  this->percep_utils_->getHRepFov(debug_H_, fov_depth, true);

  // cout << "before : " << endl;
  // cout << debug_H_ << endl;

  // double yaw_mv = 20.0 * M_PI / 180.0;
  // Eigen::Matrix3d R_yaw;
  // R_yaw << cos(yaw_mv), -sin(yaw_mv), 0.0, sin(yaw_mv), cos(yaw_mv), 0.0, 0.0, 0.0, 1.0;

  // double pitch_mv = 10.0 * M_PI / 180.0;
  // Eigen::Matrix3d R_pitch;
  // R_pitch << cos(pitch_mv), 0.0, -sin(pitch_mv), 0.0, 1.0, 0.0, sin(pitch_mv), 0.0, cos(pitch_mv);

  // Eigen::Matrix3d R = R_yaw * R_pitch;

  // for (int i = 0; i < 5; ++i) 
  // {
  //   Eigen::Vector3d n = debug_H_.block<1, 3>(i, 0);
  //   double d = debug_H_(i, 3);
  //   Eigen::Vector3d n_rotated = R * n;
  //   double d_rotated = -n_rotated.dot(vp_pos);
  //   debug_H_.block<1, 3>(i, 0) = n_rotated;
  //   debug_H_(i, 3) = d_rotated;
  // }
  // debug_H_(4, 3) -= fov_depth;

  // cout << "after : " << endl;
  // cout << debug_H_ << endl;

  auto h_t2 = chrono::high_resolution_clock::now();
  chrono::duration<double, milli> h_ms = h_t2 - h_t1;
  double h_time = (double)h_ms.count();
  ROS_INFO("\033[35m[Replanning][Debug] get H time -> %lf ms.\033[35m", h_time);

  int N = 1;
  Eigen::Matrix<double, 4, Eigen::Dynamic> test_mat(4, N*this->plan_data_.env_mat.cols());
  for (int i=0; i<N; ++i)
  {
    test_mat.block(0, i*this->plan_data_.env_mat.cols(), 4, this->plan_data_.env_mat.cols()) = this->plan_data_.env_mat;
  }

  auto eigen_t1 = chrono::high_resolution_clock::now();

  // ? faster in small-dim -> H * test_mat
  Eigen::Matrix<double, 5, Eigen::Dynamic> results(5, test_mat.cols());
  path_tools::fovMatMul(results, debug_H_, test_mat);
  bool hasNegative = path_tools::checkNegativeColumns(results);

  // ? faster in large-dim -> for loop check
//   bool hasNegative = false;
//   for (int i=0; i<(int)test_mat.cols(); ++i)
//   {
//     Eigen::Vector3d pt = test_mat.col(i).head(3);
//     if (this->percep_utils_->insideFOV(pt) == true)
//     {
//         hasNegative = true;
//     }
//   }

  auto eigen_t2 = chrono::high_resolution_clock::now();
  chrono::duration<double, milli> eigen_ms = eigen_t2 - eigen_t1;
  double eigen_time = (double)eigen_ms.count();
  ROS_INFO("\033[35m[Replanning][Debug] eigen time -> %lf ms with %d env points. -> min dist : %lf \033[35m", eigen_time, (int)test_mat.cols(), (double)hasNegative);

  Eigen::VectorXd s_pt(5), e_pt(5);
  double start_pitch = -10.0 * M_PI / 180.0, end_pitch = 10.0 * M_PI / 180.0;
  double start_yaw = -10.0 * M_PI / 180.0, end_yaw = 30.0 * M_PI / 180.0;

  s_pt << 3.0, 6.0, 5.0, start_pitch, start_yaw;
  e_pt << 3.0, 13.0, 5.0, end_pitch, end_yaw;

  auto astar_t1 = chrono::high_resolution_clock::now();

  this->hd_astar_->setOcclusion(this->plan_data_.env_mat);
  this->hd_astar_->setSearchStep(this->search_step_);
  this->hd_astar_->setPathInterval(this->path_interval_);

  if (this->hd_astar_->casVisEqSearch(s_pt, e_pt))
  {
    this->output_path_.clear();
    this->output_path_ = this->hd_astar_->getPath();
  }
  else
  {
    ROS_INFO("\033[35m[Replanning][HD-Astar] high-dim astar search FAILED. \033[35m");
  }

  this->waypts_indi_.clear();
  this->waypts_indi_.push_back(false);
  int output_size = (int)this->output_path_.size();
  if (output_size > 2)
  {
    for (int i=0; i<output_size-2; ++i)
    this->waypts_indi_.push_back(true);
  }
  this->waypts_indi_.push_back(false);

  auto astar_t2 = chrono::high_resolution_clock::now();
  chrono::duration<double, milli> astar_ms = astar_t2 - astar_t1;
  double astar_time = (double)astar_ms.count();
  ROS_INFO("\033[35m[Replanning][HD-Astar] high-dim astar time -> %lf ms. \033[35m", astar_time);

  return;
}

} // namespace fc_vision
