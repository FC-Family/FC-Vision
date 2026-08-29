/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the main algorithm of perception utils in FC-Planner.
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

#include "active_perception/perception_utils.h"

namespace fc_vision {
PerceptionUtils::PerceptionUtils(){
}

PerceptionUtils::~PerceptionUtils(){
}

void PerceptionUtils::init(ros::NodeHandle& nh)
{
  // * Params Initialization
  nh.param("perception_utils/top_angle", top_angle_, -1.0);
  nh.param("perception_utils/left_angle", left_angle_, -1.0);
  nh.param("perception_utils/right_angle", right_angle_, -1.0);

  nh.param("perception_utils/max_dist", max_dist_, -1.0);
  nh.param("perception_utils/vis_dist", vis_dist_, -1.0);
  nh.param("perception_utils/shrink_fac", shrink_factor_, -1.0);

  n_top_ << 0.0, sin(M_PI/2 - top_angle_), cos(M_PI/2 - top_angle_);
  n_bottom_ << 0.0, -sin(M_PI/2 - top_angle_), cos(M_PI/2 - top_angle_);

  n_left_ << sin(M_PI/2 - left_angle_), 0.0, cos(M_PI/2 - left_angle_);
  n_right_ << -sin(M_PI/2 - right_angle_), 0.0, cos(M_PI/2 - right_angle_);

  left_angle_shrink_ = this->shrink_factor_ * left_angle_;
  right_angle_shrink_ = this->shrink_factor_ * right_angle_;
  top_angle_shrink_ = this->shrink_factor_ * top_angle_;

  n_top_shrink_ << 0.0, sin(M_PI/2 - top_angle_shrink_), cos(M_PI/2 - top_angle_shrink_);
  n_bottom_shrink_ << 0.0, -sin(M_PI/2 - top_angle_shrink_), cos(M_PI/2 - top_angle_shrink_);

  n_left_shrink_ << sin(M_PI/2 - left_angle_shrink_), 0.0, cos(M_PI/2 - left_angle_shrink_);
  n_right_shrink_ << -sin(M_PI/2 - right_angle_shrink_), 0.0, cos(M_PI/2 - right_angle_shrink_);

  T_cb_ << 0, -1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1;
  T_bc_ = T_cb_.inverse();

  // * FOV vertices in body frame, for FOV visualization
  double hor = vis_dist_ * tan(left_angle_);
  double vert = vis_dist_ * tan(top_angle_);
  Eigen::Vector3d origin(0, 0, 0);
  Eigen::Vector3d left_up(vis_dist_, hor, vert);
  Eigen::Vector3d left_down(vis_dist_, hor, -vert);
  Eigen::Vector3d right_up(vis_dist_, -hor, vert);
  Eigen::Vector3d right_down(vis_dist_, -hor, -vert);

  cam_vertices1_.push_back(origin);
  cam_vertices2_.push_back(left_up);
  cam_vertices1_.push_back(origin);
  cam_vertices2_.push_back(left_down);
  cam_vertices1_.push_back(origin);
  cam_vertices2_.push_back(right_up);
  cam_vertices1_.push_back(origin);
  cam_vertices2_.push_back(right_down);

  cam_vertices1_.push_back(left_up);
  cam_vertices2_.push_back(right_up);
  cam_vertices1_.push_back(right_up);
  cam_vertices2_.push_back(right_down);
  cam_vertices1_.push_back(right_down);
  cam_vertices2_.push_back(left_down);
  cam_vertices1_.push_back(left_down);
  cam_vertices2_.push_back(left_up);

  // * Shrink FOV vertices in body frame, for Shrink FOV visualization
  double hor_shrink = vis_dist_ * tan(left_angle_shrink_);
  double vert_shrink = vis_dist_ * tan(top_angle_shrink_);
  Eigen::Vector3d left_up_shrink(vis_dist_, hor_shrink, vert_shrink);
  Eigen::Vector3d left_down_shrink(vis_dist_, hor_shrink, -vert_shrink);
  Eigen::Vector3d right_up_shrink(vis_dist_, -hor_shrink, vert_shrink);
  Eigen::Vector3d right_down_shrink(vis_dist_, -hor_shrink, -vert_shrink);

  cam_vertices1_shrink_.push_back(origin);
  cam_vertices2_shrink_.push_back(left_up_shrink);
  cam_vertices1_shrink_.push_back(origin);
  cam_vertices2_shrink_.push_back(left_down_shrink);
  cam_vertices1_shrink_.push_back(origin);
  cam_vertices2_shrink_.push_back(right_up_shrink);
  cam_vertices1_shrink_.push_back(origin);
  cam_vertices2_shrink_.push_back(right_down_shrink);
  cam_vertices1_shrink_.push_back(left_up_shrink);
  cam_vertices2_shrink_.push_back(right_up_shrink);
  cam_vertices1_shrink_.push_back(right_up_shrink);
  cam_vertices2_shrink_.push_back(right_down_shrink);
  cam_vertices1_shrink_.push_back(right_down_shrink);
  cam_vertices2_shrink_.push_back(left_down_shrink);
  cam_vertices1_shrink_.push_back(left_down_shrink);
  cam_vertices2_shrink_.push_back(left_up_shrink);

  double hor_actual = max_dist_ * tan(left_angle_shrink_);
  double vert_actual = max_dist_ * tan(top_angle_shrink_);
  Eigen::Vector3d left_up_actual(max_dist_, hor_actual, vert_actual);
  Eigen::Vector3d left_down_actual(max_dist_, hor_actual, -vert_actual);
  Eigen::Vector3d right_up_actual(max_dist_, -hor_actual, vert_actual);
  Eigen::Vector3d right_down_actual(max_dist_, -hor_actual, -vert_actual);
  cam_actual_.push_back(origin);
  cam_actual_.push_back(left_up_actual);
  cam_actual_.push_back(left_down_actual);
  cam_actual_.push_back(right_up_actual);
  cam_actual_.push_back(right_down_actual);
}

void PerceptionUtils::setShrinkFactor(const double& factor)
{
  this->shrink_factor_ = factor;

  return;
}

void PerceptionUtils::setPose_PY(const Eigen::Vector3d& pos, const double& pitch, const double& yaw)
{
  pos_ = pos;
  pitch_ = pitch;
  yaw_ = yaw;
  // Transform the normals of camera FOV
  Rwb_y << cos(yaw_), -sin(yaw_), 0.0, sin(yaw_), cos(yaw_), 0.0, 0.0, 0.0, 1.0;
  Rwb_p << cos(pitch_), 0.0, -sin(pitch_), 0.0, 1.0, 0.0, sin(pitch_), 0.0, cos(pitch_);
  Eigen::Vector3d pc = pos_;
  Eigen::Matrix4d T_wb = Eigen::Matrix4d::Identity();
  T_wb.block<3, 3>(0, 0) = Rwb_y * Rwb_p;
  T_wb.block<3, 1>(0, 3) = pc;
  Eigen::Matrix4d T_wc = T_wb * T_bc_;
  Eigen::Matrix3d R_wc = T_wc.block<3, 3>(0, 0);
  normals_ = { n_top_, n_bottom_, n_left_, n_right_ };
  for (auto& n : normals_)
  {
    n = R_wc * n;
  }

  shrink_normals_ = { n_top_shrink_, n_bottom_shrink_, n_left_shrink_, n_right_shrink_ };
  for (auto& n : shrink_normals_)
  {
    n = R_wc * n;
  }

  R_nr_ = Rwb_y * Rwb_p;

  return;
}

void PerceptionUtils::setPoseGimbal(const Eigen::Vector3d& pos, const double& pitch_gimbal, const double& yaw_gimbal)
{
  pos_ = pos;
  pitch_ = pitch_gimbal;
  yaw_ = yaw_gimbal;
  // Transform the normals of camera FOV
  Eigen::Matrix3d Rwb_y, Rwb_p;
  Rwb_y << cos(-yaw_), -sin(-yaw_), 0.0, sin(-yaw_), cos(-yaw_), 0.0, 0.0, 0.0, 1.0;
  Rwb_p << cos(pitch_), 0.0, -sin(pitch_), 0.0, 1.0, 0.0, sin(pitch_), 0.0, cos(pitch_);
  Eigen::Vector3d pc = pos_;
  Eigen::Matrix4d T_wb = Eigen::Matrix4d::Identity();
  T_wb.block<3, 3>(0, 0) = Rwb_y * Rwb_p;
  T_wb.block<3, 1>(0, 3) = pc;
  Eigen::Matrix4d T_wc = T_wb * T_bc_;
  Eigen::Matrix3d R_wc = T_wc.block<3, 3>(0, 0);
  normals_ = { n_top_, n_bottom_, n_left_, n_right_ };
  for (auto& n : normals_)
  {
    n = R_wc * n;
  }

  shrink_normals_ = { n_top_shrink_, n_bottom_shrink_, n_left_shrink_, n_right_shrink_ };
  for (auto& n : shrink_normals_)
  {
    n = R_wc * n;
  }

  return;
}

void PerceptionUtils::getFOV_PY(vector<Eigen::Vector3d>& list1, vector<Eigen::Vector3d>& list2)
{
  list1.clear();
  list2.clear();

  // Get info for visualizing FOV at (pos, yaw)
  Eigen::Matrix3d Rwb_y, Rwb_p;
  Rwb_y << cos(-yaw_), -sin(yaw_), 0.0, sin(yaw_), cos(yaw_), 0.0, 0.0, 0.0, 1.0;
  Rwb_p << cos(pitch_), 0.0, -sin(pitch_), 0.0, 1.0, 0.0, sin(pitch_), 0.0, cos(pitch_);
  for (int i = 0; i < (int)cam_vertices1_.size(); ++i) {
    auto p1 = Rwb_y * Rwb_p * cam_vertices1_[i] + pos_;
    auto p2 = Rwb_y * Rwb_p * cam_vertices2_[i] + pos_;
    list1.push_back(p1);
    list2.push_back(p2);
  }

  return;
}

void PerceptionUtils::getFOVShrink_PY(vector<Eigen::Vector3d>& list1, vector<Eigen::Vector3d>& list2)
{
  list1.clear();
  list2.clear();

  // Get info for visualizing Shrink FOV at (pos, yaw)
  Eigen::Matrix3d Rwb_y, Rwb_p;
  Rwb_y << cos(-yaw_), -sin(yaw_), 0.0, sin(yaw_), cos(yaw_), 0.0, 0.0, 0.0, 1.0;
  Rwb_p << cos(pitch_), 0.0, -sin(pitch_), 0.0, 1.0, 0.0, sin(pitch_), 0.0, cos(pitch_);
  for (int i = 0; i < (int)cam_vertices1_shrink_.size(); ++i) {
    auto p1 = Rwb_y * Rwb_p * cam_vertices1_shrink_[i] + pos_;
    auto p2 = Rwb_y * Rwb_p * cam_vertices2_shrink_[i] + pos_;
    list1.push_back(p1);
    list2.push_back(p2);
  }

  return;
}

void PerceptionUtils::getFOVGimbal(vector<Eigen::Vector3d>& list1, vector<Eigen::Vector3d>& list2)
{
  list1.clear();
  list2.clear();

  // Get info for visualizing FOV at (pos, yaw)
  Eigen::Matrix3d Rwb_y, Rwb_p;
  Rwb_y << cos(-yaw_), -sin(-yaw_), 0.0, sin(-yaw_), cos(-yaw_), 0.0, 0.0, 0.0, 1.0;
  Rwb_p << cos(pitch_), 0.0, -sin(pitch_), 0.0, 1.0, 0.0, sin(pitch_), 0.0, cos(pitch_);
  for (int i = 0; i < (int)cam_vertices1_.size(); ++i) {
    auto p1 = Rwb_y * Rwb_p * cam_vertices1_[i] + pos_;
    auto p2 = Rwb_y * Rwb_p * cam_vertices2_[i] + pos_;
    list1.push_back(p1);
    list2.push_back(p2);
  }

  return;
}

void PerceptionUtils::getFOVShrinkAABB_PY(Eigen::Vector3d& min_pt, Eigen::Vector3d& max_pt)
{
  Eigen::Matrix3d Rwb_y, Rwb_p;
  Rwb_y << cos(-yaw_), -sin(-yaw_), 0.0, sin(-yaw_), cos(-yaw_), 0.0, 0.0, 0.0, 1.0;
  Rwb_p << cos(pitch_), 0.0, -sin(pitch_), 0.0, 1.0, 0.0, sin(pitch_), 0.0, cos(pitch_);
  
  min_pt = Eigen::Vector3d::Constant( std::numeric_limits<double>::infinity());
  max_pt = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());

  for (int k = 0; k <  5; ++k)
  {
    const Eigen::Vector3d p = Rwb_y * Rwb_p * cam_actual_[k] + pos_;
    min_pt = min_pt.cwiseMin(p);
    max_pt = max_pt.cwiseMax(p);
  }

  return;
}

void PerceptionUtils::getFOVAABB(Eigen::Vector3d& min_pt, Eigen::Vector3d& max_pt, double& range)
{
  double box = range > this->max_dist_ ? range : this->max_dist_;
  min_pt = pos_ - Eigen::Vector3d(box, box, box);
  max_pt = pos_ + Eigen::Vector3d(box, box, box);

  return;
}

bool PerceptionUtils::insideFOV(const Eigen::Vector3d& point) 
{
  Eigen::Vector3d dir = point - pos_;
  if (dir.norm() > max_dist_) return false;

  dir.normalize();
  for (auto n : normals_) {
    if (dir.dot(n) < 0.0) return false;
  }
  return true;
}

bool PerceptionUtils::insideShrinkFOV(const Eigen::Vector3d& point)
{
  Eigen::Vector3d dir = point - pos_;
  if (dir.norm() > max_dist_) return false;

  dir.normalize();
  for (auto n : this->shrink_normals_) 
  {
    if (dir.dot(n) < 0.0) return false;
  }
  return true;
}

bool PerceptionUtils::insideFOVInflate(const Eigen::Vector3d& point, const double& inflate_dist) 
{
  Eigen::Vector3d dir = point - pos_;
  if (dir.norm() > (max_dist_+inflate_dist)) return false;

  dir.normalize();
  for (auto n : normals_) {
    if (dir.dot(n) < 0.0) return false;
  }
  return true;
}

void PerceptionUtils::getCamMat(vector<Eigen::Vector3d>& cv1, vector<Eigen::Vector3d>& cv2, const double& dist)
{
  double hor = dist * tan(left_angle_);
  double vert = dist * tan(top_angle_);

  Eigen::Vector3d origin(0, 0, 0);
  Eigen::Vector3d left_up(dist, hor, vert);
  Eigen::Vector3d left_down(dist, hor, -vert);
  Eigen::Vector3d right_up(dist, -hor, vert);
  Eigen::Vector3d right_down(dist, -hor, -vert);

  cv1.push_back(origin);
  cv2.push_back(left_up);
  cv1.push_back(origin);
  cv2.push_back(left_down);
  cv1.push_back(origin);
  cv2.push_back(right_up);
  cv1.push_back(origin);
  cv2.push_back(right_down);

  cv1.push_back(left_up);
  cv2.push_back(right_up);
  cv1.push_back(right_up);
  cv2.push_back(right_down);
  cv1.push_back(right_down);
  cv2.push_back(left_down);
  cv1.push_back(left_down);
  cv2.push_back(left_up); 

  return;
}

void PerceptionUtils::getCamMatShrink(vector<Eigen::Vector3d>& cv1, vector<Eigen::Vector3d>& cv2, const double& dist)
{
  double hor = dist * tan(left_angle_shrink_);
  double vert = dist * tan(top_angle_shrink_);

  Eigen::Vector3d origin(0, 0, 0);
  Eigen::Vector3d left_up(dist, hor, vert);
  Eigen::Vector3d left_down(dist, hor, -vert);
  Eigen::Vector3d right_up(dist, -hor, vert);
  Eigen::Vector3d right_down(dist, -hor, -vert);

  cv1.push_back(origin);
  cv2.push_back(left_up);
  cv1.push_back(origin);
  cv2.push_back(left_down);
  cv1.push_back(origin);
  cv2.push_back(right_up);
  cv1.push_back(origin);
  cv2.push_back(right_down);

  cv1.push_back(left_up);
  cv2.push_back(right_up);
  cv1.push_back(right_up);
  cv2.push_back(right_down);
  cv1.push_back(right_down);
  cv2.push_back(left_down);
  cv1.push_back(left_down);
  cv2.push_back(left_up); 

  return;
}

void PerceptionUtils::getHRepresentationFov(Eigen::Matrix<double, 5, 4>& H, const double& max_dist, bool shrink)
{
  double h_dist = max_dist;

  vector<Eigen::Vector3d> cv1, cv2;
  if (shrink) getCamMatShrink(cv1, cv2, h_dist);
  else getCamMat(cv1, cv2, h_dist);

  // * points: {origin, left_up, right_up, right_down, left_down}
  vector<Eigen::Vector3d> points;
  for (int i = 3; i < (int)cv1.size(); ++i) 
  {
    auto p1 = Rwb_y * Rwb_p * cv1[i] + pos_;
    points.push_back(p1);
  }
  
  // * get vertices of 5 planes -> {top, bottom, left, right, far}
  vector<Eigen::Vector3d> top = { points[0], points[2], points[1] }; // origin, right_up, left_up
  vector<Eigen::Vector3d> bottom = { points[0], points[4], points[3] }; // origin, left_down, right_down
  vector<Eigen::Vector3d> left = { points[0], points[1], points[4] }; // origin, left_up, left_down
  vector<Eigen::Vector3d> right = { points[0], points[3], points[2] }; // origin, right_down, right_up
  vector<Eigen::Vector3d> far = { points[1], points[2], points[3], points[4] }; // left_up, right_up, right_down, left_down

  vector<vector<Eigen::Vector3d>> vertices = { top, bottom, left, right, far };

  // * get H representation of 5 planes
  // Each row of hPoly is defined by h0, h1, h2, h3 as
  // h0*x + h1*y + h2*z + h3 <= 0 -> (h0, h1, h2) unit normal, h3 distance to origin
  for (int i = 0; i < 5; ++i) 
  {
    Eigen::Vector3d n = (vertices[i][1] - vertices[i][0]).cross(vertices[i][2] - vertices[i][0]).normalized();
    H.row(i) << n(0), n(1), n(2), -n.dot(vertices[i][0]);
  }
}

void PerceptionUtils::preComputeLocalH()
{ 
  double hor = max_dist_ * tan(left_angle_);
  double vert = max_dist_ * tan(top_angle_);
  double hor_shrink = max_dist_ * tan(left_angle_shrink_);
  double vert_shrink = max_dist_ * tan(top_angle_shrink_);

  local_points_ = 
  {
    Eigen::Vector3d(0, 0, 0),                 // origin
    Eigen::Vector3d(max_dist_, -hor, vert),        // right_up
    Eigen::Vector3d(max_dist_, hor, vert),         // left_up
    Eigen::Vector3d(max_dist_, -hor, -vert),       // right_down
    Eigen::Vector3d(max_dist_, hor, -vert)         // left_down
  };

  local_points_shrink_ = 
  {
    Eigen::Vector3d(0, 0, 0),                               // origin
    Eigen::Vector3d(max_dist_, -hor_shrink, vert_shrink),        // right_up
    Eigen::Vector3d(max_dist_, hor_shrink, vert_shrink),         // left_up
    Eigen::Vector3d(max_dist_, -hor_shrink, -vert_shrink),       // right_down
    Eigen::Vector3d(max_dist_, hor_shrink, -vert_shrink)         // left_down
  };

  plane_normals_.resize(5);
  plane_normals_[0] = (local_points_[1] - local_points_[0]).cross(local_points_[2] - local_points_[0]).normalized(); // top
  plane_normals_[1] = (local_points_[4] - local_points_[0]).cross(local_points_[3] - local_points_[0]).normalized(); // bottom
  plane_normals_[2] = (local_points_[2] - local_points_[0]).cross(local_points_[4] - local_points_[0]).normalized(); // left
  plane_normals_[3] = (local_points_[3] - local_points_[0]).cross(local_points_[1] - local_points_[0]).normalized(); // right
  plane_normals_[4] = (local_points_[3] - local_points_[1]).cross(local_points_[2] - local_points_[1]).normalized(); // far

  plane_normals_shrink_.resize(5);
  plane_normals_shrink_[0] = (local_points_shrink_[1] - local_points_shrink_[0]).cross(local_points_shrink_[2] - local_points_shrink_[0]).normalized(); // top
  plane_normals_shrink_[1] = (local_points_shrink_[4] - local_points_shrink_[0]).cross(local_points_shrink_[3] - local_points_shrink_[0]).normalized(); // bottom
  plane_normals_shrink_[2] = (local_points_shrink_[2] - local_points_shrink_[0]).cross(local_points_shrink_[4] - local_points_shrink_[0]).normalized(); // left
  plane_normals_shrink_[3] = (local_points_shrink_[3] - local_points_shrink_[0]).cross(local_points_shrink_[1] - local_points_shrink_[0]).normalized(); // right
  plane_normals_shrink_[4] = (local_points_shrink_[3] - local_points_shrink_[1]).cross(local_points_shrink_[2] - local_points_shrink_[1]).normalized(); // far

  return;
}

void PerceptionUtils::getHRepFov(Eigen::Matrix<double, 5, 4>& H, const double& max_dist, bool shrink)
{
  double scale = max_dist / max_dist_;
  R_pt_ = R_nr_ * scale;

  vector<Eigen::Vector3d> l_pts = shrink ? local_points_shrink_ : local_points_;
  vector<Eigen::Vector3d> l_plane_nrs = shrink ? plane_normals_shrink_ : plane_normals_;
  vector<Eigen::Vector3d> world_points(l_pts.size());

  for (size_t i = 0; i < l_pts.size(); ++i)
    world_points[i] = R_pt_ * l_pts[i] + pos_;

  vector<Eigen::Vector3d> dot_pt = {world_points[0], world_points[0], world_points[0], world_points[0], world_points[1]};
  double h3;
  for (int i = 0; i < 5; ++i) 
  {
    Eigen::Vector3d world_normal = R_nr_ * l_plane_nrs[i];
    h3 = -world_normal.dot(dot_pt[i]);
    H.row(i) << world_normal(0), world_normal(1), world_normal(2), h3;
  }

  return;
}

}  // namespace fc_vision