/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the header file of PerceptionUtils class, which 
 *                   implements perception utils for coverage in FC-Planner.
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

#ifndef _PERCEPTION_UTILS_H_
#define _PERCEPTION_UTILS_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <memory>
#include <vector>

using namespace std;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace fc_vision {
class PerceptionUtils {
public:
  PerceptionUtils();
  ~PerceptionUtils();

  void init(ros::NodeHandle& nh);
  void setShrinkFactor(const double& factor);
  void setPose_PY(const Eigen::Vector3d& pos, const double& pitch, const double& yaw);
  void setPoseGimbal(const Eigen::Vector3d& pos, const double& pitch_gimbal, const double& yaw_gimbal);
  void getFOV_PY(vector<Eigen::Vector3d>& list1, vector<Eigen::Vector3d>& list2);
  void getFOVShrink_PY(vector<Eigen::Vector3d>& list1, vector<Eigen::Vector3d>& list2);
  void getFOVGimbal(vector<Eigen::Vector3d>& list1, vector<Eigen::Vector3d>& list2);
  void getFOVShrinkAABB_PY(Eigen::Vector3d& min_pt, Eigen::Vector3d& max_pt);
  void getFOVAABB(Eigen::Vector3d& min_pt, Eigen::Vector3d& max_pt, double& range);
  bool insideFOV(const Eigen::Vector3d& point);
  bool insideShrinkFOV(const Eigen::Vector3d& point);
  bool insideFOVInflate(const Eigen::Vector3d& point, const double& inflate_dist);
  void getCamMat(vector<Eigen::Vector3d>& cv1, vector<Eigen::Vector3d>& cv2, const double& dist);
  void getCamMatShrink(vector<Eigen::Vector3d>& cv1, vector<Eigen::Vector3d>& cv2, const double& dist);
  
  /* Frustum-related */ 
  void getHRepresentationFov(Eigen::Matrix<double, 5, 4>& H, const double& max_dist, bool shrink);
  void preComputeLocalH();
  void getHRepFov(Eigen::Matrix<double, 5, 4>& H, const double& max_dist, bool shrink);

  /* Param */
  double max_dist_;

private:
  // Data
  // Current camera pos and yaw
  Eigen::Vector3d pos_;
  double pitch_, yaw_;
  Eigen::Matrix3d Rwb_y, Rwb_p;
  Eigen::Matrix3d R_nr_, R_pt_;
  // Camera plane's normals in world frame
  vector<Eigen::Vector3d> normals_, shrink_normals_;

  /* Params */
  // Sensing range of camera
  double left_angle_, right_angle_, top_angle_, vis_dist_;
  double shrink_factor_ = 1.0;
  double left_angle_shrink_, right_angle_shrink_, top_angle_shrink_;
  // Normal vectors of camera FOV planes in camera frame
  Eigen::Vector3d n_top_, n_bottom_, n_left_, n_right_;
  Eigen::Vector3d n_top_shrink_, n_bottom_shrink_, n_left_shrink_, n_right_shrink_;
  // Transform between camera and body
  Eigen::Matrix4d T_cb_, T_bc_;
  // FOV vertices in body frame
  vector<Eigen::Vector3d> cam_vertices1_, cam_vertices2_;
  // Shrink FOV vertices in body frame
  vector<Eigen::Vector3d> cam_vertices1_shrink_, cam_vertices2_shrink_;
  vector<Eigen::Vector3d> cam_actual_;
  // H-plane & points in body frame
  vector<Eigen::Vector3d> local_points_, local_points_shrink_;
  vector<Eigen::Vector3d> plane_normals_, plane_normals_shrink_;
};

} // namespace fc_vision
#endif