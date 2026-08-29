/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the header file of PlanningVisualization class, which 
 *                   implements the visualization tools for FC-Planner.
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

#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include "quadrotor_msgs/PolynomialTraj.h"
#include "gcopter/quickhull.hpp"
#include "gcopter/geo_utils.hpp"
#include <cstdlib>
#include <ctime>
#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <unordered_map>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using std::vector, std::map, std::unordered_map;
namespace fc_vision {

class PlanningVisualization {
private:

  /* data */
  visualization_msgs::MarkerArray path_markers_;
  visualization_msgs::MarkerArray vp_set_;

  visualization_msgs::MarkerArray path_markers_updated_;
  visualization_msgs::MarkerArray vp_set_updated_;
  /* visib_pub is seperated from previous ones for different info */
  ros::NodeHandle node;
  // Local Planner
  ros::Publisher local_pub_;      
  ros::Publisher localVP_pub_;  
  ros::Publisher local_path_order_pub_;  
  ros::Publisher localTarget_pub_;
  ros::Publisher localTar_pub_;
  ros::Publisher localEnv_pub_;
  ros::Publisher localBox_pub_;     
  ros::Publisher localVA_pub_;    
  ros::Publisher localVAct_pub_; 
  ros::Publisher worldsample_pub_;
  ros::Publisher localsample_pub_;
  ros::Publisher localAABB_pub_;
  ros::Publisher localAABBObstacle_pub_;
  ros::Publisher localInitVisST_pub_;
  ros::Publisher local_updated_pub_;
  ros::Publisher localVP_updated_pub_;
  ros::Publisher local_updated_path_order_pub_; 
  ros::Publisher local_used_g_path_pub_;
  ros::Publisher local_used_g_prior_pub_;
  ros::Publisher local_used_g_nex_sub_inliers_pub_;
  ros::Publisher local_used_g_next_sub_vps_pub_;
  // Local Planner Debug
  ros::Publisher local_updated_VP_frame_pub_;
  // ROSA
  ros::Publisher pcloud_pub_; 
  ros::Publisher mesh_pub_; 
  ros::Publisher normal_pub_;
  ros::Publisher rosa_orientation_pub_;
  ros::Publisher drosa_pub_;
  ros::Publisher le_pts_pub_;
  ros::Publisher le_lines_pub_;
  ros::Publisher rr_pts_pub_;
  ros::Publisher rr_lines_pub_;
  ros::Publisher decomp_pub_;
  ros::Publisher branch_start_end_pub_;
  ros::Publisher branch_dir_pub_;
  ros::Publisher cut_plane_pub_;
  ros::Publisher cut_pt_pub_;
  ros::Publisher sub_space_pub_;
  ros::Publisher sub_endpts_pub_;
  ros::Publisher vertex_ID_pub_;
  // ROSA Intermediate Vis
  ros::Publisher checkPoint_pub_;
  ros::Publisher checkNeigh_pub_;
  ros::Publisher checkCPdir_pub_;
  ros::Publisher checkRP_pub_;
  ros::Publisher checkCPpts_pub_;
  ros::Publisher checkCPptsCluster_pub_;
  ros::Publisher checkBranch_pub_;
  ros::Publisher checkAdj_pub_;
  ros::Publisher inliers_pub_;
  ros::Publisher inliers_intersection_pub_;
  ros::Publisher inliers_isec_pub_;
  ros::Publisher vis_graph_pub_;
  ros::Publisher cvx_cluster_pub_;
  ros::Publisher intra_edge_pub_;
  // ROSA Opt
  ros::Publisher optArea_pub_;
  // Global Planner
  ros::Publisher init_vps_pub_;
  ros::Publisher sub_vps_hull_pub_;
  ros::Publisher before_opt_vp_pub_;
  ros::Publisher after_opt_vp_pub_;
  ros::Publisher hcopp_viewpoints_pub_;
  ros::Publisher hcopp_occ_pub_;
  ros::Publisher hcopp_internal_pub_;
  ros::Publisher hcopp_fov_pub_;
  ros::Publisher hcopp_uncovered_pub_;
  ros::Publisher hcopp_global_uncovered_pub_;
  ros::Publisher hcopp_validvp_pub_;
  ros::Publisher hcopp_correctnormal_pub_;
  ros::Publisher hcopp_sub_finalvps_pub_;
  ros::Publisher hcopp_vps_drone_pub_;
  ros::Publisher hcopp_globalseq_pub_;
  ros::Publisher hcopp_globalboundary_pub_;
  ros::Publisher hcopp_local_path_pub_;
  ros::Publisher hcopp_full_path_pub_;
  ros::Publisher hcopp_full_waypts_pub_;
  ros::Publisher fullatsp_full_path_pub_;
  ros::Publisher fullgdcpca_full_path_pub_;
  ros::Publisher pca_vec_pub_;
  ros::Publisher cylinder_pub_;
  ros::Publisher posi_traj_pub_, pitch_traj_pub_, yaw_traj_pub_;
  ros::Publisher jointSphere_pub_;
  ros::Publisher hcoppYaw_pub_;
  ros::Publisher pathVisible_pub_;
  ros::Publisher global_start_pub_;
  ros::Publisher before_consistency_path_pub_;
  ros::Publisher global_next_sub_consistency_pub_;
  ros::Publisher global_vp_vis_graph_pub_;
  ros::Publisher global_vp_decomp_pub_;
  ros::Publisher global_decomp_path_pub_;
  // Updates Visualization
  ros::Publisher currentPose_pub_;
  ros::Publisher currentVoxels_pub_;
  // Flight
  ros::Publisher drawFoV_pub_;
  ros::Publisher drone_pub_;
  ros::Publisher traveltraj_pub_;
  nav_msgs::Path path_msg;
  ros::Publisher visible_pub_;
  ros::Publisher exec_path_pub_;
  std::string droneMesh;
  // Prediction
  ros::Publisher pred_mesh_pub_;
  // Viewpoints
  ros::Publisher vp_box_pub_;
  ros::Publisher fov_H_mesh_pub_;
  ros::Publisher fov_H_edge_pub_;
  ros::Publisher sample_region_pub_;
  // Registration
  ros::Publisher gicp_target_pub_;
  ros::Publisher gicp_source_pub_;
  ros::Publisher gicp_aligned_cloud_pub_;
  ros::Publisher gicp_aligned_mesh_pub_;
  // Replanning
  ros::Publisher replan_full_path_pub_;
  ros::Publisher replan_full_waypts_pub_;

public:
  PlanningVisualization(/* args */) {
  }
  ~PlanningVisualization() {
  }
  PlanningVisualization(ros::NodeHandle& nh);

  /* VISUALIZATION */
  void publishSurface(const pcl::PointCloud<pcl::PointXYZ>& input_cloud);
  void publishVisCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);
  void publishMesh(std::string& mesh);
  void publishSurfaceNormal(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, const pcl::PointCloud<pcl::Normal>& normals);
  void publishROSAOrientation(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, const pcl::PointCloud<pcl::Normal>& normals);
  void publish_dROSA(const pcl::PointCloud<pcl::PointXYZ>& input_cloud);
  void publish_lineextract_vis(Eigen::MatrixXd& skelver, Eigen::MatrixXi& skeladj);
  void publish_recenter_vis(Eigen::MatrixXd& skelver, Eigen::MatrixXi& skeladj, Eigen::MatrixXd& realVertices);
  void publish_decomposition(Eigen::MatrixXd& nodes, vector<vector<int>>& branches, vector<Eigen::Vector3d>& dirs, vector<Eigen::Vector3d>& centroids);
  void publishInliers(const pcl::PointCloud<pcl::PointXYZ>& inliers, const pcl::PointCloud<pcl::PointXYZ>& all_pts, const Eigen::MatrixXf& directions, const vector<vector<Eigen::Vector3d>>& isec_pts);
  void publishVisGraph(const vector<Eigen::Vector3d>& inliers, const Eigen::MatrixXi& vis_graph, const vector<vector<int>>& cvx_sets);
  void publishIntraEdge(const vector<Eigen::MatrixX3d>& intra_edges);
  void publishCutPlane(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, Eigen::Vector3d& p, Eigen::Vector3d& v);
  void publishSubSpace(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& space);
  void publishSubEndpts(map<int, vector<Eigen::Vector3d>>& endpts);
  void publishSegViewpoints(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& seg_vps);
  void publishOccupied(pcl::PointCloud<pcl::PointXYZ>& occupied);
  void publishInternal(pcl::PointCloud<pcl::PointXYZ>& internal);
  void publishFOV(const vector<vector<Eigen::Vector3d>>& list1, const vector<vector<Eigen::Vector3d>>& list2);
  void publishUncovered(pcl::PointCloud<pcl::PointXYZ>& uncovered);
  void publishGlobalUncovered(pcl::PointCloud<pcl::PointXYZ>& uncovered);
  void publishRevisedNormal(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, const pcl::PointCloud<pcl::Normal>& normals);
  void publishFinalFOV(map<int, vector<vector<Eigen::Vector3d>>>& list1, map<int, vector<vector<Eigen::Vector3d>>>& list2, map<int, vector<double>>& yaws);
  void publishGlobalSeq(Eigen::Vector3d& start_, vector<Eigen::Vector3d>& sub_rep, vector<int>& global_seq);
  void publishGlobalBoundary(Eigen::Vector3d& start_, map<int, vector<int>>& boundary_id_, map<int, vector<Eigen::VectorXd>>& sub_vps, vector<int>& global_seq);
  void publishLocalPath(map<int, vector<Eigen::VectorXd>>& sub_paths_);
  void publishHCOPPPath(vector<Eigen::VectorXd>& fullpath_);
  void publishFullATSPPath(vector<Eigen::VectorXd>& fullpath_);
  void publishFullGDCPCAPath(vector<Eigen::VectorXd>& fullpath_);
  void publishPCAVec(vector<Eigen::Vector3d>& sub_center, map<int, Eigen::Matrix3d>& sub_pcavec);
  void publishVPOpt(pcl::PointCloud<pcl::PointNormal>::Ptr& before_, pcl::PointCloud<pcl::PointNormal>::Ptr& after_);
  void publishFitCylinder(map<int, vector<double>>& cylinder_param);
  void publishHCOPPTraj(quadrotor_msgs::PolynomialTraj& posi, quadrotor_msgs::PolynomialTraj& pitch, quadrotor_msgs::PolynomialTraj& yaw);
  void publishJointSphere(vector<Eigen::Vector3d>& joints, double& radius, vector<vector<Eigen::Vector3d>>& InnerVps);
  void publishYawTraj(vector<Eigen::Vector3d>& waypt, vector<double>& yaw);
  void publishCurrentFoV(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2, const double& yaw);
  void publishTravelTraj(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color, int id);
  void publishVisiblePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& currentCloud, int id);
  void publishVpsCHull(std::map<int, std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>>& vpHull, vector<Eigen::Vector3d>& hamiPath);
  void publishInitVps(pcl::PointCloud<pcl::PointNormal>::Ptr& init_vps);
  void publishGlobalStart(const Eigen::VectorXd& start_);
  void publishExecPart(const vector<Eigen::VectorXd>& path);
  void publishBeforeConsistencyPath(const vector<Eigen::VectorXd>& path);
  void publishGlobalNextSubConsistency(const vector<Eigen::Vector3d>& last, const vector<Eigen::Vector3d>& current);
  void publishGlobalVPVisGraph(const vector<Eigen::VectorXd>& vps, const Eigen::MatrixXi& vp_vis_graph, const vector<vector<int>>& decomp_vps);
  void publishDecompGlobalPath(const vector<Eigen::Vector3d>& global_path);
  /* ROSA Debug Vis */
  void publishCheckNeigh(Eigen::Vector3d& checkPoint, const pcl::PointCloud<pcl::PointXYZ>& checkNeigh, Eigen::MatrixXi& edgeMat);
  void publishCheckCP(Eigen::Vector3d& CPPoint, Eigen::Vector3d& CPDir, Eigen::Vector3d& checkRP, const pcl::PointCloud<pcl::PointXYZ>& CPPts, const pcl::PointCloud<pcl::PointXYZ>& CPPtsCluster);
  /* Updates Visualization */
  void publishUpdatesPose(pcl::PointCloud<pcl::PointXYZ>& visCloud, vector<vector<Eigen::Vector3d>>& list1, vector<vector<Eigen::Vector3d>>& list2, vector<double>& yaws);
  /* ROSA Opt */
  void publishOptArea(vector<pcl::PointCloud<pcl::PointXYZ>>& optArea);
  /* Local Planning */
  void publishLocalRegion(pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& env_cloud, Eigen::Vector3d& min_bound, Eigen::Vector3d& max_bound);
  void publishCurPath(const vector<Eigen::VectorXd>& local_path, const vector<vector<Eigen::Vector3d>>& list1, const vector<vector<Eigen::Vector3d>>& list2);
  void publishLocalVST(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& input_actual);
  void publishSamples(pcl::PointCloud<pcl::PointXYZ>::Ptr& world_samples, pcl::PointCloud<pcl::PointXYZ>::Ptr& local_samples);
  void publishLocalAABB(const unordered_map<int, vector<Eigen::Vector3d>>& aabb, const unordered_map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>& aabb_obstacle);
  void publishLocalVisST(const vector<Eigen::VectorXd>& vp, const vector<Eigen::Vector3d>& vp_st);
  void publishUpdatedCurPath(const vector<Eigen::VectorXd>& local_path, const vector<vector<Eigen::Vector3d>>& list1, const vector<vector<Eigen::Vector3d>>& list2);
  void publishUpdatedVPFrame(const vector<Eigen::Vector3d>& vp_pos, const vector<Eigen::Matrix3d>& vp_frames);
  void publishLocalUsedGlobal(const vector<Eigen::VectorXd>& path, const vector<Eigen::VectorXd>& prior_path);
  void publishLocalUsedGlobalNextSub(const vector<Eigen::Vector3d>& inliers, const vector<Eigen::VectorXd>& vps);
  /* Prediction */
  void publishPredMesh(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& faces);
  /* Viewpoints */
  void publishVPBox(const Eigen::Vector3d& box_min, const Eigen::Vector3d& box_max);
  void publishFOVHRep(Eigen::Matrix<double, 5, 4>& hPolys);
  void publishSampleRegions(const vector<vector<double>>& sectors);
  /* Registration */
  void publishRegResults(const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& aligned_cloud, const Eigen::MatrixXd& aligned_mesh_V, const Eigen::MatrixXi& aligned_mesh_F);
  /* Replanning */
  void publishReplanGPath(vector<Eigen::VectorXd>& fullpath_);

private:
  vector<double> red_list;
  vector<double> green_list;
  vector<double> blue_list;
};
}
#endif