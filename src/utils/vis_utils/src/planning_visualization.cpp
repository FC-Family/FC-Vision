/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the main functions of visualization tools
 *                   for FC-Planner. 
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

#include <vis_utils/planning_visualization.h> 

using std::cout;
using std::endl;
namespace fc_vision {
PlanningVisualization::PlanningVisualization(ros::NodeHandle& nh) {
  node = nh;

  /* ROSA */
  pcloud_pub_ = node.advertise<sensor_msgs::PointCloud2>("/rosa_vis/input_cloud", 10);
  mesh_pub_ = node.advertise<visualization_msgs::Marker>("/rosa_vis/input_mesh", 1);
  normal_pub_ = node.advertise<visualization_msgs::MarkerArray>("/rosa_vis/input_normal", 10);
  rosa_orientation_pub_ = node.advertise<visualization_msgs::MarkerArray>("/rosa_vis/rosa_orientation", 10);
  drosa_pub_ = node.advertise<sensor_msgs::PointCloud2>("/rosa_vis/drosa_pts", 10);
  le_pts_pub_ = node.advertise<sensor_msgs::PointCloud2>("/rosa_vis/le_pts", 10);
  le_lines_pub_ = node.advertise<visualization_msgs::Marker>("/rosa_vis/le_lines", 10);
  rr_pts_pub_ = node.advertise<sensor_msgs::PointCloud2>("/rosa_vis/rr_pts", 10);
  rr_lines_pub_ = node.advertise<visualization_msgs::Marker>("/rosa_vis/rr_lines", 10);
  decomp_pub_ = node.advertise<visualization_msgs::MarkerArray>("/rosa_vis/branches", 10);
  branch_start_end_pub_ = node.advertise<visualization_msgs::MarkerArray>("/rosa_vis/branches_start_end", 10);
  branch_dir_pub_ = node.advertise<visualization_msgs::MarkerArray>("/rosa_vis/branches_dir", 10);
  cut_plane_pub_ = node.advertise<sensor_msgs::PointCloud2>("/rosa_vis/cut_plane", 10);
  cut_pt_pub_ = node.advertise<visualization_msgs::Marker>("/rosa_vis/cut_pt", 10);
  sub_space_pub_ = node.advertise<sensor_msgs::PointCloud2>("/rosa_vis/sub_space", 1);
  sub_endpts_pub_ = node.advertise<visualization_msgs::MarkerArray>("/rosa_vis/sub_endpts", 1);
  vertex_ID_pub_ = node.advertise<visualization_msgs::MarkerArray>("/rosa_vis/vertex_ID", 1);

  /* ROSA Debug Vis */
  checkPoint_pub_ = node.advertise<visualization_msgs::Marker>("/rosa_debug/checkPoint", 1);
  checkNeigh_pub_ = node.advertise<sensor_msgs::PointCloud2>("/rosa_debug/checkNeigh", 1);
  checkCPdir_pub_ = node.advertise<visualization_msgs::Marker>("/rosa_debug/CPDirection", 1);
  checkRP_pub_ = node.advertise<visualization_msgs::Marker>("/rosa_debug/checkRP", 1);
  checkCPpts_pub_ = node.advertise<sensor_msgs::PointCloud2>("/rosa_debug/CPPoints", 1);
  checkCPptsCluster_pub_ = node.advertise<sensor_msgs::PointCloud2>("/rosa_debug/CPPointsCluster", 1);
  checkBranch_pub_ = node.advertise<visualization_msgs::MarkerArray>("/rosa_debug/checkBranches", 10);
  checkAdj_pub_ = node.advertise<visualization_msgs::Marker>("/rosa_debug/adj", 1);
  inliers_pub_ = node.advertise<sensor_msgs::PointCloud2>("/rosa_debug/inliers", 1);
  inliers_intersection_pub_ = node.advertise<visualization_msgs::MarkerArray>("/rosa_debug/inliers_intersec", 1);
  inliers_isec_pub_ = node.advertise<sensor_msgs::PointCloud2>("/rosa_debug/isec", 1);
  vis_graph_pub_ = node.advertise<visualization_msgs::Marker>("/rosa_debug/vis_graph", 1);
  cvx_cluster_pub_ = node.advertise<visualization_msgs::MarkerArray>("/rosa_debug/cvx_cluster", 10);
  intra_edge_pub_ = node.advertise<visualization_msgs::Marker>("/rosa_debug/intra_edges", 10);

  /* ROS Opt */
  optArea_pub_ = node.advertise<sensor_msgs::PointCloud2>("/rosa_opt/opt_area", 1);

  /* Global Planner */
  init_vps_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/init_vps", 1);
  sub_vps_hull_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/vps_hull", 1);
  before_opt_vp_pub_ = node.advertise<sensor_msgs::PointCloud2>("/hcopp/before_vps", 1);
  after_opt_vp_pub_ = node.advertise<sensor_msgs::PointCloud2>("/hcopp/after_vps", 1); 
  hcopp_viewpoints_pub_ = node.advertise<sensor_msgs::PointCloud2>("/hcopp/seg_viewpoints", 1);
  hcopp_occ_pub_ = node.advertise<sensor_msgs::PointCloud2>("/hcopp/occupied", 1);
  hcopp_internal_pub_ = node.advertise<sensor_msgs::PointCloud2>("/hcopp/internal", 1);
  hcopp_fov_pub_ = node.advertise<visualization_msgs::Marker>("/hcopp/fov_set", 1);
  hcopp_uncovered_pub_ = node.advertise<sensor_msgs::PointCloud2>("/hcopp/uncovered_area", 1);
  hcopp_global_uncovered_pub_ = node.advertise<sensor_msgs::PointCloud2>("/hcopp/global_uncovered_area", 1);
  hcopp_validvp_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/valid_vp", 1);
  hcopp_correctnormal_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/correctNormals", 10);
  hcopp_sub_finalvps_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/sub_finalvps", 1);
  hcopp_vps_drone_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/finalvps_drones", 1);
  hcopp_globalseq_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/global_seq", 1);
  hcopp_globalboundary_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/global_boundary", 1);
  hcopp_local_path_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/local_paths", 1);
  hcopp_full_path_pub_ = node.advertise<visualization_msgs::Marker>("/hcopp/HCOPP_Path", 1);
  hcopp_full_waypts_pub_ = node.advertise<visualization_msgs::Marker>("/hcopp/HCOPP_Waypts", 1);
  fullatsp_full_path_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/FullATSP_Path", 1);
  fullgdcpca_full_path_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/FullGDCPCA_Path", 1);
  pca_vec_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/PCA_Vec", 1);
  cylinder_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/fit_cylinder_", 1);
  posi_traj_pub_ = node.advertise<quadrotor_msgs::PolynomialTraj>("/fc_planner/position_traj", 1);
  pitch_traj_pub_ = node.advertise<quadrotor_msgs::PolynomialTraj>("/fc_planner/pitch_traj", 1);
  yaw_traj_pub_ = node.advertise<quadrotor_msgs::PolynomialTraj>("/fc_planner/yaw_traj", 1);
  jointSphere_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/JointSphere", 1);
  hcoppYaw_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/yaw_traj_", 1);
  pathVisible_pub_ = node.advertise<sensor_msgs::PointCloud2>("/hcopp/vis_path_cloud_", 1);
  global_start_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/global_start_", 1);
  before_consistency_path_pub_ = node.advertise<visualization_msgs::Marker>("/hcopp/before_consistency_path", 1);
  global_next_sub_consistency_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/global_next_sub_consistency", 1);
  global_vp_vis_graph_pub_ = node.advertise<visualization_msgs::Marker>("/hcopp/vps_vis_graph", 1);
  global_vp_decomp_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/vps_decomp", 1);
  global_decomp_path_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/decomp_global_path", 1);

  /* Updates Visualization */
  currentPose_pub_ = node.advertise<visualization_msgs::MarkerArray>("/SGVG/cur_vps_", 1);
  currentVoxels_pub_ = node.advertise<sensor_msgs::PointCloud2>("/SGVG/cur_vox_", 1);

  /* Flight */
  drawFoV_pub_ = node.advertise<visualization_msgs::Marker>("/fc_planner/cmd_fov", 10);
  drone_pub_ = node.advertise<visualization_msgs::Marker>("/fc_planner/drone", 10);
  traveltraj_pub_ = node.advertise<nav_msgs::Path>("/fc_planner/travel_traj", 1, true);
  visible_pub_ = node.advertise<sensor_msgs::PointCloud2>("/fc_planner/vis_points", 1);
  nh.param("hcopp/droneMesh", droneMesh, std::string("null"));
  exec_path_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/exec_part", 1);

  /* Local Planner */
  local_pub_ = node.advertise<visualization_msgs::MarkerArray>("/local_planning/local_tour", 10);
  localVP_pub_ = node.advertise<visualization_msgs::MarkerArray>("/local_planning/vp_local", 10);
  local_path_order_pub_ = node.advertise<visualization_msgs::MarkerArray>("/local_planning/local_tour_order", 1);
  localTar_pub_ = node.advertise<sensor_msgs::PointCloud2>("/local_planning/local_target", 10);
  localEnv_pub_ = node.advertise<sensor_msgs::PointCloud2>("/local_planning/local_env", 10);
  localBox_pub_ = node.advertise<visualization_msgs::Marker>("/local_planning/local_box", 10);
  localVA_pub_ = node.advertise<sensor_msgs::PointCloud2>("/local_planning/local_visibility_st", 10);
  localVAct_pub_ = node.advertise<sensor_msgs::PointCloud2>("/local_planning/local_visibility_actual", 10);
  worldsample_pub_ = node.advertise<sensor_msgs::PointCloud2>("/local_planning/world_samples", 10);
  localsample_pub_ = node.advertise<sensor_msgs::PointCloud2>("/local_planning/local_samples", 10);
  localAABB_pub_ = node.advertise<visualization_msgs::MarkerArray>("/local_planning/local_AABB", 10);
  localAABBObstacle_pub_ = node.advertise<sensor_msgs::PointCloud2>("/local_planning/local_AABB_obs", 10);
  localInitVisST_pub_ = node.advertise<visualization_msgs::MarkerArray>("/local_planning/local_init_visst", 10);
  local_updated_pub_ = node.advertise<visualization_msgs::MarkerArray>("/local_planning/local_updated_tour", 10);
  localVP_updated_pub_ = node.advertise<visualization_msgs::MarkerArray>("/local_planning/updated_vp_local", 10);
  local_updated_path_order_pub_ = node.advertise<visualization_msgs::MarkerArray>("/local_planning/local_updated_tour_order", 1);
  local_used_g_path_pub_ = node.advertise<visualization_msgs::MarkerArray>("/local_planning/local_used_global_path", 1);
  local_used_g_prior_pub_ = node.advertise<visualization_msgs::MarkerArray>("/local_planning/local_used_global_prior", 1);
  local_used_g_nex_sub_inliers_pub_ = node.advertise<visualization_msgs::Marker>("/local_planning/local_used_global_next_sub_inliers", 1);
  local_used_g_next_sub_vps_pub_ = node.advertise<visualization_msgs::MarkerArray>("/local_planning/local_used_global_next_sub_vps", 1);

  /* LocalPlanner Debug */
  local_updated_VP_frame_pub_ = node.advertise<visualization_msgs::MarkerArray>("/local_planning/updated_vp_frames", 10);

  /* Prediction */
  pred_mesh_pub_ = node.advertise<visualization_msgs::Marker>("/prediction/mesh", 1);

  /* Viewpoints */
  vp_box_pub_ = node.advertise<visualization_msgs::MarkerArray>("/viewpoints/viewpoints_box", 1);
  fov_H_mesh_pub_ = node.advertise<visualization_msgs::Marker>("/debug/fov_h/mesh", 1);
  fov_H_edge_pub_ = node.advertise<visualization_msgs::Marker>("/debug/fov_h/edge", 1);
  sample_region_pub_ = node.advertise<visualization_msgs::MarkerArray>("/debug/sample_region", 1);

  /* Registration */
  gicp_target_pub_ = node.advertise<sensor_msgs::PointCloud2>("/registration/target_cloud", 1);
  gicp_source_pub_ = node.advertise<sensor_msgs::PointCloud2>("/registration/source_cloud", 1);
  gicp_aligned_cloud_pub_ = node.advertise<sensor_msgs::PointCloud2>("/registration/aligned_cloud", 1);
  gicp_aligned_mesh_pub_ = node.advertise<visualization_msgs::Marker>("/registration/aligned_mesh", 1);

  /* Replanning */
  replan_full_path_pub_ = node.advertise<visualization_msgs::Marker>("/replan/cur_g_path", 1);
  replan_full_waypts_pub_ = node.advertise<visualization_msgs::Marker>("/replan/cur_g_waypts", 1);

  red_list.reserve(100);
  green_list.reserve(100);
  blue_list.reserve(100);
  for (int i = 0; i < 100; i++) 
  {
    double red = (double)rand() / (double)RAND_MAX;
    double green = (double)rand() / (double)RAND_MAX;
    double blue = (double)rand() / (double)RAND_MAX;
    red_list.push_back(red);
    green_list.push_back(green);
    blue_list.push_back(blue);
  }
}

void PlanningVisualization::publishSurface(const pcl::PointCloud<pcl::PointXYZ>& input_cloud)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_pred;
  cloud_pred = input_cloud;

  cloud_pred.width = cloud_pred.points.size();
  cloud_pred.height = 1;
  cloud_pred.is_dense = true;
  cloud_pred.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_pred, cloud_msg);
  pcloud_pub_.publish(cloud_msg);
}

void PlanningVisualization::publishVisCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_pred;
  for (auto p:input_cloud->points)
    cloud_pred.points.push_back(p);

  cloud_pred.width = cloud_pred.points.size();
  cloud_pred.height = 1;
  cloud_pred.is_dense = true;
  cloud_pred.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_pred, cloud_msg);
  pathVisible_pub_.publish(cloud_msg);
}

void PlanningVisualization::publishMesh(std::string& mesh)
{
  visualization_msgs::Marker meshModel;
  meshModel.header.frame_id = "world";
  meshModel.header.stamp = ros::Time::now();
  meshModel.id = 0;
  meshModel.ns = "mesh";
  meshModel.type = visualization_msgs::Marker::MESH_RESOURCE;
  meshModel.color.r = 128.0/256.0;
  meshModel.color.g = 128.0/256.0;
  meshModel.color.b = 128.0/256.0;
  meshModel.color.a = 1.0;
  meshModel.scale.x = 1.0;
  meshModel.scale.y = 1.0;
  meshModel.scale.z = 1.0;
  meshModel.pose.orientation.w = 1.0;
  meshModel.mesh_resource = "file://";
  meshModel.mesh_resource += mesh;
  meshModel.mesh_use_embedded_materials = true;

  mesh_pub_.publish(meshModel);
}

void PlanningVisualization::publishSurfaceNormal(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, const pcl::PointCloud<pcl::Normal>& normals)
{
  visualization_msgs::MarkerArray pcloud_normals;
  int counter = 0;
  double scale = 3.0;
  for (int i=0; i<(int)input_cloud.points.size(); ++i)
  {
    visualization_msgs::Marker nm;
    nm.header.frame_id = "world";
    nm.header.stamp = ros::Time::now();
    nm.id = counter;
    nm.type = visualization_msgs::Marker::ARROW;
    nm.action = visualization_msgs::Marker::ADD;

    nm.pose.orientation.w = 1.0;
    nm.scale.x = 0.2;
    nm.scale.y = 0.3;
    nm.scale.z = 0.2;

    geometry_msgs::Point pt_;
    pt_.x = input_cloud.points[i].x;
    pt_.y = input_cloud.points[i].y;
    pt_.z = input_cloud.points[i].z;
    nm.points.push_back(pt_);

    pt_.x = input_cloud.points[i].x + scale*normals.points[i].normal_x;
    pt_.y = input_cloud.points[i].y + scale*normals.points[i].normal_y;
    pt_.z = input_cloud.points[i].z + scale*normals.points[i].normal_z;
    nm.points.push_back(pt_);

    nm.color.r = 0.1;
    nm.color.g = 0.2;
    nm.color.b = 0.7;
    nm.color.a = 1.0;
    
    pcloud_normals.markers.push_back(nm);
    counter++;
  }

  normal_pub_.publish(pcloud_normals);
}

void PlanningVisualization::publishROSAOrientation(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, const pcl::PointCloud<pcl::Normal>& normals)
{
  visualization_msgs::MarkerArray pcloud_normals;
  int counter = 0;
  double scale = 3.0;
  for (int i=0; i<(int)input_cloud.points.size(); ++i)
  {
    visualization_msgs::Marker nm;
    nm.header.frame_id = "world";
    nm.header.stamp = ros::Time::now();
    nm.id = counter;
    nm.type = visualization_msgs::Marker::ARROW;
    nm.action = visualization_msgs::Marker::ADD;

    nm.pose.orientation.w = 1.0;
    nm.scale.x = 0.2;
    nm.scale.y = 0.3;
    nm.scale.z = 0.2;

    geometry_msgs::Point pt_;
    pt_.x = input_cloud.points[i].x;
    pt_.y = input_cloud.points[i].y;
    pt_.z = input_cloud.points[i].z;
    nm.points.push_back(pt_);

    pt_.x = input_cloud.points[i].x + scale*normals.points[i].normal_x;
    pt_.y = input_cloud.points[i].y + scale*normals.points[i].normal_y;
    pt_.z = input_cloud.points[i].z + scale*normals.points[i].normal_z;
    nm.points.push_back(pt_);

    nm.color.r = 0.2;
    nm.color.g = 0.7;
    nm.color.b = 0.1;
    nm.color.a = 1.0;
    
    pcloud_normals.markers.push_back(nm);
    counter++;
  }

  rosa_orientation_pub_.publish(pcloud_normals);
}

void PlanningVisualization::publish_dROSA(const pcl::PointCloud<pcl::PointXYZ>& local_region)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_pred;
  cloud_pred = local_region;

  cloud_pred.width = cloud_pred.points.size();
  cloud_pred.height = 1;
  cloud_pred.is_dense = true;
  cloud_pred.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_pred, cloud_msg);
  drosa_pub_.publish(cloud_msg);
}

void PlanningVisualization::publish_lineextract_vis(Eigen::MatrixXd& skelver, Eigen::MatrixXi& skeladj)
{
  pcl::PointCloud<pcl::PointXYZ> rosa_le_pts;
  pcl::PointXYZ pt;

  for (int i=0; i<skelver.rows(); ++i)
  {
    if (skelver(i,0) < -1e5+1) continue;
    pt.x = skelver(i,0); pt.y = skelver(i,1); pt.z = skelver(i,2);
    rosa_le_pts.points.push_back(pt);
  }

  rosa_le_pts.width = rosa_le_pts.points.size();
  rosa_le_pts.height = 1;
  rosa_le_pts.is_dense = true;
  rosa_le_pts.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(rosa_le_pts, cloud_msg);

  int counter_vpg = 0;

  visualization_msgs::Marker lines;
  lines.header.frame_id = "world";
  lines.header.stamp = ros::Time::now();
  lines.id = counter_vpg;
  lines.type = visualization_msgs::Marker::LINE_LIST;
  lines.action = visualization_msgs::Marker::ADD;

  lines.pose.orientation.w = 1.0;
  lines.scale.x = 0.01;

  lines.color.r = 0.2;
  lines.color.g = 0.8;
  lines.color.b = 0.4;
  lines.color.a = 1.0;
  
  geometry_msgs::Point p1, p2;
  for (int i=0; i<skeladj.rows(); ++i)
  {
    for (int j=0; j<skeladj.cols(); ++j)
    {
      if (skeladj(i,j) == 1)
      {
        p1.x = skelver(i,0); p1.y = skelver(i,1); p1.z = skelver(i,2);
        p2.x = skelver(j,0); p2.y = skelver(j,1); p2.z = skelver(j,2);
        lines.points.push_back(p1);
        lines.points.push_back(p2);
      }
    }
  }

  /* Publish */
  le_pts_pub_.publish(cloud_msg);
  le_lines_pub_.publish(lines);
}

void PlanningVisualization::publish_recenter_vis(Eigen::MatrixXd& skelver, Eigen::MatrixXi& skeladj, Eigen::MatrixXd& realVertices)
{
  pcl::PointCloud<pcl::PointXYZ> rosa_le_pts;
  visualization_msgs::MarkerArray vID;
  pcl::PointXYZ pt;

  for (int i=0; i<realVertices.rows(); ++i)
  {
    if (realVertices(i,0) < -1e5+1) continue;
    pt.x = realVertices(i,0); pt.y = realVertices(i,1); pt.z = realVertices(i,2);

    rosa_le_pts.points.push_back(pt);
  }

  int count = 0;
  for (int j=0; j<(int)skelver.rows(); ++j)
  {
    visualization_msgs::Marker textMarker;
    textMarker.header.frame_id = "world";
    textMarker.header.stamp = ros::Time::now();
    textMarker.ns = "vertex_ID";
    textMarker.id = count;
    textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    textMarker.action = visualization_msgs::Marker::ADD;

    textMarker.pose.position.x = skelver(j,0);
    textMarker.pose.position.y = skelver(j,1);
    textMarker.pose.position.z = skelver(j,2);
    textMarker.pose.orientation.x = 0.0;
    textMarker.pose.orientation.y = 0.0;
    textMarker.pose.orientation.z = 0.0;
    textMarker.pose.orientation.w = 1.0;
    textMarker.scale.x = 2.0;
    textMarker.scale.y = 2.0;
    textMarker.scale.z = 2.0;
    textMarker.color.r = 0.0;
    textMarker.color.g = 0.0;
    textMarker.color.b = 0.0;
    textMarker.color.a = 1.0;
    textMarker.text = std::to_string(j);

    vID.markers.push_back(textMarker);
    count++;
  }

  rosa_le_pts.width = rosa_le_pts.points.size();
  rosa_le_pts.height = 1;
  rosa_le_pts.is_dense = true;
  rosa_le_pts.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(rosa_le_pts, cloud_msg);

  int counter_vpg = 0;

  visualization_msgs::Marker lines;
  lines.header.frame_id = "world";
  lines.header.stamp = ros::Time::now();
  lines.id = counter_vpg;
  lines.type = visualization_msgs::Marker::LINE_LIST;
  lines.action = visualization_msgs::Marker::ADD;

  lines.pose.orientation.w = 1.0;
  lines.scale.x = 0.3;

  lines.color.r = 0.2;
  lines.color.g = 0.8;
  lines.color.b = 0.4;
  lines.color.a = 1.0;
  
  geometry_msgs::Point p1, p2;
  for (int i=0; i<skeladj.rows(); ++i)
  {
    {
      p1.x = skelver(skeladj(i,0),0); p1.y = skelver(skeladj(i,0),1); p1.z = skelver(skeladj(i,0),2);
      p2.x = skelver(skeladj(i,1),0); p2.y = skelver(skeladj(i,1),1); p2.z = skelver(skeladj(i,1),2);
      lines.points.push_back(p1);
      lines.points.push_back(p2);
    }
  }

  /* Publish */
  rr_pts_pub_.publish(cloud_msg);
  rr_lines_pub_.publish(lines);
  vertex_ID_pub_.publish(vID);
}

void PlanningVisualization::publish_decomposition(Eigen::MatrixXd& nodes, vector<vector<int>>& branches, vector<Eigen::Vector3d>& dirs, vector<Eigen::Vector3d>& centroids)
{
  srand(time(NULL)); 
  visualization_msgs::MarkerArray b_set, bse_set, bdir_set;
  int counter = 0;
  for (int i=0; i<(int)branches.size(); ++i)
  {
    visualization_msgs::Marker segment;
    segment.header.frame_id = "world";
    segment.header.stamp = ros::Time::now();
    segment.id = counter;

    segment.type = visualization_msgs::Marker::LINE_LIST;
    segment.action = visualization_msgs::Marker::ADD;

    segment.pose.orientation.w = 1.0;
    segment.scale.x = 0.3;

    segment.color.r = (double)rand() / (double)RAND_MAX;
    segment.color.g = (double)rand() / (double)RAND_MAX;
    segment.color.b = (double)rand() / (double)RAND_MAX;
    segment.color.a = 1.0;

    geometry_msgs::Point p1, p2;
    for (int j=0; j<(int)branches[i].size()-1; ++j)
    {
      p1.x = nodes(branches[i][j],0); p1.y = nodes(branches[i][j],1); p1.z = nodes(branches[i][j],2); 
      p2.x = nodes(branches[i][j+1],0); p2.y = nodes(branches[i][j+1],1); p2.z = nodes(branches[i][j+1],2);
      segment.points.push_back(p1);
      segment.points.push_back(p2);
    }

    visualization_msgs::Marker begin;
    begin.header.frame_id = "world";
    begin.header.stamp = ros::Time::now();
    begin.id = counter;
    begin.type = visualization_msgs::Marker::CUBE;
    begin.color.r = segment.color.r;
    begin.color.g = segment.color.g;
    begin.color.b = segment.color.b;
    begin.color.a = 0.3;
    begin.scale.x = 1.3;
    begin.scale.y = 1.3;
    begin.scale.z = 1.3;
    begin.pose.orientation.w = 1.0;
    begin.pose.position.x = nodes(branches[i][0],0);
    begin.pose.position.y = nodes(branches[i][0],1);
    begin.pose.position.z = nodes(branches[i][0],2);

    visualization_msgs::Marker end;
    end.header.frame_id = "world";
    end.header.stamp = ros::Time::now();
    end.id = counter+20;
    end.type = visualization_msgs::Marker::SPHERE;
    end.color.r = segment.color.r;
    end.color.g = segment.color.g;
    end.color.b = segment.color.b;
    end.color.a = 1.0;
    end.scale.x = 1.1;
    end.scale.y = 1.1;
    end.scale.z = 1.1;
    end.pose.orientation.w = 1.0;
    end.pose.position.x = nodes(branches[i][(int)branches[i].size()-1],0);
    end.pose.position.y = nodes(branches[i][(int)branches[i].size()-1],1);
    end.pose.position.z = nodes(branches[i][(int)branches[i].size()-1],2);

    double scale = (nodes.row(branches[i][0]) - nodes.row(branches[i][(int)branches[i].size()-1])).norm();
    visualization_msgs::Marker nm;
    nm.header.frame_id = "world";
    nm.header.stamp = ros::Time::now();
    nm.id = counter;
    nm.type = visualization_msgs::Marker::ARROW;
    nm.pose.orientation.w = 1.0;
    nm.scale.x = 0.2;
    nm.scale.y = 0.3;
    nm.scale.z = 0.2;
    geometry_msgs::Point pt_;
    pt_.x = centroids[i](0) - 0.5*scale*dirs[i](0);
    pt_.y = centroids[i](1) - 0.5*scale*dirs[i](1);
    pt_.z = centroids[i](2) - 0.5*scale*dirs[i](2);
    nm.points.push_back(pt_);

    pt_.x = centroids[i](0) + 0.5*scale*dirs[i](0);
    pt_.y = centroids[i](1) + 0.5*scale*dirs[i](1);
    pt_.z = centroids[i](2) + 0.5*scale*dirs[i](2);
    nm.points.push_back(pt_);

    nm.color.r = 0.9;
    nm.color.g = 0.2;
    nm.color.b = 0.1;
    nm.color.a = 1.0;

    b_set.markers.push_back(segment);
    bse_set.markers.push_back(begin);
    bse_set.markers.push_back(end);
    bdir_set.markers.push_back(nm);
    counter++;
  }

  decomp_pub_.publish(b_set);
  branch_start_end_pub_.publish(bse_set);
  branch_dir_pub_.publish(bdir_set);
}

void PlanningVisualization::publishInliers(const pcl::PointCloud<pcl::PointXYZ>& inliers, const pcl::PointCloud<pcl::PointXYZ>& all_pts, const Eigen::MatrixXf& directions, const vector<vector<Eigen::Vector3d>>& isec_pts)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud = inliers;

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = "world";

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  inliers_pub_.publish(cloud_msg);
  
  // intersection
  visualization_msgs::MarkerArray inliers_inter;
  int counter = 0;
  double scale = 1000.0;
  for (int i=0; i<(int)all_pts.points.size(); ++i)
  {
    visualization_msgs::Marker nm;
    nm.header.frame_id = "world";
    nm.header.stamp = ros::Time::now();
    nm.id = counter;
    nm.type = visualization_msgs::Marker::ARROW;
    nm.action = visualization_msgs::Marker::ADD;

    nm.pose.orientation.w = 1.0;
    nm.scale.x = 0.2;
    nm.scale.y = 0.3;
    nm.scale.z = 0.2;

    geometry_msgs::Point pt_;
    pt_.x = all_pts.points[i].x;
    pt_.y = all_pts.points[i].y;
    pt_.z = all_pts.points[i].z;
    nm.points.push_back(pt_);

    pt_.x = all_pts.points[i].x + scale*directions(i,0);
    pt_.y = all_pts.points[i].y + scale*directions(i,1);
    pt_.z = all_pts.points[i].z + scale*directions(i,2);
    nm.points.push_back(pt_);

    nm.color.r = 0.1;
    nm.color.g = 0.2;
    nm.color.b = 0.8;
    nm.color.a = 0.8;

    inliers_inter.markers.push_back(nm);
    counter++;
  }

  inliers_intersection_pub_.publish(inliers_inter);

  pcl::PointCloud<pcl::PointXYZ> cloud_isec;
  pcl::PointXYZ p;
  for (int i=0; i<(int)isec_pts.size(); ++i)
  {
    for (int j=0; j<(int)isec_pts[i].size(); ++j)
    {
      p.x = isec_pts[i][j](0);
      p.y = isec_pts[i][j](1);
      p.z = isec_pts[i][j](2);
      cloud_isec.points.push_back(p);
    }
  }

  cloud_isec.width = cloud_isec.points.size();
  cloud_isec.height = 1;
  cloud_isec.is_dense = true;
  cloud_isec.header.frame_id = "world";

  sensor_msgs::PointCloud2 cloud_isec_msg;
  pcl::toROSMsg(cloud_isec, cloud_isec_msg);
  inliers_isec_pub_.publish(cloud_isec_msg);
}

void PlanningVisualization::publishVisGraph(const vector<Eigen::Vector3d>& inliers, const Eigen::MatrixXi& vis_graph, const vector<vector<int>>& cvx_sets)
{
  visualization_msgs::Marker lines;
  lines.header.frame_id = "world";
  lines.header.stamp = ros::Time::now();
  lines.id = 0;
  lines.ns = "visibility_graph";
  lines.type = visualization_msgs::Marker::LINE_LIST;
  lines.action = visualization_msgs::Marker::ADD;

  lines.pose.orientation.w = 1.0;
  lines.scale.x = 0.07;

  lines.color.r = 0.8;
  lines.color.g = 0.2;
  lines.color.b = 0.2;
  lines.color.a = 0.8;

  geometry_msgs::Point p1, p2;
  for (int i=0; i<vis_graph.rows(); ++i)
  {
    for (int j=0; j<vis_graph.cols(); ++j)
    {
      if (vis_graph(i,j) == 1 && i > j)
      {
        p1.x = inliers[i](0); p1.y = inliers[i](1); p1.z = inliers[i](2);
        p2.x = inliers[j](0); p2.y = inliers[j](1); p2.z = inliers[j](2);
        lines.points.push_back(p1);
        lines.points.push_back(p2);
      }
    }
  }

  vis_graph_pub_.publish(lines);
  
  visualization_msgs::MarkerArray cvx_cluster;
  int counter = 0;
  for (int i=0; i<(int)cvx_sets.size(); ++i)
  {
    vector<int> x = cvx_sets[i];
    for (auto idx:x)
    {
      visualization_msgs::Marker mk;
      mk.header.frame_id = "world";
      mk.header.stamp = ros::Time::now();
      mk.id = counter;
      mk.ns = "cvx_cluster";
      mk.type = visualization_msgs::Marker::SPHERE;
      mk.action = visualization_msgs::Marker::ADD;

      mk.pose.orientation.w = 1.0;
      mk.scale.x = 0.3;
      mk.scale.y = 0.3;
      mk.scale.z = 0.3;

      mk.color.r = red_list[i];
      mk.color.g = green_list[i];
      mk.color.b = blue_list[i];
      mk.color.a = 0.8;

      mk.pose.position.x = inliers[idx](0);
      mk.pose.position.y = inliers[idx](1);
      mk.pose.position.z = inliers[idx](2);
      cvx_cluster.markers.push_back(mk);
      counter++;
    }
  }
  
  cvx_cluster_pub_.publish(cvx_cluster);

  return;
}

void PlanningVisualization::publishIntraEdge(const vector<Eigen::MatrixX3d>& intra_edges)
{
  visualization_msgs::Marker lines;
  lines.header.frame_id = "world";
  lines.header.stamp = ros::Time::now();
  lines.id = 0;
  lines.type = visualization_msgs::Marker::LINE_LIST;
  lines.action = visualization_msgs::Marker::ADD;

  lines.pose.orientation.w = 1.0;
  lines.scale.x = 0.15;

  lines.color.r = 0.2;
  lines.color.g = 0.8;
  lines.color.b = 0.4;
  lines.color.a = 1.0;

  geometry_msgs::Point p1, p2;
  for (int i=0; i<(int)intra_edges.size(); ++i)
  {
    p1.x = intra_edges[i](0,0); p1.y = intra_edges[i](0,1); p1.z = intra_edges[i](0,2);
    p2.x = intra_edges[i](1,0); p2.y = intra_edges[i](1,1); p2.z = intra_edges[i](1,2);
    lines.points.push_back(p1);
    lines.points.push_back(p2);
  }

  intra_edge_pub_.publish(lines);
}

void PlanningVisualization::publishCutPlane(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, Eigen::Vector3d& p, Eigen::Vector3d& v)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_pred;
  cloud_pred = input_cloud;

  cloud_pred.width = cloud_pred.points.size();
  cloud_pred.height = 1;
  cloud_pred.is_dense = true;
  cloud_pred.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_pred, cloud_msg);
  cut_plane_pub_.publish(cloud_msg);

  visualization_msgs::Marker lines;
  lines.header.frame_id = "world";
  lines.header.stamp = ros::Time::now();
  lines.id = 0;
  lines.type = visualization_msgs::Marker::LINE_LIST;
  lines.action = visualization_msgs::Marker::ADD;

  lines.pose.orientation.w = 1.0;
  lines.scale.x = 0.02;

  lines.color.r = 0.2;
  lines.color.g = 0.8;
  lines.color.b = 0.4;
  lines.color.a = 1.0;

  geometry_msgs::Point p1, p2;
  p1.x = p(0); p1.y = p(1); p1.z = p(2); 
  p2.x = p(0)+0.1*v(0); p2.y = p(1)+0.1*v(1); p2.z = p(2)+0.1*v(2); 
  lines.points.push_back(p1);
  lines.points.push_back(p2);

  cut_pt_pub_.publish(lines);
}

void PlanningVisualization::publishSubSpace(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& space)
{
  srand((int)time(0));
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_pcl_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointXYZRGB p;
  int red, blue, green;
  for (int i=0; i<(int)space.size(); ++i)
  {
    red = rand()%255;
    blue = rand()%255;
    green = rand()%255;
    for (int j=0; j<(int)space[i]->points.size(); ++j)
    {
      p.x = space[i]->points[j].x;
      p.y = space[i]->points[j].y;
      p.z = space[i]->points[j].z;
      // color
      p.r = red;
      p.g = green;
      p.b = blue;
      colored_pcl_ptr->points.push_back(p);
    }
  }

  colored_pcl_ptr->width = colored_pcl_ptr->points.size();
  colored_pcl_ptr->height = 1;
  colored_pcl_ptr->is_dense = true;
  colored_pcl_ptr->header.frame_id = "world";

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*colored_pcl_ptr, cloud_msg);
  sub_space_pub_.publish(cloud_msg);
}

void PlanningVisualization::publishSubEndpts(map<int, vector<Eigen::Vector3d>>& endpts)
{
  srand((int)time(0));
  int counter = 0;
  visualization_msgs::MarkerArray sub_endpts;
  for (const auto& pair:endpts)
  {     
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = counter;
    mk.ns = "sub_space_endpt";
    mk.type = visualization_msgs::Marker::CUBE;
    mk.color.r = 1.0;
    mk.color.g = 0.0;
    mk.color.b = 0.0;
    mk.color.a = 1.0;
    mk.scale.x = 1.2;
    mk.scale.y = 1.2;
    mk.scale.z = 1.2;
    mk.pose.orientation.w = 1.0;

    mk.pose.position.x = pair.second[0](0);
    mk.pose.position.y = pair.second[0](1);
    mk.pose.position.z = pair.second[0](2);
    sub_endpts.markers.push_back(mk);
    counter++;
    
    mk.id = counter;
    mk.pose.position.x = pair.second[1](0);
    mk.pose.position.y = pair.second[1](1);
    mk.pose.position.z = pair.second[1](2);
    sub_endpts.markers.push_back(mk);
    counter++;
  }

  sub_endpts_pub_.publish(sub_endpts);
}

void PlanningVisualization::publishSegViewpoints(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& seg_vps)
{
  srand((int)time(0));
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_pcl_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointXYZRGB p;
  int red, blue, green;

  for (int i=0; i<(int)seg_vps.size(); ++i)
  {
    red = rand()%255;
    blue = rand()%255;
    green = rand()%255;
    for (int j=0; j<(int)seg_vps[i]->points.size(); ++j)
    {
      p.x = seg_vps[i]->points[j].x;
      p.y = seg_vps[i]->points[j].y;
      p.z = seg_vps[i]->points[j].z;
      // color
      p.r = red;
      p.g = green;
      p.b = blue;
      colored_pcl_ptr->points.push_back(p);
    }
  }

  colored_pcl_ptr->width = colored_pcl_ptr->points.size();
  colored_pcl_ptr->height = 1;
  colored_pcl_ptr->is_dense = true;
  colored_pcl_ptr->header.frame_id = "world";

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*colored_pcl_ptr, cloud_msg);
  hcopp_viewpoints_pub_.publish(cloud_msg);
}

void PlanningVisualization::publishOccupied(pcl::PointCloud<pcl::PointXYZ>& occupied)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_pred;
  cloud_pred = occupied;

  cloud_pred.width = cloud_pred.points.size();
  cloud_pred.height = 1;
  cloud_pred.is_dense = true;
  cloud_pred.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_pred, cloud_msg);
  hcopp_occ_pub_.publish(cloud_msg);
}

void PlanningVisualization::publishInternal(pcl::PointCloud<pcl::PointXYZ>& internal)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_pred;
  cloud_pred = internal;

  cloud_pred.width = cloud_pred.points.size();
  cloud_pred.height = 1;
  cloud_pred.is_dense = true;
  cloud_pred.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_pred, cloud_msg);
  hcopp_internal_pub_.publish(cloud_msg);
}

void PlanningVisualization::publishFOV(const vector<vector<Eigen::Vector3d>>& list1, const vector<vector<Eigen::Vector3d>>& list2)
{
  visualization_msgs::MarkerArray vp_set;
  int counter = 0;
  for (int j=0; j<(int)list1.size(); ++j)
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = counter;
    mk.ns = "current_pose";
    mk.type = visualization_msgs::Marker::LINE_LIST;
    mk.color.r = 1.0;
    mk.color.g = 0.1;
    mk.color.b = 0.1;
    mk.color.a = 0.85;
    mk.scale.x = 0.18;
    mk.scale.y = 0.18;
    mk.scale.z = 0.18;
    mk.pose.orientation.w = 1.0;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(list1[j].size()); ++i) {
      pt.x = list1[j][i](0);
      pt.y = list1[j][i](1);
      pt.z = list1[j][i](2);
      mk.points.push_back(pt);

      pt.x = list2[j][i](0);
      pt.y = list2[j][i](1);
      pt.z = list2[j][i](2);
      mk.points.push_back(pt);
    }
    
    vp_set.markers.push_back(mk);
    counter++;
  }

  hcopp_validvp_pub_.publish(vp_set);
}

void PlanningVisualization::publishUncovered(pcl::PointCloud<pcl::PointXYZ>& uncovered)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_pred;
  cloud_pred = uncovered;

  cloud_pred.width = cloud_pred.points.size();
  cloud_pred.height = 1;
  cloud_pred.is_dense = true;
  cloud_pred.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_pred, cloud_msg);
  hcopp_uncovered_pub_.publish(cloud_msg);
}

void PlanningVisualization::publishGlobalUncovered(pcl::PointCloud<pcl::PointXYZ>& uncovered)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_pred;
  cloud_pred = uncovered;

  cloud_pred.width = cloud_pred.points.size();
  cloud_pred.height = 1;
  cloud_pred.is_dense = true;
  cloud_pred.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_pred, cloud_msg);
  hcopp_global_uncovered_pub_.publish(cloud_msg);
}

void PlanningVisualization::publishRevisedNormal(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, const pcl::PointCloud<pcl::Normal>& normals)
{
  visualization_msgs::MarkerArray pcloud_normals;
  int counter = 0;
  double scale = 5.0;
  for (int i=0; i<(int)input_cloud.points.size(); ++i)
  {
    visualization_msgs::Marker nm;
    nm.header.frame_id = "world";
    nm.header.stamp = ros::Time::now();
    nm.id = counter;
    nm.type = visualization_msgs::Marker::ARROW;
    nm.action = visualization_msgs::Marker::ADD;

    nm.pose.orientation.w = 1.0;
    nm.scale.x = 0.2;
    nm.scale.y = 0.3;
    nm.scale.z = 0.2;

    geometry_msgs::Point pt_;
    pt_.x = input_cloud.points[i].x;
    pt_.y = input_cloud.points[i].y;
    pt_.z = input_cloud.points[i].z;
    nm.points.push_back(pt_);

    pt_.x = input_cloud.points[i].x + scale*normals.points[i].normal_x;
    pt_.y = input_cloud.points[i].y + scale*normals.points[i].normal_y;
    pt_.z = input_cloud.points[i].z + scale*normals.points[i].normal_z;
    nm.points.push_back(pt_);

    nm.color.r = 0.1;
    nm.color.g = 0.2;
    nm.color.b = 0.7;
    nm.color.a = 1.0;
    
    pcloud_normals.markers.push_back(nm);
    counter++;
  }

  hcopp_correctnormal_pub_.publish(pcloud_normals);
}

void PlanningVisualization::publishFinalFOV(map<int, vector<vector<Eigen::Vector3d>>>& list1, map<int, vector<vector<Eigen::Vector3d>>>& list2, map<int, vector<double>>& yaws)
{
  double fov_scale = 0.03;

  srand((int)time(0));
  int red, blue, green;
  visualization_msgs::MarkerArray final_vps;
  visualization_msgs::MarkerArray vps_drones;
  int counter = 0;
  int sub_id;
  for (auto& sub_fov:list1)
  {
    sub_id = sub_fov.first;
    red = rand()%255;
    blue = rand()%255;
    green = rand()%255;

    for (int j=0; j<(int)sub_fov.second.size(); ++j)
    {
      visualization_msgs::Marker mk;
      mk.header.frame_id = "world";
      mk.header.stamp = ros::Time::now();
      mk.id = counter;
      mk.ns = "sub_viewpoints";
      mk.type = visualization_msgs::Marker::LINE_LIST;
      mk.color.r = red;
      mk.color.g = blue;
      mk.color.b = green;
      mk.color.a = 1.0;
      mk.scale.x = fov_scale;
      mk.scale.y = fov_scale;
      mk.scale.z = fov_scale;
      mk.pose.orientation.w = 1.0;

      // visualization_msgs::Marker meshROS;
      // meshROS.header.frame_id = "world";
      // meshROS.header.stamp = ros::Time::now();
      // meshROS.id = counter;
      // meshROS.ns = "drones_mesh";
      // meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
      // meshROS.color.r = 0.8;
      // meshROS.color.g = 0.0;
      // meshROS.color.b = 0.5;
      // meshROS.color.a = 1.0;
      // meshROS.scale.x = 8.0;
      // meshROS.scale.y = 8.0;
      // meshROS.scale.z = 9.0;
      // meshROS.pose.orientation.w = 1.0;
      // meshROS.mesh_resource = "file://";
      // meshROS.mesh_resource += droneMesh;

      // double yaw = yaws[sub_id][j];
      // double yaw_mesh = yaw;
      // meshROS.pose.orientation.x = 0.0;
      // meshROS.pose.orientation.y = 0.0;
      // meshROS.pose.orientation.z = sin(0.5*yaw_mesh);
      // meshROS.pose.orientation.w = cos(0.5*yaw_mesh);
      // meshROS.pose.position.x = list1[sub_id][j][0](0);
      // meshROS.pose.position.y = list1[sub_id][j][0](1);
      // meshROS.pose.position.z = list1[sub_id][j][0](2);

      geometry_msgs::Point pt;
      for (int i = 0; i < int(sub_fov.second[j].size()); ++i) {
        pt.x = list1[sub_id][j][i](0);
        pt.y = list1[sub_id][j][i](1);
        pt.z = list1[sub_id][j][i](2);
        mk.points.push_back(pt);

        pt.x = list2[sub_id][j][i](0);
        pt.y = list2[sub_id][j][i](1);
        pt.z = list2[sub_id][j][i](2);
        mk.points.push_back(pt);
      }

      // vps_drones.markers.push_back(meshROS);
      final_vps.markers.push_back(mk);
      counter++;
    }
  }

  hcopp_sub_finalvps_pub_.publish(final_vps);
  // hcopp_vps_drone_pub_.publish(vps_drones);
}

void PlanningVisualization::publishGlobalSeq(Eigen::Vector3d& start_, vector<Eigen::Vector3d>& sub_rep, vector<int>& global_seq)
{
  vector<Eigen::Vector3d> total_site_;
  total_site_.push_back(start_);
  total_site_.insert(total_site_.begin()+1, sub_rep.begin(), sub_rep.end());
  vector<int> total_seq_;
  total_seq_.push_back(0);
  for (auto x:global_seq)
    total_seq_.push_back(x+1);

  visualization_msgs::MarkerArray global_results;
  int counter = 0;

  visualization_msgs::Marker begin;
  begin.header.frame_id = "world";
  begin.header.stamp = ros::Time::now();
  begin.id = counter;
  begin.ns = "current_pose";
  begin.type = visualization_msgs::Marker::SPHERE;
  begin.color.r = 1.0;
  begin.color.g = 0.0;
  begin.color.b = 0.0;
  begin.color.a = 1.0;
  begin.scale.x = 1.5;
  begin.scale.y = 1.5;
  begin.scale.z = 1.5;
  begin.pose.orientation.w = 1.0;
  begin.pose.position.x = start_(0);
  begin.pose.position.y = start_(1);
  begin.pose.position.z = start_(2);

  global_results.markers.push_back(begin);
  counter++;

  for (int i=0; i<(int)sub_rep.size(); ++i)
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = counter;
    mk.ns = "current_pose";
    mk.type = visualization_msgs::Marker::CUBE;
    mk.color.r = 0.0;
    mk.color.g = 0.0;
    mk.color.b = 1.0;
    mk.color.a = 1.0;
    mk.scale.x = 1.5;
    mk.scale.y = 1.5;
    mk.scale.z = 1.5;
    mk.pose.orientation.w = 1.0;
    mk.pose.position.x = sub_rep[i](0);
    mk.pose.position.y = sub_rep[i](1);
    mk.pose.position.z = sub_rep[i](2);

    global_results.markers.push_back(mk);
    counter++;
  }

  for (int j=0; j<(int)total_seq_.size()-1; ++j)
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = counter;
    mk.ns = "current_pose";
    mk.type = visualization_msgs::Marker::LINE_LIST;
    mk.color.r = 0.0;
    mk.color.g = 1.0;
    mk.color.b = 0.0;
    mk.color.a = 1.0;
    mk.scale.x = 1.5;
    mk.scale.y = 1.5;
    mk.scale.z = 1.5;
    mk.pose.orientation.w = 1.0;

    geometry_msgs::Point pt;
    pt.x = total_site_[total_seq_[j]](0);
    pt.y = total_site_[total_seq_[j]](1);
    pt.z = total_site_[total_seq_[j]](2);
    mk.points.push_back(pt);

    pt.x = total_site_[total_seq_[j+1]](0);
    pt.y = total_site_[total_seq_[j+1]](1);
    pt.z = total_site_[total_seq_[j+1]](2);
    mk.points.push_back(pt);

    global_results.markers.push_back(mk);
    counter++;
  }

  hcopp_globalseq_pub_.publish(global_results);
}

void PlanningVisualization::publishGlobalBoundary(Eigen::Vector3d& start_, map<int, vector<int>>& boundary_id_, map<int, vector<Eigen::VectorXd>>& sub_vps, vector<int>& global_seq)
{
  vector<Eigen::Vector3d> boundaries;
  boundaries.push_back(start_);
  Eigen::Vector3d b_s, b_e;
  for (auto id:global_seq)
  {
    if ((int)boundary_id_.find(id)->second.size() == 2)
    {
      b_s(0) = sub_vps.find(id)->second[boundary_id_.find(id)->second[0]](0);
      b_s(1) = sub_vps.find(id)->second[boundary_id_.find(id)->second[0]](1);
      b_s(2) = sub_vps.find(id)->second[boundary_id_.find(id)->second[0]](2);
      boundaries.push_back(b_s);
      b_e(0) = sub_vps.find(id)->second[boundary_id_.find(id)->second[1]](0);
      b_e(1) = sub_vps.find(id)->second[boundary_id_.find(id)->second[1]](1);
      b_e(2) = sub_vps.find(id)->second[boundary_id_.find(id)->second[1]](2);
      boundaries.push_back(b_e);
    }
    else
    {
      b_s(0) = sub_vps.find(id)->second[boundary_id_.find(id)->second[0]](0);
      b_s(1) = sub_vps.find(id)->second[boundary_id_.find(id)->second[0]](1);
      b_s(2) = sub_vps.find(id)->second[boundary_id_.find(id)->second[0]](2);
      boundaries.push_back(b_s);
    }
  }
  
  visualization_msgs::MarkerArray boundary_results;
  int counter = 0;
  for (int i=0; i<(int)boundaries.size(); ++i)
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = counter;
    mk.ns = "current_pose";
    mk.type = visualization_msgs::Marker::CUBE;
    mk.color.r = 0.0;
    mk.color.g = 1.0;
    mk.color.b = 1.0;
    mk.color.a = 1.0;
    mk.scale.x = 1.2;
    mk.scale.y = 1.2;
    mk.scale.z = 1.2;
    mk.pose.orientation.w = 1.0;
    mk.pose.position.x = boundaries[i](0);
    mk.pose.position.y = boundaries[i](1);
    mk.pose.position.z = boundaries[i](2);

    boundary_results.markers.push_back(mk);
    counter++;
  }

  for (int j=0; j<(int)boundaries.size()-1; ++j)
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = counter;
    mk.ns = "current_pose";
    mk.type = visualization_msgs::Marker::LINE_LIST;
    mk.color.r = 1.0;
    mk.color.g = 0.0;
    mk.color.b = 0.0;
    mk.color.a = 1.0;
    mk.scale.x = 1.0;
    mk.scale.y = 1.0;
    mk.scale.z = 1.0;
    mk.pose.orientation.w = 1.0;

    geometry_msgs::Point pt;
    pt.x = boundaries[j](0);
    pt.y = boundaries[j](1);
    pt.z = boundaries[j](2);
    mk.points.push_back(pt);

    pt.x = boundaries[j+1](0);
    pt.y = boundaries[j+1](1);
    pt.z = boundaries[j+1](2);
    mk.points.push_back(pt);

    boundary_results.markers.push_back(mk);
    counter++;
  }

  hcopp_globalboundary_pub_.publish(boundary_results);
}

void PlanningVisualization::publishLocalPath(map<int, vector<Eigen::VectorXd>>& sub_paths_)
{
  srand((int)time(0));
  int red, blue, green;
  vector<Eigen::VectorXd> vps_;
  int counter = 0;
  visualization_msgs::MarkerArray local_results;
  for (const auto& pair:sub_paths_)
  {
    vps_ = pair.second;
    red = rand()%255;
    blue = rand()%255;
    green = rand()%255;

    for (int i=0; i<(int)vps_.size()-1; ++i)
    {
      visualization_msgs::Marker mk;
      mk.header.frame_id = "world";
      mk.header.stamp = ros::Time::now();
      mk.id = counter;
      mk.ns = "current_pose";
      mk.type = visualization_msgs::Marker::LINE_LIST;
      mk.color.r = red;
      mk.color.g = blue;
      mk.color.b = green;
      mk.color.a = 1.0;
      mk.scale.x = 1.0;
      mk.scale.y = 1.0;
      mk.scale.z = 1.0;
      mk.pose.orientation.w = 1.0;

      geometry_msgs::Point pt;
      pt.x = vps_[i](0);
      pt.y = vps_[i](1);
      pt.z = vps_[i](2);
      mk.points.push_back(pt);

      pt.x = vps_[i+1](0);
      pt.y = vps_[i+1](1);
      pt.z = vps_[i+1](2);
      mk.points.push_back(pt);

      local_results.markers.push_back(mk);
      counter++;
    }
  }

  hcopp_local_path_pub_.publish(local_results);
}

void PlanningVisualization::publishHCOPPPath(vector<Eigen::VectorXd>& fullpath_)
{
  double line_scale = 0.07, point_scale = 0.15;
  
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 0;
  mk.ns = "global_path";
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.color.r = 0.3;
  mk.color.g = 1.0;
  mk.color.b = 0.3;
  mk.color.a = 1.0;
  mk.scale.x = line_scale;
  mk.scale.y = line_scale;
  mk.scale.z = line_scale;
  mk.pose.orientation.w = 1.0;

  mk.action = visualization_msgs::Marker::DELETEALL;
  hcopp_full_path_pub_.publish(mk);
  geometry_msgs::Point pt;
  for (int i=0; i<(int)fullpath_.size()-1; ++i)
  {
    pt.x = fullpath_[i](0);
    pt.y = fullpath_[i](1);
    pt.z = fullpath_[i](2);
    mk.points.push_back(pt);

    pt.x = fullpath_[i+1](0);
    pt.y = fullpath_[i+1](1);
    pt.z = fullpath_[i+1](2);
    mk.points.push_back(pt);
  }

  mk.action = visualization_msgs::Marker::ADD;
  hcopp_full_path_pub_.publish(mk);

  visualization_msgs::Marker mk_pts;
  mk_pts.header.frame_id = "world";
  mk_pts.header.stamp = ros::Time::now();
  mk_pts.id = 0;
  mk_pts.ns = "global_path_waypts";
  mk_pts.type = visualization_msgs::Marker::SPHERE_LIST;
  mk_pts.color.r = 0.0;
  mk_pts.color.g = 0.0;
  mk_pts.color.b = 0.0;
  mk_pts.color.a = 1.0;
  mk_pts.scale.x = point_scale;
  mk_pts.scale.y = point_scale;
  mk_pts.scale.z = point_scale;
  mk_pts.pose.orientation.w = 1.0;

  mk_pts.action = visualization_msgs::Marker::DELETEALL;
  hcopp_full_waypts_pub_.publish(mk_pts);

  for (int i=0; i<(int)fullpath_.size(); ++i)
  {
    pt.x = fullpath_[i](0);
    pt.y = fullpath_[i](1);
    pt.z = fullpath_[i](2);
    mk_pts.points.push_back(pt);
  }

  mk_pts.action = visualization_msgs::Marker::ADD;
  hcopp_full_waypts_pub_.publish(mk_pts);
}

void PlanningVisualization::publishBeforeConsistencyPath(const vector<Eigen::VectorXd>& path)
{
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 0;
  mk.ns = "before_consis_path";
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.color.r = 0.6;
  mk.color.g = 0.0;
  mk.color.b = 0.3;
  mk.color.a = 1.0;
  mk.scale.x = 0.2;
  mk.scale.y = 0.2;
  mk.scale.z = 0.2;
  mk.pose.orientation.w = 1.0;

  mk.action = visualization_msgs::Marker::DELETEALL;
  before_consistency_path_pub_.publish(mk);
  
  geometry_msgs::Point pt;
  for (int i=0; i<(int)path.size()-1; ++i)
  {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);

    pt.x = path[i+1](0);
    pt.y = path[i+1](1);
    pt.z = path[i+1](2);
    mk.points.push_back(pt);
  }

  mk.action = visualization_msgs::Marker::ADD;
  before_consistency_path_pub_.publish(mk);
}

void PlanningVisualization::publishGlobalNextSubConsistency(const vector<Eigen::Vector3d>& last, const vector<Eigen::Vector3d>& current)
{
  visualization_msgs::MarkerArray two_inliers;

  double last_scale = 0.7, current_scale = 0.8;

  visualization_msgs::Marker last_marker;
  last_marker.header.frame_id = "world";
  last_marker.header.stamp = ros::Time::now();
  last_marker.id = 0;
  last_marker.ns = "last_subspace";
  last_marker.type = visualization_msgs::Marker::CUBE_LIST;
  last_marker.color.r = 0.0;
  last_marker.color.g = 0.8;
  last_marker.color.b = 0.5;
  last_marker.color.a = 1.0;
  last_marker.scale.x = last_scale;
  last_marker.scale.y = last_scale;
  last_marker.scale.z = last_scale;
  last_marker.pose.orientation.w = 1.0;

  for (int i=0; i<(int)last.size(); ++i)
  {
    geometry_msgs::Point pt;
    pt.x = last[i](0);
    pt.y = last[i](1);
    pt.z = last[i](2);
    last_marker.points.push_back(pt);
  }
  two_inliers.markers.push_back(last_marker);

  visualization_msgs::Marker current_marker;
  current_marker.header.frame_id = "world";
  current_marker.header.stamp = ros::Time::now();
  current_marker.id = 1;
  current_marker.ns = "current_subspace";
  current_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  current_marker.color.r = 0.0;
  current_marker.color.g = 0.0;
  current_marker.color.b = 1.0;
  current_marker.color.a = 0.6;
  current_marker.scale.x = current_scale;
  current_marker.scale.y = current_scale;
  current_marker.scale.z = current_scale;
  current_marker.pose.orientation.w = 1.0;

  for (int i=0; i<(int)current.size(); ++i)
  {
    geometry_msgs::Point pt;
    pt.x = current[i](0);
    pt.y = current[i](1);
    pt.z = current[i](2);
    current_marker.points.push_back(pt);
  }
  two_inliers.markers.push_back(current_marker);

  global_next_sub_consistency_pub_.publish(two_inliers);

  return;
}

void PlanningVisualization::publishGlobalVPVisGraph(const vector<Eigen::VectorXd>& vps, const Eigen::MatrixXi& vp_vis_graph, const vector<vector<int>>& decomp_vps)
{
  visualization_msgs::Marker lines;
  lines.header.frame_id = "world";
  lines.header.stamp = ros::Time::now();
  lines.id = 0;
  lines.ns = "vps_vis_graph";
  lines.type = visualization_msgs::Marker::LINE_LIST;
  lines.action = visualization_msgs::Marker::ADD;

  lines.pose.orientation.w = 1.0;
  lines.scale.x = 0.05;

  lines.color.r = 0.3;
  lines.color.g = 0.1;
  lines.color.b = 0.8;
  lines.color.a = 0.5;

  geometry_msgs::Point p1, p2;
  for (int i=0; i<vp_vis_graph.rows(); ++i)
  {
    for (int j=0; j<vp_vis_graph.cols(); ++j)
    {
      if (vp_vis_graph(i,j) == 1 && i > j)
      {
        p1.x = vps[i](0); p1.y = vps[i](1); p1.z = vps[i](2);
        p2.x = vps[j](0); p2.y = vps[j](1); p2.z = vps[j](2);
        lines.points.push_back(p1);
        lines.points.push_back(p2);
      }
    }
  }

  global_vp_vis_graph_pub_.publish(lines);

  visualization_msgs::MarkerArray decomp_vps_markers;
  for (int i=0; i<(int)decomp_vps.size(); ++i)
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = i;
    mk.ns = "decomp_vps";
    mk.type = visualization_msgs::Marker::SPHERE_LIST;
    mk.color.r = this->red_list[i];
    mk.color.g = this->green_list[i];
    mk.color.b = this->blue_list[i];
    mk.color.a = 1.0;
    mk.scale.x = 0.15;
    mk.scale.y = 0.15;
    mk.scale.z = 0.15;
    mk.pose.orientation.w = 1.0;

    for (int j=0; j<(int)decomp_vps[i].size(); ++j)
    {
      geometry_msgs::Point pt;
      pt.x = vps[decomp_vps[i][j]](0);
      pt.y = vps[decomp_vps[i][j]](1);
      pt.z = vps[decomp_vps[i][j]](2);
      mk.points.push_back(pt);
    }

    decomp_vps_markers.markers.push_back(mk);
  }

  global_vp_decomp_pub_.publish(decomp_vps_markers);

  return;
}

void PlanningVisualization::publishDecompGlobalPath(const vector<Eigen::Vector3d>& global_path)
{
  visualization_msgs::MarkerArray global_path_markers;

  visualization_msgs::Marker pts;
  pts.header.frame_id = "world";
  pts.header.stamp = ros::Time::now();
  pts.id = 0;
  pts.ns = "decomp_global_path";
  pts.type = visualization_msgs::Marker::CUBE_LIST;
  pts.color.r = 0.0;
  pts.color.g = 0.0;
  pts.color.b = 0.0;
  pts.color.a = 0.5;
  pts.scale.x = 1.2;
  pts.scale.y = 1.2;
  pts.scale.z = 1.2;
  pts.pose.orientation.w = 1.0;

  for (int i=0; i<(int)global_path.size(); ++i)
  {
    geometry_msgs::Point pt;
    pt.x = global_path[i](0);
    pt.y = global_path[i](1);
    pt.z = global_path[i](2);
    pts.points.push_back(pt);
  }

  global_path_markers.markers.push_back(pts);

  if ((int)global_path.size() > 1)
  {
    visualization_msgs::Marker lines;
    lines.header.frame_id = "world";
    lines.header.stamp = ros::Time::now();
    lines.id = 1;
    lines.ns = "decomp_global_path";
    lines.type = visualization_msgs::Marker::LINE_LIST;
    lines.color.r = 0.5;
    lines.color.g = 0.5;
    lines.color.b = 0.0;
    lines.color.a = 0.7;
    lines.scale.x = 0.6;
    lines.scale.y = 0.6;
    lines.scale.z = 0.6;
    lines.pose.orientation.w = 1.0;

    geometry_msgs::Point p1, p2;
    for (int i=0; i<(int)global_path.size()-1; ++i)
    {
      p1.x = global_path[i](0); p1.y = global_path[i](1); p1.z = global_path[i](2);
      p2.x = global_path[i+1](0); p2.y = global_path[i+1](1); p2.z = global_path[i+1](2);
      lines.points.push_back(p1);
      lines.points.push_back(p2);
    }

    global_path_markers.markers.push_back(lines);
  }

  global_decomp_path_pub_.publish(global_path_markers);

  return;
}

void PlanningVisualization::publishFullATSPPath(vector<Eigen::VectorXd>& fullpath_)
{
  // srand((int)time(0));
  // int red, blue, green;
  // red = rand()%255;
  // green = rand()%255;
  // blue = rand()%255;
  int counter = 0;
  visualization_msgs::MarkerArray fullatsp_results;
  for (int i=0; i<(int)fullpath_.size()-1; ++i)
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = counter;
    mk.ns = "current_pose";
    mk.type = visualization_msgs::Marker::LINE_LIST;
    mk.color.r = 0.0;
    mk.color.g = 0.0;
    mk.color.b = 1.0;
    mk.color.a = 1.0;
    mk.scale.x = 0.5;
    mk.scale.y = 0.5;
    mk.scale.z = 0.5;
    mk.pose.orientation.w = 1.0;

    geometry_msgs::Point pt;
    pt.x = fullpath_[i](0);
    pt.y = fullpath_[i](1);
    pt.z = fullpath_[i](2);
    mk.points.push_back(pt);

    pt.x = fullpath_[i+1](0);
    pt.y = fullpath_[i+1](1);
    pt.z = fullpath_[i+1](2);
    mk.points.push_back(pt);

    fullatsp_results.markers.push_back(mk);
    counter++;
  }

  fullatsp_full_path_pub_.publish(fullatsp_results);
}

void PlanningVisualization::publishFullGDCPCAPath(vector<Eigen::VectorXd>& fullpath_)
{
  int counter = 0;
  visualization_msgs::MarkerArray fullgdcpca_results;
  for (int i=0; i<(int)fullpath_.size()-1; ++i)
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = counter;
    mk.ns = "current_pose";
    mk.type = visualization_msgs::Marker::LINE_LIST;
    mk.color.r = 0.0;
    mk.color.g = 1.0;
    mk.color.b = 0.0;
    mk.color.a = 1.0;
    mk.scale.x = 0.5;
    mk.scale.y = 0.5;
    mk.scale.z = 0.5;
    mk.pose.orientation.w = 1.0;

    geometry_msgs::Point pt;
    pt.x = fullpath_[i](0);
    pt.y = fullpath_[i](1);
    pt.z = fullpath_[i](2);
    mk.points.push_back(pt);

    pt.x = fullpath_[i+1](0);
    pt.y = fullpath_[i+1](1);
    pt.z = fullpath_[i+1](2);
    mk.points.push_back(pt);

    fullgdcpca_results.markers.push_back(mk);
    counter++;
  }

  fullgdcpca_full_path_pub_.publish(fullgdcpca_results);
}

void PlanningVisualization::publishPCAVec(vector<Eigen::Vector3d>& sub_center, map<int, Eigen::Matrix3d>& sub_pcavec)
{
  visualization_msgs::MarkerArray pca_results;
  int sub_id, counter = 0;
  vector<double> red = {1.0, 0.0, 0.0};
  vector<double> green = {0.0, 1.0, 0.0};
  vector<double> blue = {0.0, 0.0, 1.0};
  double scale = 5.0;

  for (auto& pair:sub_pcavec)
  {
    sub_id = pair.first;
    Eigen::Vector3d center_ = sub_center[sub_id];
    for (int i=0; i<3; ++i)
    {
      visualization_msgs::Marker mk;
      mk.header.frame_id = "world";
      mk.header.stamp = ros::Time::now();
      mk.id = counter;
      mk.ns = "pca_vec";
      mk.type = visualization_msgs::Marker::LINE_LIST;
      mk.color.r = red[i];
      mk.color.g = green[i];
      mk.color.b = blue[i];
      mk.color.a = 1.0;
      mk.scale.x = 0.5;
      mk.scale.y = 0.5;
      mk.scale.z = 0.5;
      mk.pose.orientation.w = 1.0;

      geometry_msgs::Point pt;
      pt.x = center_(0);
      pt.y = center_(1);
      pt.z = center_(2);
      mk.points.push_back(pt);

      pt.x = center_(0) + scale*pair.second(i,0);
      pt.y = center_(1) + scale*pair.second(i,1);
      pt.z = center_(2) + scale*pair.second(i,2);
      mk.points.push_back(pt);

      pca_results.markers.push_back(mk);
      counter++;
    }
  }
  
  pca_vec_pub_.publish(pca_results);
}

void PlanningVisualization::publishVPOpt(pcl::PointCloud<pcl::PointNormal>::Ptr& before_, pcl::PointCloud<pcl::PointNormal>::Ptr& after_)
{
  pcl::PointCloud<pcl::PointXYZ> before_cloud;
  pcl::PointXYZ before_pt;
  for (int i=0; i<(int)before_->points.size(); ++i)
  {
    before_pt.x = before_->points[i].x; 
    before_pt.y = before_->points[i].y; 
    before_pt.z = before_->points[i].z;
    before_cloud.points.push_back(before_pt); 
  }
  before_cloud.width = before_cloud.points.size();
  before_cloud.height = 1;
  before_cloud.is_dense = true;
  before_cloud.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 before_msg;
  pcl::toROSMsg(before_cloud, before_msg);
  before_opt_vp_pub_.publish(before_msg);

  pcl::PointCloud<pcl::PointXYZ> after_cloud;
  pcl::PointXYZ after_pt;
  for (int i=0; i<(int)after_->points.size(); ++i)
  {
    after_pt.x = after_->points[i].x; 
    after_pt.y = after_->points[i].y; 
    after_pt.z = after_->points[i].z;
    after_cloud.points.push_back(after_pt); 
  }
  after_cloud.width = after_cloud.points.size();
  after_cloud.height = 1;
  after_cloud.is_dense = true;
  after_cloud.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 after_msg;
  pcl::toROSMsg(after_cloud, after_msg);
  after_opt_vp_pub_.publish(after_msg);
}

void PlanningVisualization::publishFitCylinder(map<int, vector<double>>& cylinder_param)
{
  visualization_msgs::MarkerArray cylinder_set;
  int counter = 0;
  srand((int)time(0));
  Eigen::Vector3d rot_vec;

  for (auto& pair:cylinder_param)
  { 
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = counter;
    mk.ns = "fit_cylinder";
    mk.type = visualization_msgs::Marker::LINE_LIST;
    mk.color.r = 1.0;
    mk.color.g = 0.0;
    mk.color.b = 0.0;
    mk.color.a = 1.0;
    mk.scale.x = 0.5;
    mk.scale.y = 0.5;
    mk.scale.z = 0.5;
    mk.pose.orientation.w = 1.0;

    geometry_msgs::Point pt;
    pt.x = pair.second[9];
    pt.y = pair.second[10];
    pt.z = pair.second[11];
    mk.points.push_back(pt);

    pt.x = pair.second[0];
    pt.y = pair.second[1];
    pt.z = pair.second[2];
    mk.points.push_back(pt);

    pt.x = pair.second[0];
    pt.y = pair.second[1];
    pt.z = pair.second[2];
    mk.points.push_back(pt);

    pt.x = pair.second[6];
    pt.y = pair.second[7];
    pt.z = pair.second[8];
    mk.points.push_back(pt);

    cylinder_set.markers.push_back(mk);
    counter++;
  }

  cylinder_pub_.publish(cylinder_set);
}

void PlanningVisualization::publishHCOPPTraj(quadrotor_msgs::PolynomialTraj& posi, quadrotor_msgs::PolynomialTraj& pitch, quadrotor_msgs::PolynomialTraj& yaw)
{
  posi_traj_pub_.publish(posi);
  pitch_traj_pub_.publish(pitch);
  yaw_traj_pub_.publish(yaw);
}

void PlanningVisualization::publishJointSphere(vector<Eigen::Vector3d>& joints, double& radius, vector<vector<Eigen::Vector3d>>& InnerVps)
{
  visualization_msgs::MarkerArray JointSpheres;
  int counter = 0, vpscount = 0;

  for (int i=0; i<(int)joints.size(); ++i)
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = counter;
    mk.ns = "JointSphere";
    mk.type = visualization_msgs::Marker::SPHERE;
    mk.color.r = 1.0;
    mk.color.g = 0.5;
    mk.color.b = 0.0;
    mk.color.a = 0.5;
    mk.scale.x = 2*radius;
    mk.scale.y = 2*radius;
    mk.scale.z = 2*radius;
    mk.pose.orientation.w = 1.0;
    mk.pose.position.x = joints[i](0);
    mk.pose.position.y = joints[i](1);
    mk.pose.position.z = joints[i](2);

    JointSpheres.markers.push_back(mk);
    counter++;

    for (int j=0; j<(int)InnerVps[i].size(); ++j)
    {
      visualization_msgs::Marker vp;
      vp.header.frame_id = "world";
      vp.header.stamp = ros::Time::now();
      vp.id = vpscount;
      vp.ns = "JointVp";
      vp.type = visualization_msgs::Marker::CUBE;
      vp.color.r = 0.0;
      vp.color.g = 0.0;
      vp.color.b = 1.0;
      vp.color.a = 1.0;
      vp.scale.x = 1.5;
      vp.scale.y = 1.5;
      vp.scale.z = 1.5;
      vp.pose.orientation.w = 1.0;
      vp.pose.position.x = InnerVps[i][j](0);
      vp.pose.position.y = InnerVps[i][j](1);
      vp.pose.position.z = InnerVps[i][j](2);

      JointSpheres.markers.push_back(vp);
      vpscount++;
    }
  }

  jointSphere_pub_.publish(JointSpheres);
}

void PlanningVisualization::publishYawTraj(vector<Eigen::Vector3d>& waypt, vector<double>& yaw)
{
  visualization_msgs::MarkerArray yaw_traj;
  int counter = 0;
  double scale = 3.0;
  for (int i=0; i<(int)yaw.size(); ++i)
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = counter;
    mk.ns = "yaw_traj";
    mk.type = visualization_msgs::Marker::ARROW;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = 0.2;
    mk.scale.y = 0.4;
    mk.scale.z = 0.3;
    mk.color.r = 0.0;
    mk.color.g = 0.0;
    mk.color.b = 1.0;
    mk.color.a = 1.0;

    geometry_msgs::Point pt_;
    pt_.x = waypt[i](0);
    pt_.y = waypt[i](1);
    pt_.z = waypt[i](2);
    mk.points.push_back(pt_);

    pt_.x = waypt[i](0) + scale*cos(yaw[i]);
    pt_.y = waypt[i](1) + scale*sin(yaw[i]);
    pt_.z = waypt[i](2);
    mk.points.push_back(pt_);

    yaw_traj.markers.push_back(mk);
    counter++;
  }

  hcoppYaw_pub_.publish(yaw_traj);
}

void PlanningVisualization::publishCurrentFoV(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2, const double& yaw)
{
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 0;
  mk.ns = "current_fov";
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.color.r = 0.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.color.a = 1.0;
  mk.scale.x = 0.2;
  mk.scale.y = 0.2;
  mk.scale.z = 0.2;
  mk.pose.orientation.w = 1.0;

  visualization_msgs::Marker meshROS;
  meshROS.header.frame_id = "world";
  meshROS.header.stamp = ros::Time::now();
  meshROS.id = 0;
  meshROS.ns = "current_mesh";
  meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
  meshROS.color.a = 1.0;
  meshROS.scale.x = 5.0;
  meshROS.scale.y = 5.0;
  meshROS.scale.z = 6.0;
  meshROS.pose.orientation.w = 1.0;
  meshROS.mesh_resource = "file://";
  meshROS.mesh_resource += droneMesh;
  meshROS.mesh_use_embedded_materials = true;

  meshROS.action = visualization_msgs::Marker::DELETE;
  drone_pub_.publish(meshROS);
  mk.action = visualization_msgs::Marker::DELETE;
  drawFoV_pub_.publish(mk);

  if (list1.size() == 0) return;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(list1.size()); ++i) 
  {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }
  mk.action = visualization_msgs::Marker::ADD;
  drawFoV_pub_.publish(mk);

  // double yaw_mesh = yaw + 45.0 * M_PI / 180.0;
  double yaw_mesh = yaw;
  meshROS.pose.orientation.x = 0.0;
  meshROS.pose.orientation.y = 0.0;
  meshROS.pose.orientation.z = sin(0.5*yaw_mesh);
  meshROS.pose.orientation.w = cos(0.5*yaw_mesh);
  meshROS.pose.position.x = list1[0](0);
  meshROS.pose.position.y = list1[0](1);
  meshROS.pose.position.z = list1[0](2);
  meshROS.action = visualization_msgs::Marker::ADD;
  drone_pub_.publish(meshROS);
}

void PlanningVisualization::publishTravelTraj(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color, int id)
{
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = "world";
  // path_msg.header.seq = id;

  for (const auto& point : path)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = point(0);
    pose.pose.position.y = point(1);
    pose.pose.position.z = point(2);

    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;

    pose.header.stamp=ros::Time::now();;
    pose.header.frame_id="world";

    path_msg.poses.push_back(pose);
  }

  traveltraj_pub_.publish(path_msg);
}

void PlanningVisualization::publishVisiblePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& currentCloud, int id)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_pred;
  cloud_pred = *currentCloud;

  cloud_pred.width = cloud_pred.points.size();
  cloud_pred.height = 1;
  cloud_pred.is_dense = true;
  cloud_pred.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_pred, cloud_msg);
  visible_pub_.publish(cloud_msg);

  // sensor_msgs::PointCloud2 emptyPointCloudMsg;
  // emptyPointCloudMsg.header.frame_id = "world";
  // visible_pub_.publish(emptyPointCloudMsg);
}

void PlanningVisualization::publishCheckNeigh(Eigen::Vector3d& checkPoint, const pcl::PointCloud<pcl::PointXYZ>& checkNeigh, Eigen::MatrixXi& edgeMat)
{
  double radius = 1.5;
  
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 1;
  mk.ns = "rosa_debug_ckpt";
  mk.type = visualization_msgs::Marker::SPHERE;
  mk.color.r = 0.0;
  mk.color.g = 0.0;
  mk.color.b = 1.0;
  mk.color.a = 0.8;
  mk.scale.x = radius;
  mk.scale.y = radius;
  mk.scale.z = radius;
  mk.pose.orientation.w = 1.0;
  mk.pose.position.x = checkPoint(0);
  mk.pose.position.y = checkPoint(1);
  mk.pose.position.z = checkPoint(2);

  checkPoint_pub_.publish(mk);

  pcl::PointCloud<pcl::PointXYZ> cloud_pred;
  cloud_pred = checkNeigh;
  cloud_pred.width = cloud_pred.points.size();
  cloud_pred.height = 1;
  cloud_pred.is_dense = true;
  cloud_pred.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_pred, cloud_msg);
  checkNeigh_pub_.publish(cloud_msg);

  int counter_vpg = 0;
  visualization_msgs::Marker lines;
  lines.header.frame_id = "world";
  lines.header.stamp = ros::Time::now();
  lines.id = counter_vpg;
  lines.ns = "rosa_debug_adj";
  lines.type = visualization_msgs::Marker::LINE_LIST;
  lines.action = visualization_msgs::Marker::ADD;

  lines.pose.orientation.w = 1.0;
  lines.scale.x = 0.05;

  lines.color.r = 0.2;
  lines.color.g = 0.0;
  lines.color.b = 0.8;
  lines.color.a = 0.8;

  geometry_msgs::Point p1, p2;
  for (int i=0; i<(int)edgeMat.rows(); ++i)
  {
    for (int j=0; j<(int)edgeMat.cols(); ++j)
    {
      if (edgeMat(i,j) == 1 && i >= j)
      {
        p1.x = checkNeigh.points[i].x; p1.y = checkNeigh.points[i].y; p1.z = checkNeigh.points[i].z;
        p2.x = checkNeigh.points[j].x; p2.y = checkNeigh.points[j].y; p2.z = checkNeigh.points[j].z;
        lines.points.push_back(p1);
        lines.points.push_back(p2);
      }
    }
  }

  checkAdj_pub_.publish(lines);
}

void PlanningVisualization::publishCheckCP(Eigen::Vector3d& CPPoint, Eigen::Vector3d& CPDir, Eigen::Vector3d& checkRP, const pcl::PointCloud<pcl::PointXYZ>& CPPts, const pcl::PointCloud<pcl::PointXYZ>& CPPtsCluster)
{
  double scale = 3.0;
  visualization_msgs::Marker nm;
  nm.header.frame_id = "world";
  nm.header.stamp = ros::Time::now();
  nm.id = 1;
  nm.ns = "rosa_debug_CPdir";
  nm.type = visualization_msgs::Marker::ARROW;

  nm.pose.orientation.w = 1.0;
  nm.scale.x = 0.2;
  nm.scale.y = 0.3;
  nm.scale.z = 0.2;

  geometry_msgs::Point pt_;
  pt_.x = CPPoint(0);
  pt_.y = CPPoint(1);
  pt_.z = CPPoint(2);
  nm.points.push_back(pt_);

  pt_.x = CPPoint(0) + scale*CPDir(0);
  pt_.y = CPPoint(1) + scale*CPDir(1);
  pt_.z = CPPoint(2) + scale*CPDir(2);
  nm.points.push_back(pt_);

  nm.color.r = 0.7;
  nm.color.g = 0.2;
  nm.color.b = 0.4;
  nm.color.a = 1.0;
  checkCPdir_pub_.publish(nm);

  double radius = 1.5;
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 1;
  mk.ns = "rosa_debug_ckRP";
  mk.type = visualization_msgs::Marker::CUBE;
  mk.color.r = 1.0;
  mk.color.g = 0.5;
  mk.color.b = 0.2;
  mk.color.a = 0.8;
  mk.scale.x = radius;
  mk.scale.y = radius;
  mk.scale.z = radius;
  mk.pose.orientation.w = 1.0;
  mk.pose.position.x = checkRP(0);
  mk.pose.position.y = checkRP(1);
  mk.pose.position.z = checkRP(2);
  checkRP_pub_.publish(mk);

  pcl::PointCloud<pcl::PointXYZ> cloud_pred;
  cloud_pred = CPPts;
  cloud_pred.width = cloud_pred.points.size();
  cloud_pred.height = 1;
  cloud_pred.is_dense = true;
  cloud_pred.header.frame_id = "world";
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_pred, cloud_msg);
  checkCPpts_pub_.publish(cloud_msg);

  pcl::PointCloud<pcl::PointXYZ> cloud_cluster;
  cloud_cluster = CPPtsCluster;
  cloud_cluster.width = cloud_cluster.points.size();
  cloud_cluster.height = 1;
  cloud_cluster.is_dense = true;
  cloud_cluster.header.frame_id = "world";
  sensor_msgs::PointCloud2 cloud_msg_cluster;
  pcl::toROSMsg(cloud_cluster, cloud_msg_cluster);
  checkCPptsCluster_pub_.publish(cloud_msg_cluster);
}

void PlanningVisualization::publishUpdatesPose(pcl::PointCloud<pcl::PointXYZ>& visCloud, vector<vector<Eigen::Vector3d>>& list1, vector<vector<Eigen::Vector3d>>& list2, vector<double>& yaws)
{
  visualization_msgs::MarkerArray vps;
  int counter = 0;
  for (int j=0; j<(int)list1.size(); ++j)
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = counter;
    mk.ns = "current_vps";
    mk.type = visualization_msgs::Marker::LINE_LIST;
    mk.color.r = 0.0;
    mk.color.g = 0.0;
    mk.color.b = 0.0;
    mk.color.a = 1.0;
    mk.scale.x = 0.2;
    mk.scale.y = 0.2;
    mk.scale.z = 0.2;
    mk.pose.orientation.w = 1.0;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(list1[j].size()); ++i) 
    {
      pt.x = list1[j][i](0);
      pt.y = list1[j][i](1);
      pt.z = list1[j][i](2);
      mk.points.push_back(pt);

      pt.x = list2[j][i](0);
      pt.y = list2[j][i](1);
      pt.z = list2[j][i](2);
      mk.points.push_back(pt);
    }

    vps.markers.push_back(mk);
    counter++;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud_pred;
  cloud_pred = visCloud;

  cloud_pred.width = cloud_pred.points.size();
  cloud_pred.height = 1;
  cloud_pred.is_dense = true;
  cloud_pred.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_pred, cloud_msg);

  currentPose_pub_.publish(vps);
  currentVoxels_pub_.publish(cloud_msg);

  ros::Duration(3.0).sleep();
}

void PlanningVisualization::publishVpsCHull(std::map<int, std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>>& vpHull, vector<Eigen::Vector3d>& hamiPath)
{
  visualization_msgs::MarkerArray vpHulls;
  srand((int)time(0));
  int red, blue, green;
  int counter = 0;
  for (const auto& pair:vpHull)
  {
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> hull = pair.second;
    red = rand()%255;
    blue = rand()%255;
    green = rand()%255;

    if (hull.size() == 0) continue;
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "vps_hull";
    marker.id = counter;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    for (const auto& edge : hull)
    {
      geometry_msgs::Point p1, p2;
      p1.x = edge.first[0]; p1.y = edge.first[1]; p1.z = edge.first[2];
      p2.x = edge.second[0]; p2.y = edge.second[1]; p2.z = edge.second[2];
      marker.points.push_back(p1);
      marker.points.push_back(p2);
    }
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.color.a = 0.5;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    // vpHulls.markers.push_back(marker);
    counter++;
  }

  visualization_msgs::Marker path;
  path.header.frame_id = "world";
  path.header.stamp = ros::Time();
  path.ns = "hamiPath";
  path.id = 0;
  path.type = visualization_msgs::Marker::LINE_LIST;

  for (int i=0; i<(int)hamiPath.size()-1; ++i)
  {
    geometry_msgs::Point p1, p2;
    p1.x = hamiPath[i](0); p1.y = hamiPath[i](1); p1.z = hamiPath[i](2);
    p2.x = hamiPath[i+1](0); p2.y = hamiPath[i+1](1); p2.z = hamiPath[i+1](2);
    path.points.push_back(p1);
    path.points.push_back(p2);
  }

  path.pose.orientation.w = 1.0;
  path.scale.x = 0.5;
  path.color.a = 1.0;
  path.color.r = 1.0;
  path.color.g = 0.0;
  path.color.b = 0.0;
  vpHulls.markers.push_back(path);

  sub_vps_hull_pub_.publish(vpHulls);
}

void PlanningVisualization::publishInitVps(pcl::PointCloud<pcl::PointNormal>::Ptr& init_vps)
{
  visualization_msgs::MarkerArray init_vps_markers;
  int counter = 0;
  double scale = 3.0;

  for (int i=0; i<(int)init_vps->points.size(); ++i)
  {
    visualization_msgs::Marker nm;
    nm.header.frame_id = "world";
    nm.header.stamp = ros::Time::now();
    nm.id = counter;
    nm.ns = "init_vps_dir";
    nm.type = visualization_msgs::Marker::ARROW;
    nm.action = visualization_msgs::Marker::ADD;

    nm.pose.orientation.w = 1.0;
    nm.scale.x = 0.2;
    nm.scale.y = 0.3;
    nm.scale.z = 0.2;

    geometry_msgs::Point pt_;
    pt_.x = init_vps->points[i].x;
    pt_.y = init_vps->points[i].y;
    pt_.z = init_vps->points[i].z;
    nm.points.push_back(pt_);

    pt_.x = init_vps->points[i].x + scale*init_vps->points[i].normal_x;
    pt_.y = init_vps->points[i].y + scale*init_vps->points[i].normal_y;
    pt_.z = init_vps->points[i].z + scale*init_vps->points[i].normal_z;

    nm.points.push_back(pt_);

    nm.color.r = 0.0;
    nm.color.g = 0.4;
    nm.color.b = 0.9;
    nm.color.a = 0.8;

    init_vps_markers.markers.push_back(nm);
    counter++;

    visualization_msgs::Marker pos;
    pos.header.frame_id = "world";
    pos.header.stamp = ros::Time::now();
    pos.id = counter;
    pos.ns = "init_vps_pos";
    pos.type = visualization_msgs::Marker::SPHERE;
    pos.color.r = 0.0;
    pos.color.g = 0.0;
    pos.color.b = 0.0;
    pos.color.a = 1.0;
    pos.scale.x = 0.5;
    pos.scale.y = 0.5;
    pos.scale.z = 0.5;
    pos.pose.orientation.w = 1.0;
    pos.pose.position.x = init_vps->points[i].x;
    pos.pose.position.y = init_vps->points[i].y;
    pos.pose.position.z = init_vps->points[i].z;

    init_vps_markers.markers.push_back(pos);
    counter++;
  }

  init_vps_pub_.publish(init_vps_markers);
}

void PlanningVisualization::publishOptArea(vector<pcl::PointCloud<pcl::PointXYZ>>& optArea)
{
  srand((int)time(0));
  pcl::PointCloud<pcl::PointXYZRGB> colorCloud;
  pcl::PointXYZRGB p;
  int red, blue, green;
  for (int i=0; i<(int)optArea.size(); ++i)
  {
    red = rand()%255;
    blue = rand()%255;
    green = rand()%255;
    for (int j=0; j<(int)optArea[i].points.size(); ++j)
    {
      p.x = optArea[i].points[j].x;
      p.y = optArea[i].points[j].y;
      p.z = optArea[i].points[j].z;
      // color
      p.r = red;
      p.g = green;
      p.b = blue;
      colorCloud.points.push_back(p);
    }
  }

  colorCloud.width = colorCloud.points.size();
  colorCloud.height = 1;
  colorCloud.is_dense = true;
  colorCloud.header.frame_id = "world";

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(colorCloud, cloud_msg);
  optArea_pub_.publish(cloud_msg);
}

void PlanningVisualization::publishLocalRegion(pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& env_cloud, Eigen::Vector3d& min_bound, Eigen::Vector3d& max_bound)
{
  if ((int)target_cloud->points.size() > 0)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud_tar;
    for (auto p:target_cloud->points)
    cloud_tar.points.push_back(p);

    cloud_tar.width = cloud_tar.points.size();
    cloud_tar.height = 1;
    cloud_tar.is_dense = true;
    cloud_tar.header.frame_id = "world";
    
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud_tar, cloud_msg);
    localTar_pub_.publish(cloud_msg);
  }

  if ((int)env_cloud->points.size() > 0)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud_env;
    for (auto p:env_cloud->points)
      cloud_env.points.push_back(p);
    
    cloud_env.width = cloud_env.points.size();
    cloud_env.height = 1;
    cloud_env.is_dense = true;
    cloud_env.header.frame_id = "world";

    sensor_msgs::PointCloud2 cloud_msg_env;
    pcl::toROSMsg(cloud_env, cloud_msg_env);
    localEnv_pub_.publish(cloud_msg_env);
  }

  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 0;
  mk.ns = "local_box";
  mk.type = visualization_msgs::Marker::CUBE;
  mk.color.r = 0.5;
  mk.color.g = 1.0;
  mk.color.b = 0.5;
  mk.color.a = 0.2;
  mk.scale.x = max_bound(0) - min_bound(0);
  mk.scale.y = max_bound(1) - min_bound(1);
  mk.scale.z = max_bound(2) - min_bound(2);
  mk.pose.position.x = (max_bound(0) + min_bound(0)) / 2.0;
  mk.pose.position.y = (max_bound(1) + min_bound(1)) / 2.0;
  mk.pose.position.z = (max_bound(2) + min_bound(2)) / 2.0;
  mk.pose.orientation.w = 1.0;

  localBox_pub_.publish(mk);

  return;
}

void PlanningVisualization::publishCurPath(const vector<Eigen::VectorXd>& local_path, const vector<vector<Eigen::Vector3d>>& list1, const vector<vector<Eigen::Vector3d>>& list2)
{
  double scale_line = 0.07;
  double scale_sphere = 0.15;
  double scale_wp = 0.03;
  
  if (!path_markers_.markers.empty()) {
    for (auto& marker : path_markers_.markers) {
      marker.action = visualization_msgs::Marker::DELETE;
    }
    local_pub_.publish(path_markers_);
    path_markers_.markers.clear();
  }

  // * path
  int counter = 0;
  visualization_msgs::MarkerArray path_markers;
  
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = counter;
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.color.r = 0.0;
  mk.color.g = 1.0;
  mk.color.b = 0.0;
  mk.color.a = 1.0;
  mk.scale.x = scale_line;
  mk.scale.y = scale_line;
  mk.scale.z = scale_line;
  mk.pose.orientation.w = 1.0;

  for (int i=0; i<(int)local_path.size()-1; ++i)
  {
    geometry_msgs::Point pt;
    pt.x = local_path[i](0);
    pt.y = local_path[i](1);
    pt.z = local_path[i](2);
    mk.points.push_back(pt);

    pt.x = local_path[i+1](0);
    pt.y = local_path[i+1](1);
    pt.z = local_path[i+1](2);
    mk.points.push_back(pt);
  }

  path_markers.markers.push_back(mk);
  counter++;

  visualization_msgs::Marker sp;
  sp.header.frame_id = "world";
  sp.header.stamp = ros::Time::now();
  sp.id = counter;
  sp.type = visualization_msgs::Marker::SPHERE_LIST;
  sp.color.r = 0.0;
  sp.color.g = 0.0;
  sp.color.b = 1.0;
  sp.color.a = 1.0;
  sp.scale.x = scale_sphere;
  sp.scale.y = scale_sphere;
  sp.scale.z = scale_sphere;
  sp.pose.orientation.w = 1.0;

  for (int i=0; i<(int)local_path.size(); ++i)
  {
    geometry_msgs::Point pt;
    pt.x = local_path[i](0);
    pt.y = local_path[i](1);
    pt.z = local_path[i](2);
    sp.points.push_back(pt);
  }

  path_markers.markers.push_back(sp);
  counter++;

  local_pub_.publish(path_markers);
  path_markers_ = path_markers;

  // * path order
  int counter_order = 0;
  visualization_msgs::MarkerArray orders;
  for (int i=0; i<(int)local_path.size(); ++i)
  {
    visualization_msgs::Marker text_marker;
    text_marker.header.frame_id = "world";
    text_marker.header.stamp = ros::Time::now();
    text_marker.id = counter_order;
    text_marker.ns = "local_path_order";
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.text = std::to_string(i);
    text_marker.pose.position.x = local_path[i](0);
    text_marker.pose.position.y = local_path[i](1);
    text_marker.pose.position.z = local_path[i](2);
    text_marker.scale.z = 4*scale_sphere;
    text_marker.color.r = 0.0;
    text_marker.color.g = 0.0;
    text_marker.color.b = 0.0;
    text_marker.color.a = 1.0;
    orders.markers.push_back(text_marker);
    counter_order++;
  }

  local_path_order_pub_.publish(orders);

  if (!vp_set_.markers.empty()) {
    for (auto& marker : vp_set_.markers) {
      marker.action = visualization_msgs::Marker::DELETE;
    }
    localVP_pub_.publish(vp_set_);
    vp_set_.markers.clear();
  }

  // * waypoints
  visualization_msgs::MarkerArray vp_set;
  int counter_wp = 0;
  for (int j=0; j<(int)list1.size(); ++j)
  {
    visualization_msgs::Marker wp;
    wp.header.frame_id = "world";
    wp.header.stamp = ros::Time::now();
    wp.id = counter_wp;
    wp.ns = "local_waypoints";
    wp.type = visualization_msgs::Marker::LINE_LIST;
    wp.color.r = j == 0? 1.0:0.0;
    wp.color.g = 0.0;
    wp.color.b = 0.0;
    wp.color.a = 1.0;
    wp.scale.x = scale_wp;
    wp.scale.y = scale_wp;
    wp.scale.z = scale_wp;
    wp.pose.orientation.w = 1.0;

    geometry_msgs::Point pt;
    for (int i = 0; i < (int)list1[j].size(); ++i) 
    {
      pt.x = list1[j][i](0);
      pt.y = list1[j][i](1);
      pt.z = list1[j][i](2);
      wp.points.push_back(pt);

      pt.x = list2[j][i](0);
      pt.y = list2[j][i](1);
      pt.z = list2[j][i](2);
      wp.points.push_back(pt);
    }

    vp_set.markers.push_back(wp);
    counter_wp++;
  }

  localVP_pub_.publish(vp_set);
  vp_set_ = vp_set;
}

void PlanningVisualization::publishLocalVST(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& input_actual)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_pred;
  for (auto p:input_cloud->points)
    cloud_pred.points.push_back(p);

  cloud_pred.width = cloud_pred.points.size();
  cloud_pred.height = 1;
  cloud_pred.is_dense = true;
  cloud_pred.header.frame_id = "world";

  pcl::PointCloud<pcl::PointXYZ> cloud_actual;
  for (auto p:input_actual->points)
    cloud_actual.points.push_back(p);
  
  cloud_actual.width = cloud_actual.points.size();
  cloud_actual.height = 1;
  cloud_actual.is_dense = true;
  cloud_actual.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 cloud_msg, cloud_actual_msg;
  pcl::toROSMsg(cloud_pred, cloud_msg);
  pcl::toROSMsg(cloud_actual, cloud_actual_msg);
  localVA_pub_.publish(cloud_msg);
  localVAct_pub_.publish(cloud_actual_msg);

  return;
}

void PlanningVisualization::publishSamples(pcl::PointCloud<pcl::PointXYZ>::Ptr& world_samples, pcl::PointCloud<pcl::PointXYZ>::Ptr& local_samples)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_pred;
  for (auto p:world_samples->points)
    cloud_pred.points.push_back(p);

  cloud_pred.width = cloud_pred.points.size();
  cloud_pred.height = 1;
  cloud_pred.is_dense = true;
  cloud_pred.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_pred, cloud_msg);
  worldsample_pub_.publish(cloud_msg);

  pcl::PointCloud<pcl::PointXYZ> cloud_occ;
  for (auto p:local_samples->points)
    cloud_occ.points.push_back(p);

  cloud_occ.width = cloud_occ.points.size();
  cloud_occ.height = 1;
  cloud_occ.is_dense = true;
  cloud_occ.header.frame_id = "world";

  sensor_msgs::PointCloud2 cloud_occ_msg;
  pcl::toROSMsg(cloud_occ, cloud_occ_msg);
  localsample_pub_.publish(cloud_occ_msg);
}

void PlanningVisualization::publishLocalAABB(const unordered_map<int, vector<Eigen::Vector3d>>& aabb, const unordered_map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>& aabb_obstacle)
{
  visualization_msgs::MarkerArray aabb_markers;
  int counter = 0;
  for (const auto& pair:aabb)
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = counter;
    mk.ns = "local_aabb";
    mk.type = visualization_msgs::Marker::CUBE;
    mk.color.r = red_list[pair.first];
    mk.color.g = green_list[pair.first];
    mk.color.b = blue_list[pair.first];
    mk.color.a = 0.4;
    mk.scale.x = 0.1;
    mk.scale.y = 0.1;
    mk.scale.z = 0.1;
    mk.pose.orientation.w = 1.0;

    vector<Eigen::Vector3d> aabb_pts = pair.second;
    mk.scale.x = aabb_pts[1](0) - aabb_pts[0](0);
    mk.scale.y = aabb_pts[1](1) - aabb_pts[0](1);
    mk.scale.z = aabb_pts[1](2) - aabb_pts[0](2);
    mk.pose.position.x = (aabb_pts[1](0) + aabb_pts[0](0)) / 2.0;
    mk.pose.position.y = (aabb_pts[1](1) + aabb_pts[0](1)) / 2.0;
    mk.pose.position.z = (aabb_pts[1](2) + aabb_pts[0](2)) / 2.0;
    mk.pose.orientation.w = 1.0;

    aabb_markers.markers.push_back(mk);
    counter++;
  }

  localAABB_pub_.publish(aabb_markers);

  pcl::PointCloud<pcl::PointXYZRGB> cloud_pred;
  for (const auto& pair:aabb_obstacle)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud = *(pair.second);
    for (auto p:cloud.points)
    {
      pcl::PointXYZRGB point;
      point.x = p.x;
      point.y = p.y;
      point.z = p.z;
      point.r = red_list[pair.first];
      point.g = green_list[pair.first];
      point.b = blue_list[pair.first];
      cloud_pred.points.push_back(point);
    }
  }

  cloud_pred.width = cloud_pred.points.size();
  cloud_pred.height = 1;
  cloud_pred.is_dense = true;
  cloud_pred.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_pred, cloud_msg);
  localAABBObstacle_pub_.publish(cloud_msg);
}

void PlanningVisualization::publishLocalVisST(const vector<Eigen::VectorXd>& vp, const vector<Eigen::Vector3d>& vp_st)
{
  int counter = 0;
  visualization_msgs::MarkerArray vis_st_lines_;

  for (int i=0; i<(int)vp.size(); ++i)
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = counter;
    mk.type = visualization_msgs::Marker::ARROW;
    mk.color.r = red_list[i];
    mk.color.g = green_list[i];
    mk.color.b = blue_list[i];
    mk.color.a = 1.0;
    mk.scale.x = 0.3;
    mk.scale.y = 0.5;
    mk.scale.z = 0.3;
    mk.pose.orientation.w = 1.0;

    geometry_msgs::Point pt;
    pt.x = vp[i](0);
    pt.y = vp[i](1);
    pt.z = vp[i](2);
    mk.points.push_back(pt);

    pt.x = vp_st[i](0);
    pt.y = vp_st[i](1);
    pt.z = vp_st[i](2);
    mk.points.push_back(pt);

    vis_st_lines_.markers.push_back(mk);
    counter++;
  }

  localInitVisST_pub_.publish(vis_st_lines_);
}

void PlanningVisualization::publishUpdatedCurPath(const vector<Eigen::VectorXd>& local_path, const vector<vector<Eigen::Vector3d>>& list1, const vector<vector<Eigen::Vector3d>>& list2)
{
  double scale_line = 0.07;
  double scale_sphere = 0.15;
  double scale_wp = 0.03;
  
  if (!path_markers_updated_.markers.empty()) {
    for (auto& marker : path_markers_updated_.markers) {
      marker.action = visualization_msgs::Marker::DELETE;
    }
    local_updated_pub_.publish(path_markers_updated_);
    path_markers_updated_.markers.clear();
  }

  // * path
  int counter = 0;
  visualization_msgs::MarkerArray path_markers;
  
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = counter;
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.color.r = red_list[50];
  mk.color.g = green_list[50];
  mk.color.b = blue_list[50];
  mk.color.a = 1.0;
  mk.scale.x = scale_line;
  mk.scale.y = scale_line;
  mk.scale.z = scale_line;
  mk.pose.orientation.w = 1.0;

  for (int i=0; i<(int)local_path.size()-1; ++i)
  {
    geometry_msgs::Point pt;
    pt.x = local_path[i](0);
    pt.y = local_path[i](1);
    pt.z = local_path[i](2);
    mk.points.push_back(pt);

    pt.x = local_path[i+1](0);
    pt.y = local_path[i+1](1);
    pt.z = local_path[i+1](2);
    mk.points.push_back(pt);
  }

  path_markers.markers.push_back(mk);
  counter++;

  visualization_msgs::Marker sp;
  sp.header.frame_id = "world";
  sp.header.stamp = ros::Time::now();
  sp.id = counter;
  sp.type = visualization_msgs::Marker::SPHERE_LIST;
  sp.color.r = red_list[40];
  sp.color.g = green_list[40];
  sp.color.b = blue_list[40];
  sp.color.a = 1.0;
  sp.scale.x = scale_sphere;
  sp.scale.y = scale_sphere;
  sp.scale.z = scale_sphere;
  sp.pose.orientation.w = 1.0;

  for (int i=0; i<(int)local_path.size(); ++i)
  {
    geometry_msgs::Point pt;
    pt.x = local_path[i](0);
    pt.y = local_path[i](1);
    pt.z = local_path[i](2);
    sp.points.push_back(pt);
  }

  path_markers.markers.push_back(sp);
  counter++;

  // * path order
  int counter_order = 0;
  visualization_msgs::MarkerArray orders;
  for (int i=0; i<(int)local_path.size(); ++i)
  {
    visualization_msgs::Marker text_marker;
    text_marker.header.frame_id = "world";
    text_marker.header.stamp = ros::Time::now();
    text_marker.id = counter_order;
    text_marker.ns = "local_updated_path_order";
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.text = std::to_string(i);
    text_marker.pose.position.x = local_path[i](0);
    text_marker.pose.position.y = local_path[i](1);
    text_marker.pose.position.z = local_path[i](2);
    text_marker.scale.z = 4*scale_sphere;
    text_marker.color.r = 1.0;
    text_marker.color.g = 0.0;
    text_marker.color.b = 0.0;
    text_marker.color.a = 1.0;
    orders.markers.push_back(text_marker);
    counter_order++;
  }

  local_updated_path_order_pub_.publish(orders);

  local_updated_pub_.publish(path_markers);
  path_markers_updated_ = path_markers;

  if (!vp_set_updated_.markers.empty()) {
    for (auto& marker : vp_set_updated_.markers) {
      marker.action = visualization_msgs::Marker::DELETE;
    }
    localVP_updated_pub_.publish(vp_set_updated_);
    vp_set_updated_.markers.clear();
  }

  // * waypoints
  visualization_msgs::MarkerArray vp_set;
  int counter_wp = 0;
  for (int j=0; j<(int)list1.size(); ++j)
  {
    visualization_msgs::Marker wp;
    wp.header.frame_id = "world";
    wp.header.stamp = ros::Time::now();
    wp.id = counter_wp;
    wp.ns = "local_updated_waypoints";
    wp.type = visualization_msgs::Marker::LINE_LIST;
    wp.color.r = j == 0? 1.0:red_list[18];
    wp.color.g = j == 0? 0.0:green_list[18];
    wp.color.b = j == 0? 0.0:blue_list[18];
    wp.color.a = 1.0;
    wp.scale.x = scale_wp;
    wp.scale.y = scale_wp;
    wp.scale.z = scale_wp;
    wp.pose.orientation.w = 1.0;

    geometry_msgs::Point pt;
    for (int i = 0; i < (int)list1[j].size(); ++i) 
    {
      pt.x = list1[j][i](0);
      pt.y = list1[j][i](1);
      pt.z = list1[j][i](2);
      wp.points.push_back(pt);

      pt.x = list2[j][i](0);
      pt.y = list2[j][i](1);
      pt.z = list2[j][i](2);
      wp.points.push_back(pt);
    }

    vp_set.markers.push_back(wp);
    counter_wp++;
  }

  localVP_updated_pub_.publish(vp_set);
  vp_set_updated_ = vp_set;
}

void PlanningVisualization::publishUpdatedVPFrame(const vector<Eigen::Vector3d>& vp_pos, const vector<Eigen::Matrix3d>& vp_frames)
{
  visualization_msgs::MarkerArray marker_array;
  int counter = 0;

  vector<double> red_distri = {1.0, 0.0, 0.0};
  vector<double> green_distri = {0.0, 1.0, 0.0};
  vector<double> blue_distri = {0.0, 0.0, 1.0};

  for (int i=0; i<(int)vp_pos.size(); ++i)
  {

    for (int j=0; j<3; ++j)
    {
      visualization_msgs::Marker mk;
      mk.header.frame_id = "world";
      mk.header.stamp = ros::Time::now();
      mk.id = counter;
      mk.ns = "vp_frame";
      mk.type = visualization_msgs::Marker::ARROW;
      mk.action = visualization_msgs::Marker::ADD;
      mk.color.r = red_distri[j];
      mk.color.g = green_distri[j];
      mk.color.b = blue_distri[j];
      mk.color.a = 1.0;
      mk.scale.x = 0.2;
      mk.scale.y = 0.3;
      mk.scale.z = 0.2;
      mk.pose.orientation.w = 1.0;

      geometry_msgs::Point pt;
      pt.x = vp_pos[i](0);
      pt.y = vp_pos[i](1);
      pt.z = vp_pos[i](2);
      mk.points.push_back(pt);

      Eigen::Vector3d axis = vp_frames[i].col(j);

      pt.x = vp_pos[i](0) + 2.0*axis(0);
      pt.y = vp_pos[i](1) + 2.0*axis(1);
      pt.z = vp_pos[i](2) + 2.0*axis(2);
      mk.points.push_back(pt);

      marker_array.markers.push_back(mk);
      counter++;
    }
  }

  local_updated_VP_frame_pub_.publish(marker_array);
}

void PlanningVisualization::publishLocalUsedGlobal(const vector<Eigen::VectorXd>& path, const vector<Eigen::VectorXd>& prior_path)
{
  double path_line_scale = 0.2, path_sphere_scale = 0.3;
  double prior_line_scale = 0.2, prior_sphere_scale = 0.4;
  
  visualization_msgs::MarkerArray path_marker;
  path_marker.markers.clear();

  local_used_g_path_pub_.publish(path_marker);

  if ((int)path.size() > 1)
  {
    visualization_msgs::Marker line;
    line.header.frame_id = "world";
    line.header.stamp = ros::Time::now();
    line.id = 0;
    line.ns = "local_used_global_line";
    line.type = visualization_msgs::Marker::LINE_LIST;
    line.color.r = 0.0;
    line.color.g = 1.0;
    line.color.b = 0.0;
    line.color.a = 0.5;
    line.scale.x = path_line_scale;
    line.scale.y = path_line_scale;
    line.scale.z = path_line_scale;
    line.pose.orientation.w = 1.0;
    for (int i=0; i<(int)path.size()-1; ++i)
    {
      geometry_msgs::Point p1, p2;
      p1.x = path[i](0);
      p1.y = path[i](1);
      p1.z = path[i](2);
      p2.x = path[i+1](0);
      p2.y = path[i+1](1);
      p2.z = path[i+1](2);
      line.points.push_back(p1);
      line.points.push_back(p2);
    }
    path_marker.markers.push_back(line);
  }

  visualization_msgs::Marker sphere;
  sphere.header.frame_id = "world";
  sphere.header.stamp = ros::Time::now();
  sphere.id = 1;
  sphere.ns = "local_used_global_sphere";
  sphere.type = visualization_msgs::Marker::SPHERE_LIST;
  sphere.scale.x = path_sphere_scale;
  sphere.scale.y = path_sphere_scale;
  sphere.scale.z = path_sphere_scale;
  sphere.color.r = 0.0;
  sphere.color.g = 0.0;
  sphere.color.b = 0.0;
  sphere.color.a = 1.0;
  sphere.pose.orientation.w = 1.0;

  visualization_msgs::Marker text_marker;
  text_marker.header.frame_id = "world";
  text_marker.header.stamp = ros::Time::now();
  text_marker.ns = "local_used_global_sphere_idx";
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.color.r = 1.0;
  text_marker.color.g = 0.0;
  text_marker.color.b = 0.0;
  text_marker.color.a = 1.0;
  text_marker.scale.z = 0.6;

  for (int i=1; i<(int)path.size(); ++i)
  {
    geometry_msgs::Point pt;
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    sphere.points.push_back(pt);

    text_marker.id = i;
    text_marker.pose.position = pt;
    text_marker.text = std::to_string(i);
    path_marker.markers.push_back(text_marker);
  }
  path_marker.markers.push_back(sphere);

  visualization_msgs::Marker start_cube;
  start_cube.header.frame_id = "world";
  start_cube.header.stamp = ros::Time::now();
  start_cube.id = 2;
  start_cube.ns = "local_used_global_start";
  start_cube.type = visualization_msgs::Marker::CUBE;
  start_cube.color.r = 0.0;
  start_cube.color.g = 0.5;
  start_cube.color.b = 0.5;
  start_cube.color.a = 1.0;
  start_cube.scale.x = path_sphere_scale;
  start_cube.scale.y = path_sphere_scale;
  start_cube.scale.z = path_sphere_scale;
  start_cube.pose.orientation.w = 1.0;
  start_cube.pose.position.x = path[0](0);
  start_cube.pose.position.y = path[0](1);
  start_cube.pose.position.z = path[0](2);
  path_marker.markers.push_back(start_cube);

  local_used_g_path_pub_.publish(path_marker);

  visualization_msgs::MarkerArray prior_path_marker;
  prior_path_marker.markers.clear();

  local_used_g_prior_pub_.publish(prior_path_marker);

  if ((int)prior_path.size() > 1)
  {
    visualization_msgs::Marker prior_line;
    prior_line.header.frame_id = "world";
    prior_line.header.stamp = ros::Time::now();
    prior_line.id = 0;
    prior_line.ns = "local_used_global_prior_line";
    prior_line.type = visualization_msgs::Marker::LINE_LIST;
    prior_line.color.r = 1.0;
    prior_line.color.g = 0.0;
    prior_line.color.b = 0.0;
    prior_line.color.a = 0.7;
    prior_line.scale.x = prior_line_scale;
    prior_line.scale.y = prior_line_scale;
    prior_line.scale.z = prior_line_scale;
    prior_line.pose.orientation.w = 1.0;
    for (int i=0; i<(int)prior_path.size()-1; ++i)
    {
      geometry_msgs::Point p1, p2;
      p1.x = prior_path[i](0);
      p1.y = prior_path[i](1);
      p1.z = prior_path[i](2);
      p2.x = prior_path[i+1](0);
      p2.y = prior_path[i+1](1);
      p2.z = prior_path[i+1](2);
      prior_line.points.push_back(p1);
      prior_line.points.push_back(p2);
    }
    prior_path_marker.markers.push_back(prior_line);
  }

  visualization_msgs::Marker prior_sphere;
  prior_sphere.header.frame_id = "world";
  prior_sphere.header.stamp = ros::Time::now();
  prior_sphere.id = 1;
  prior_sphere.ns = "local_used_global_prior_sphere";
  prior_sphere.type = visualization_msgs::Marker::SPHERE_LIST;
  prior_sphere.color.r = 0.0;
  prior_sphere.color.g = 0.0;
  prior_sphere.color.b = 1.0;
  prior_sphere.color.a = 1.0;
  prior_sphere.scale.x = prior_sphere_scale;
  prior_sphere.scale.y = prior_sphere_scale;
  prior_sphere.scale.z = prior_sphere_scale;
  prior_sphere.pose.orientation.w = 1.0;
  for (int i=0; i<(int)prior_path.size(); ++i)
  {
    geometry_msgs::Point pt;
    pt.x = prior_path[i](0);
    pt.y = prior_path[i](1);
    pt.z = prior_path[i](2);
    prior_sphere.points.push_back(pt);
  }
  prior_path_marker.markers.push_back(prior_sphere);

  local_used_g_prior_pub_.publish(prior_path_marker);

  return;
}

void PlanningVisualization::publishLocalUsedGlobalNextSub(const vector<Eigen::Vector3d>& inliers, const vector<Eigen::VectorXd>& vps)
{
  double inlier_scale = 1.0;

  visualization_msgs::Marker inliers_marker;
  inliers_marker.header.frame_id = "world";
  inliers_marker.header.stamp = ros::Time::now();
  inliers_marker.id = 0;
  inliers_marker.ns = "local_used_global_inliers";
  inliers_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  inliers_marker.color.r = 0.4;
  inliers_marker.color.g = 0.7;
  inliers_marker.color.b = 1.0;
  inliers_marker.color.a = 1.0;
  inliers_marker.scale.x = inlier_scale;
  inliers_marker.scale.y = inlier_scale;
  inliers_marker.scale.z = inlier_scale;
  inliers_marker.pose.orientation.w = 1.0;

  for (int i=0; i<(int)inliers.size(); ++i)
  {
    geometry_msgs::Point pt;
    pt.x = inliers[i](0);
    pt.y = inliers[i](1);
    pt.z = inliers[i](2);
    inliers_marker.points.push_back(pt);
  }

  local_used_g_nex_sub_inliers_pub_.publish(inliers_marker);

  double arrow_length = 1.5;
  visualization_msgs::MarkerArray vps_set;
  int counter = 0;

  for (int i=0; i<(int)vps.size(); ++i)
  {
    Eigen::VectorXd vp = vps[i];
    Eigen::Vector3d dir;
    dir(0) = cos(vp(3)) * cos(vp(4));
    dir(1) = cos(vp(3)) * sin(vp(4));
    dir(2) = sin(vp(3));

    visualization_msgs::Marker vp_marker;
    vp_marker.header.frame_id = "world";
    vp_marker.header.stamp = ros::Time::now();
    vp_marker.id = counter;
    vp_marker.ns = "local_used_global_next_sub_vp";
    vp_marker.type = visualization_msgs::Marker::ARROW;
    vp_marker.action = visualization_msgs::Marker::ADD;
    vp_marker.color.r = 0.4;
    vp_marker.color.g = 0.7;
    vp_marker.color.b = 1.0;
    vp_marker.color.a = 1.0;
    vp_marker.scale.x = 0.2;
    vp_marker.scale.y = 0.3;
    vp_marker.scale.z = 0.2;
    vp_marker.pose.orientation.w = 1.0;

    geometry_msgs::Point pt;
    pt.x = vp(0);
    pt.y = vp(1);
    pt.z = vp(2);
    vp_marker.points.push_back(pt);

    pt.x = vp(0) + arrow_length * dir(0);
    pt.y = vp(1) + arrow_length * dir(1);
    pt.z = vp(2) + arrow_length * dir(2);
    vp_marker.points.push_back(pt);

    counter++;

    vps_set.markers.push_back(vp_marker);
  }

  local_used_g_next_sub_vps_pub_.publish(vps_set);

  return;
}

void PlanningVisualization::publishPredMesh(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& faces)
{
  visualization_msgs::Marker mesh_marker;
  mesh_marker.header.frame_id = "world";
  mesh_marker.header.stamp = ros::Time::now();
  mesh_marker.ns = "mesh";
  mesh_marker.id = 0;
  mesh_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  mesh_marker.pose.orientation.w = 1.0;
  mesh_marker.scale.x = 1.0;
  mesh_marker.scale.y = 1.0;
  mesh_marker.scale.z = 1.0;
  mesh_marker.color.a = 0.5;
  mesh_marker.color.r = 0.4;
  mesh_marker.color.g = 0.3;
  mesh_marker.color.b = 0.5;

  mesh_marker.action = visualization_msgs::Marker::DELETEALL;
  pred_mesh_pub_.publish(mesh_marker);

  for (int i = 0; i < faces.rows(); ++i)
  {
      for (int j = 0; j < faces.cols(); ++j)
      {
          int idx = faces(i, j);
          geometry_msgs::Point p;
          p.x = vertices(idx, 0);
          p.y = vertices(idx, 1);
          p.z = vertices(idx, 2);
          mesh_marker.points.push_back(p);
      }
  }

  mesh_marker.action = visualization_msgs::Marker::ADD;
  pred_mesh_pub_.publish(mesh_marker);
}

void PlanningVisualization::publishGlobalStart(const Eigen::VectorXd& start_)
{
  Eigen::Vector3d start_pos = start_.head(3);
  double pitch = start_(3), yaw = start_(4);

  Eigen::Vector3d vec;
  vec(0) = cos(pitch) * cos(yaw);
  vec(1) = cos(pitch) * sin(yaw);
  vec(2) = sin(pitch);

  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 0;
  mk.type = visualization_msgs::Marker::ARROW;
  mk.action = visualization_msgs::Marker::ADD;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.color.a = 1.0;
  mk.scale.x = 0.3;
  mk.scale.y = 0.5;
  mk.scale.z = 0.3;
  mk.pose.orientation.w = 1.0;
  geometry_msgs::Point pt;
  pt.x = start_pos(0);
  pt.y = start_pos(1);
  pt.z = start_pos(2);
  mk.points.push_back(pt);

  pt.x = start_pos(0) + 2.0*vec(0);
  pt.y = start_pos(1) + 2.0*vec(1);
  pt.z = start_pos(2) + 2.0*vec(2);
  mk.points.push_back(pt);

  marker_array.markers.push_back(mk);

  visualization_msgs::Marker pos;
  pos.header.frame_id = "world";
  pos.header.stamp = ros::Time::now();
  pos.id = 1;
  pos.type = visualization_msgs::Marker::SPHERE;
  pos.action = visualization_msgs::Marker::ADD;
  pos.color.r = 0.0;
  pos.color.g = 0.0;
  pos.color.b = 0.0;
  pos.color.a = 1.0;
  pos.scale.x = 0.5;
  pos.scale.y = 0.5;
  pos.scale.z = 0.5;
  pos.pose.orientation.w = 1.0;
  pos.pose.position.x = start_pos(0);
  pos.pose.position.y = start_pos(1);
  pos.pose.position.z = start_pos(2);

  marker_array.markers.push_back(pos);

  global_start_pub_.publish(marker_array);
}

void PlanningVisualization::publishExecPart(const vector<Eigen::VectorXd>& path)
{
  double sphere_scale = 0.25;
  visualization_msgs::MarkerArray part;

  visualization_msgs::Marker wp;
  wp.header.frame_id = "world";
  wp.header.stamp = ros::Time::now();
  wp.id = 0;
  wp.ns = "exec_path_wp";
  wp.type = visualization_msgs::Marker::SPHERE_LIST;
  wp.color.r = 1.0;
  wp.color.g = 0.0;
  wp.color.b = 0.0;
  wp.color.a = 1.0;
  wp.scale.x = sphere_scale;
  wp.scale.y = sphere_scale;
  wp.scale.z = sphere_scale;
  wp.pose.orientation.w = 1.0;

  for (int i=0; i<(int)path.size(); ++i)
  {
    geometry_msgs::Point pt;
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    wp.points.push_back(pt);
  }
  part.markers.push_back(wp);

  // if ((int)path.size() > 1)
  // {
  //   double line_scale = 0.1;
  //   visualization_msgs::Marker line;
  //   line.header.frame_id = "world";
  //   line.header.stamp = ros::Time::now();
  //   line.id = 1;
  //   line.ns = "exec_path_line";
  //   line.type = visualization_msgs::Marker::LINE_LIST;
  //   line.color.r = 0.1;
  //   line.color.g = 0.1;
  //   line.color.b = 0.1;
  //   line.color.a = 0.8;
  //   line.scale.x = line_scale;
  //   line.scale.y = line_scale;
  //   line.scale.z = line_scale;
  //   line.pose.orientation.w = 1.0;

  //   for (int i=0; i<(int)path.size()-1; ++i)
  //   {
  //     geometry_msgs::Point pt;
  //     pt.x = path[i](0);
  //     pt.y = path[i](1);
  //     pt.z = path[i](2);
  //     line.points.push_back(pt);

  //     pt.x = path[i+1](0);
  //     pt.y = path[i+1](1);
  //     pt.z = path[i+1](2);
  //     line.points.push_back(pt);
  //   }
  //   part.markers.push_back(line);
  // }

  exec_path_pub_.publish(part);
}

void PlanningVisualization::publishVPBox(const Eigen::Vector3d& box_min, const Eigen::Vector3d& box_max)
{
  if (box_min == box_max)
    return;
  
  visualization_msgs::MarkerArray marker_array;

  // * Edge of Mapping Range
  visualization_msgs::Marker line_marker;
  line_marker.header.frame_id = "world";
  line_marker.header.stamp = ros::Time::now();
  line_marker.ns = "viewpoints_box";
  line_marker.id = 0;
  line_marker.type = visualization_msgs::Marker::LINE_LIST;
  line_marker.action = visualization_msgs::Marker::ADD;
  line_marker.pose.orientation.w = 1.0;
  line_marker.scale.x = 0.5;
  line_marker.color.a = 1.0;
  line_marker.color.r = 0.0;
  line_marker.color.g = 0.0;
  line_marker.color.b = 0.0;

  geometry_msgs::Point p[8];
  p[0].x = box_min.x(); p[0].y = box_min.y(); p[0].z = box_min.z();
  p[1].x = box_max.x(); p[1].y = box_min.y(); p[1].z = box_min.z();
  p[2].x = box_max.x(); p[2].y = box_max.y(); p[2].z = box_min.z();
  p[3].x = box_min.x(); p[3].y = box_max.y(); p[3].z = box_min.z();
  p[4].x = box_min.x(); p[4].y = box_min.y(); p[4].z = box_max.z();
  p[5].x = box_max.x(); p[5].y = box_min.y(); p[5].z = box_max.z();
  p[6].x = box_max.x(); p[6].y = box_max.y(); p[6].z = box_max.z();
  p[7].x = box_min.x(); p[7].y = box_max.y(); p[7].z = box_max.z();

  vector<geometry_msgs::Point> points = 
  {
    p[0], p[1], p[1], p[2], p[2], p[3], p[3], p[0],
    p[4], p[5], p[5], p[6], p[6], p[7], p[7], p[4],
    p[0], p[4], p[1], p[5], p[2], p[6], p[3], p[7]
  };

  line_marker.points.insert(line_marker.points.end(), points.begin(), points.end());
  marker_array.markers.push_back(line_marker);

  // * Cube of Mapping Range
  visualization_msgs::Marker cube_marker;
  cube_marker.header.frame_id = "world";
  cube_marker.header.stamp = ros::Time::now();
  cube_marker.ns = "viewpoints_box";
  cube_marker.id = 1;
  cube_marker.type = visualization_msgs::Marker::CUBE;
  cube_marker.action = visualization_msgs::Marker::ADD;
  cube_marker.pose.position.x = 0.5 * (box_min.x() + box_max.x());
  cube_marker.pose.position.y = 0.5 * (box_min.y() + box_max.y());
  cube_marker.pose.position.z = 0.5 * (box_min.z() + box_max.z());
  cube_marker.pose.orientation.w = 1.0;
  cube_marker.scale.x = box_max.x() - box_min.x();
  cube_marker.scale.y = box_max.y() - box_min.y();
  cube_marker.scale.z = box_max.z() - box_min.z();

  cube_marker.color.r = 0.0;
  cube_marker.color.g = 0.5;
  cube_marker.color.b = 1.0;
  cube_marker.color.a = 0.3;

  marker_array.markers.push_back(cube_marker);

  vp_box_pub_.publish(marker_array);

  return;
}

void PlanningVisualization::publishFOVHRep(Eigen::Matrix<double, 5, 4>& hPolys)
{
  Eigen::Matrix3Xd mesh(3, 0), curTris(3, 0), oldTris(3, 0);
  Eigen::MatrixX4d h_frus = hPolys;

  oldTris = mesh;
  Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vPoly;
  geo_utils::enumerateVs(h_frus, vPoly);

  quickhull::QuickHull<double> tinyQH;
  const auto polyHull = tinyQH.getConvexHull(vPoly.data(), vPoly.cols(), false, true);
  const auto &idxBuffer = polyHull.getIndexBuffer();
  int hNum = idxBuffer.size() / 3;

  curTris.resize(3, hNum * 3);
  for (int i = 0; i < hNum * 3; i++)
  {
    curTris.col(i) = vPoly.col(idxBuffer[i]);
  }
  mesh.resize(3, oldTris.cols() + curTris.cols());
  mesh.leftCols(oldTris.cols()) = oldTris;
  mesh.rightCols(curTris.cols()) = curTris;

  visualization_msgs::Marker meshMarker, edgeMarker;

  meshMarker.id = 0;
  meshMarker.header.stamp = ros::Time::now();
  meshMarker.header.frame_id = "world";
  meshMarker.pose.orientation.w = 1.00;
  meshMarker.action = visualization_msgs::Marker::ADD;
  meshMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  meshMarker.ns = "fov_h_mesh";
  meshMarker.color.r = 0.00;
  meshMarker.color.g = 0.00;
  meshMarker.color.b = 1.00;
  meshMarker.color.a = 0.2;
  meshMarker.scale.x = 1.0;
  meshMarker.scale.y = 1.0;
  meshMarker.scale.z = 1.0;

  edgeMarker = meshMarker;
  edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
  edgeMarker.ns = "fov_h_edge";
  edgeMarker.color.r = 0.00;
  edgeMarker.color.g = 1.00;
  edgeMarker.color.b = 1.00;
  edgeMarker.color.a = 1.00;
  edgeMarker.scale.x = 0.04;

  geometry_msgs::Point point;

  int ptnum = mesh.cols();

  for (int i = 0; i < ptnum; i++)
  {
    point.x = mesh(0, i);
    point.y = mesh(1, i);
    point.z = mesh(2, i);
    meshMarker.points.push_back(point);
  }

  for (int i = 0; i < ptnum / 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      point.x = mesh(0, 3 * i + j);
      point.y = mesh(1, 3 * i + j);
      point.z = mesh(2, 3 * i + j);
      edgeMarker.points.push_back(point);
      point.x = mesh(0, 3 * i + (j + 1) % 3);
      point.y = mesh(1, 3 * i + (j + 1) % 3);
      point.z = mesh(2, 3 * i + (j + 1) % 3);
      edgeMarker.points.push_back(point);
    }
  }

  fov_H_mesh_pub_.publish(meshMarker);
  fov_H_edge_pub_.publish(edgeMarker);

  return;
}

void PlanningVisualization::publishSampleRegions(const vector<vector<double>>& sectors)
{
  visualization_msgs::MarkerArray sample_regions;
  for (size_t idx = 0; idx < sectors.size(); ++idx) {
    const auto& sector = sectors[idx];
    if (sector.size() != 9) {
        ROS_WARN("Invalid sector parameters. Expected 9 values, got %zu.", sector.size());
        continue;
    }

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "sectors";
    marker.id = idx;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0;

    marker.color.r = red_list[idx];
    marker.color.g = green_list[idx];
    marker.color.b = blue_list[idx];
    marker.color.a = 0.5;

    geometry_msgs::Point start;
    start.x = sector[0]; start.y = sector[1]; start.z = sector[2];

    tf2::Vector3 dir_vec(sector[3], sector[4], sector[5]);
    dir_vec.normalize();

    // add arrow marker
    visualization_msgs::Marker arrow_marker;
    arrow_marker.header.frame_id = "world";
    arrow_marker.header.stamp = ros::Time::now();
    arrow_marker.ns = "sectors_dir";
    arrow_marker.id = idx + 1000;
    arrow_marker.type = visualization_msgs::Marker::ARROW;
    arrow_marker.action = visualization_msgs::Marker::ADD;
    arrow_marker.pose.orientation.w = 1.0;
    arrow_marker.scale.x = 0.1;
    arrow_marker.scale.y = 0.2;
    arrow_marker.scale.z = 0.2;
    arrow_marker.color.r = red_list[idx];
    arrow_marker.color.g = green_list[idx];
    arrow_marker.color.b = blue_list[idx];
    arrow_marker.color.a = 1.0;
    geometry_msgs::Point arrow_start;
    arrow_start.x = start.x; arrow_start.y = start.y; arrow_start.z = start.z;
    arrow_marker.points.push_back(arrow_start);
    geometry_msgs::Point arrow_end;
    arrow_end.x = start.x + 2.0 * dir_vec.x();
    arrow_end.y = start.y + 2.0 * dir_vec.y();
    arrow_end.z = start.z + 2.0 * dir_vec.z();
    arrow_marker.points.push_back(arrow_end);
    sample_regions.markers.push_back(arrow_marker);
    
    tf2::Vector3 x_axis(1, 0, 0);
    tf2::Vector3 rot_axis = x_axis.cross(dir_vec);

    double rot_angle = acos(x_axis.dot(dir_vec));

    tf2::Quaternion q;
    if (rot_axis.length() > 1e-6) { 
        rot_axis.normalize();
        q.setRotation(rot_axis, rot_angle);
        
        tf2::Matrix3x3 rot(q);
        double roll, pitch, yaw;
        rot.getRPY(roll, pitch, yaw);
        
        q.setRPY(0.0, pitch, yaw);
    } else {
        if (dir_vec.z() < 0) {
            q.setRotation(tf2::Vector3(1, 0, 0), M_PI);
        } else {
            q = tf2::Quaternion::getIdentity();
        }
    }

    tf2::Matrix3x3 rot(q);

    float r_min = sector[6];
    float r_max = sector[7];
    float angle_width = sector[8];

    std::vector<geometry_msgs::Point> inner_points, outer_points;
    float angle_step = 0.01;
    for (float theta = -angle_width/2; theta <= angle_width/2; theta += angle_step) 
    {
      double local_x = cos(theta);
      double local_y = sin(theta);
      
      tf2::Vector3 p_inner_local = r_min * tf2::Vector3(local_x, local_y, 0);
      tf2::Vector3 p_inner_world = rot * p_inner_local + tf2::Vector3(start.x, start.y, start.z);
      
      tf2::Vector3 p_outer_local = r_max * tf2::Vector3(local_x, local_y, 0);
      tf2::Vector3 p_outer_world = rot * p_outer_local + tf2::Vector3(start.x, start.y, start.z);
  
      geometry_msgs::Point p_inner, p_outer;
      p_inner.x = p_inner_world.x(); p_inner.y = p_inner_world.y(); p_inner.z = p_inner_world.z();
      p_outer.x = p_outer_world.x(); p_outer.y = p_outer_world.y(); p_outer.z = p_outer_world.z();
      
      inner_points.push_back(p_inner);
      outer_points.push_back(p_outer);
    }

    for (size_t i = 0; i < inner_points.size() - 1; ++i) 
    {
      marker.points.push_back(inner_points[i]);
      marker.points.push_back(outer_points[i]);
      marker.points.push_back(inner_points[i+1]);

      marker.points.push_back(outer_points[i]);
      marker.points.push_back(outer_points[i+1]);
      marker.points.push_back(inner_points[i+1]);
    }

    sample_regions.markers.push_back(marker);
  }

  sample_region_pub_.publish(sample_regions);

  return;
}

void PlanningVisualization::publishRegResults(const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& aligned_cloud, const Eigen::MatrixXd& aligned_mesh_V, const Eigen::MatrixXi& aligned_mesh_F)
{
  pcl::PointCloud<pcl::PointXYZ> tar = *target_cloud;
  pcl::PointCloud<pcl::PointXYZ> src = *source_cloud;
  pcl::PointCloud<pcl::PointXYZ> aligned = *aligned_cloud;

  sensor_msgs::PointCloud2 target_msg, source_msg, aligned_msg;

  tar.width = tar.size();
  tar.height = 1;
  tar.is_dense = true;
  tar.header.frame_id = "world";

  src.width = src.size();
  src.height = 1;
  src.is_dense = true;
  src.header.frame_id = "world";

  aligned.width = aligned.size();
  aligned.height = 1;
  aligned.is_dense = true;
  aligned.header.frame_id = "world";

  pcl::toROSMsg(tar, target_msg);
  pcl::toROSMsg(src, source_msg);
  pcl::toROSMsg(aligned, aligned_msg);
  target_msg.header.stamp = ros::Time::now();
  source_msg.header.stamp = ros::Time::now();
  aligned_msg.header.stamp = ros::Time::now();

  gicp_target_pub_.publish(target_msg);
  gicp_source_pub_.publish(source_msg);
  gicp_aligned_cloud_pub_.publish(aligned_msg);

  visualization_msgs::Marker mesh_marker;
  mesh_marker.header.frame_id = "world";
  mesh_marker.header.stamp = ros::Time::now();
  mesh_marker.ns = "registration_aligned_mesh";
  mesh_marker.id = 0;
  mesh_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  mesh_marker.pose.orientation.w = 1.0;
  mesh_marker.scale.x = 1.0;
  mesh_marker.scale.y = 1.0;
  mesh_marker.scale.z = 1.0;
  mesh_marker.color.a = 1.0;
  mesh_marker.color.r = 0.75;
  mesh_marker.color.g = 0.75;
  mesh_marker.color.b = 0.75;

  mesh_marker.action = visualization_msgs::Marker::DELETEALL;
  gicp_aligned_mesh_pub_.publish(mesh_marker);

  for (int i = 0; i < aligned_mesh_F.rows(); ++i)
  {
    for (int j = 0; j < aligned_mesh_F.cols(); ++j)
    {
      int idx = aligned_mesh_F(i, j);
      geometry_msgs::Point p;
      p.x = aligned_mesh_V(idx, 0);
      p.y = aligned_mesh_V(idx, 1);
      p.z = aligned_mesh_V(idx, 2);
      mesh_marker.points.push_back(p);
    }
  }

  mesh_marker.action = visualization_msgs::Marker::ADD;
  gicp_aligned_mesh_pub_.publish(mesh_marker);

  return;
}

void PlanningVisualization::publishReplanGPath(vector<Eigen::VectorXd>& fullpath_)
{
  double line_scale = 0.07, sphere_scale = 0.15;

  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 0;
  mk.ns = "cur_global_path";
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.color.a = 0.8;
  mk.scale.x = line_scale;
  mk.scale.y = line_scale;
  mk.scale.z = line_scale;
  mk.pose.orientation.w = 1.0;

  mk.action = visualization_msgs::Marker::DELETEALL;
  replan_full_path_pub_.publish(mk);
  geometry_msgs::Point pt;
  for (int i=0; i<(int)fullpath_.size()-1; ++i)
  {
    pt.x = fullpath_[i](0);
    pt.y = fullpath_[i](1);
    pt.z = fullpath_[i](2);
    mk.points.push_back(pt);

    pt.x = fullpath_[i+1](0);
    pt.y = fullpath_[i+1](1);
    pt.z = fullpath_[i+1](2);
    mk.points.push_back(pt);
  }

  mk.action = visualization_msgs::Marker::ADD;
  replan_full_path_pub_.publish(mk);

  visualization_msgs::Marker mk_pts;
  mk_pts.header.frame_id = "world";
  mk_pts.header.stamp = ros::Time::now();
  mk_pts.id = 0;
  mk_pts.ns = "cur_global_path_waypts";
  mk_pts.type = visualization_msgs::Marker::SPHERE_LIST;
  mk_pts.color.r = 0.0;
  mk_pts.color.g = 0.0;
  mk_pts.color.b = 0.5;
  mk_pts.color.a = 0.7;
  mk_pts.scale.x = sphere_scale;
  mk_pts.scale.y = sphere_scale;
  mk_pts.scale.z = sphere_scale;
  mk_pts.pose.orientation.w = 1.0;

  mk_pts.action = visualization_msgs::Marker::DELETEALL;
  replan_full_waypts_pub_.publish(mk_pts);

  for (int i=0; i<(int)fullpath_.size(); ++i)
  {
    pt.x = fullpath_[i](0);
    pt.y = fullpath_[i](1);
    pt.z = fullpath_[i](2);
    mk_pts.points.push_back(pt);
  }

  mk_pts.action = visualization_msgs::Marker::ADD;
  replan_full_waypts_pub_.publish(mk_pts);

  return;
}

}
