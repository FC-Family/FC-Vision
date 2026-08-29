/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Mar. 2025
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the header file of replan_fsm class, which is 
 *                   fsm of visibility-aware replanning in FC-Vision.
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

#ifndef _REPLAN_FSM_H_
#define _REPLAN_FSM_H_

#include "plan_env/sdf_map.h"
#include "vis_utils/planning_visualization.h"
#include "active_perception/perception_utils.h"
#include "path_searching/geometry_utils.hpp"
#include "path_searching/path_tools.hpp"
#include "visibility_replan/visibility_replan.h"
#include "traj_generator/traj_gen.h"
#include "gcopter/trajectory.hpp"
#include "quadrotor_msgs/PolynomialTrajGroup.h"
#include "quadrotor_msgs/EigenVectorArray.h"
#include <ros/ros.h>
#include <thread>
#include <Eigen/Core>
#include <boost/circular_buffer.hpp>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>

using namespace std;

namespace fc_vision
{

class SDFMap;
class PlanningVisualization;
class PerceptionUtils;
class VisibilityReplan;
class TrajGenerator;

enum LOCAL_STATE { LOCAL_SILENCE, LOCAL_PLAN, LOCAL_EXEC, LOCAL_FINISHED };

class ReplanFSM
{
public:
  ReplanFSM(){
  }
  ~ReplanFSM(){
  }
  /* Func */
  void init(ros::NodeHandle& nh);
  void startService();
  void stopService();
  void setMap(shared_ptr<SDFMap> map);
  void setGlobalPlan(const vector<Eigen::VectorXd>& g_path, const vector<bool>& g_indi);
  void triggerLocalPlan();
  void getCurGlobalPath(vector<Eigen::VectorXd>& g_path, vector<bool>& g_indi);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  /* Task */
  std::thread local_planning_thread_;
  std::thread safety_thread_;
  std::thread vis_thread_;
  std::thread progress_thread_;
  int local_planning_running_ = 1, safety_running_ = 1, vis_running_ = 1, progress_running_ = 1;
  std::mutex local_state_mtx_, local_output_mtx_, vis_mtx_, prog_mtx_;
  vector<string> local_state_str_ = {"LOCAL_SILENCE", "LOCAL_PLAN", "LOCAL_EXEC", "LOCAL_FINISHED"};
  /* ROS Rate */
  unique_ptr<ros::Rate> local_planning_rate_ = nullptr;
  unique_ptr<ros::Rate> safety_rate_ = nullptr;
  unique_ptr<ros::Rate> vis_rate_ = nullptr;
  unique_ptr<ros::Rate> progress_rate_ = nullptr;
  /* Param */
  double drone_radius_ = 0.0, safety_horizon_ = 0.0;
  double local_fps_ = 0.0, safety_fps_ = 0.0, vis_fps_ = 0.0, progress_fps_ = 0.0;
  bool en_local_replan_ = false;
  double path_interval_ = 0.0;
  double local_traj_length_, local_path_length_;
  double replan_time_, replan_full_exec_, replan_periodic_, replan_proportion_;
  /* Func */
  void localPlanThread();
  void safetyThread();
  void visThread();
  void progressThread();
  /* Buffer */
  boost::circular_buffer<LOCAL_STATE> local_state_;
  boost::circular_buffer<Trajectory<7>> buffer_pos_trajs_;
  boost::circular_buffer<Trajectory<7>> buffer_ori_trajs_;
  boost::circular_buffer<ros::Time> buffer_start_times_;
  boost::circular_buffer<vector<Eigen::VectorXd>> buffer_cur_g_paths_;
  /* Data */
  vector<Eigen::VectorXd> local_g_path_;
  vector<bool> local_g_indi_;
  vector<Eigen::VectorXd> original_g_path_;
  vector<vector<Eigen::Vector3d>> original_fov_starts_;
  vector<vector<Eigen::Vector3d>> original_fov_ends_;
  int cur_start_g_idx_ = 0, traj_id_ = 1, reach_end_count_ = 0;
  Eigen::VectorXd local_start_;
  Eigen::Vector3d local_start_vel_, local_start_acc_, local_start_pyd_, local_start_pyd_dot_;
  nav_msgs::Odometry odom_;
  vector<Eigen::VectorXd> exec_traj_waypts_, new_exec_waypts_;
  int process_exec_wp_idx_ = 0, already_rcv_num_ = 0;
  /* ROS Service */
  ros::Publisher local_traj_pub_, finish_pub_, brake_pub_;
  ros::Subscriber odom_sub_, exec_sub_;
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);
  void execCallback(const quadrotor_msgs::EigenVectorArrayConstPtr& msg);
  /* Utils */
  shared_ptr<SDFMap> map_;
  unique_ptr<PlanningVisualization> vis_utils_ = nullptr;
  unique_ptr<PerceptionUtils> percep_utils_ = nullptr;
  unique_ptr<VisibilityReplan> vis_replan_ = nullptr;
  unique_ptr<TrajGenerator> traj_gen_ = nullptr;
  /* Traj */
  ros::Time last_start_time_, newest_start_time_;
  vector<int> last_ctrl_ids_, newest_ctrl_ids_;
  Trajectory<7> last_pos_traj_, last_orientation_traj_, newest_pos_traj_, newest_orientation_traj_;
  double last_duration_, newest_duration_;
  /* Tools */
  void localPathPlan();
  void localTrajPlan(Trajectory<7>& pos_traj, Trajectory<7>& ori_traj, vector<int>& ctrl_ids, bool& finish);
  void trajConverter(const Trajectory<7> &pos, const Trajectory<7> &ori, quadrotor_msgs::PolynomialTrajGroup &msg, const ros::Time &cur_stamp, int &traj_id);
  void enforceMinIntervalCumulative(vector<Eigen::VectorXd> &replan_path, vector<bool> &replan_indi, double min_interval);
};

}

#endif
