/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Mar. 2025
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the main algorithm of fsm of visibility-aware replanning in FC-Vision.
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

#include "visibility_replan/replan_fsm.h"

namespace fc_vision
{
void ReplanFSM::init(ros::NodeHandle& nh)
{
  // * Module Initialization
  this->vis_utils_.reset(new PlanningVisualization(nh));
  this->percep_utils_.reset(new PerceptionUtils);
  this->percep_utils_->init(nh);
  this->vis_replan_.reset(new VisibilityReplan);
  this->vis_replan_->init(nh);
  this->vis_replan_->en_vis_ = true;
  this->traj_gen_.reset(new TrajGenerator);
  this->traj_gen_->init(nh);
  this->traj_gen_->visFlag = true;

  // * Param Initialization
  nh.param("replanning_fsm/drone_radius",      this->drone_radius_, 0.5);
  nh.param("replanning_fsm/path_interval",     this->path_interval_, 1.0);
  nh.param("replanning_fsm/local_fps",         this->local_fps_, 1.0);
  nh.param("replanning_fsm/safety_fps",        this->safety_fps_, 1.0);
  nh.param("replanning_fsm/vis_fps",           this->vis_fps_, 1.0);
  nh.param("replanning_fsm/progress_fps",      this->progress_fps_, 1.0);
  nh.param("replanning_fsm/traj_l",            this->local_traj_length_, 1.0);
  nh.param("replanning_fsm/path_l",            this->local_path_length_, 1.0);
  nh.param("replanning_fsm/replan_time",       this->replan_time_, 1.0);
  nh.param("replanning_fsm/replan_full_exec",  this->replan_full_exec_, 1.0);
  nh.param("replanning_fsm/replan_periodic",   this->replan_periodic_, 1.0);
  nh.param("replanning_fsm/replan_proportion", this->replan_proportion_, 1.0);
  nh.param("replanning_fsm/safety_horizon",    this->safety_horizon_, 1.0);

  this->en_local_replan_ = false;
  this->cur_start_g_idx_ = 0;
  this->traj_id_ = 1;
  this->reach_end_count_ = 0;
  this->last_start_time_ = ros::Time(0.0);
  this->newest_start_time_ = ros::Time(0.0);

  // * Thread Initialization
  this->local_planning_rate_ = make_unique<ros::Rate>(ros::Rate(this->local_fps_));
  this->safety_rate_ = make_unique<ros::Rate>(ros::Rate(this->safety_fps_));
  this->vis_rate_ = make_unique<ros::Rate>(ros::Rate(this->vis_fps_));
  this->progress_rate_ = make_unique<ros::Rate>(ros::Rate(this->progress_fps_));

  // * Trigger Initialization
  this->local_state_ = boost::circular_buffer<LOCAL_STATE>(1);
  this->local_state_.push_back(LOCAL_SILENCE);
  this->buffer_pos_trajs_ = boost::circular_buffer<Trajectory<7>>(2);
  this->buffer_ori_trajs_ = boost::circular_buffer<Trajectory<7>>(2);
  this->buffer_start_times_ = boost::circular_buffer<ros::Time>(2);
  this->buffer_cur_g_paths_ = boost::circular_buffer<vector<Eigen::VectorXd>>(10);

  // * Service Initialization
  this->local_traj_pub_ = nh.advertise<quadrotor_msgs::PolynomialTrajGroup>("/local_newest_traj", 1);
  this->finish_pub_ = nh.advertise<std_msgs::Bool>("/flight_end", 1);
  this->brake_pub_ = nh.advertise<std_msgs::Bool>("/emergency_brake", 1);
  this->odom_sub_ = nh.subscribe("/replanning_fsm/odometry", 1, &ReplanFSM::odomCallback, this);
  this->exec_sub_ = nh.subscribe("/traj_server/exec_traj_waypts", 100, &ReplanFSM::execCallback, this);

  ROS_INFO("\033[35m[VisEqReplan] Initialized! \033[32m");

  return;
}

void ReplanFSM::startService()
{
  this->local_planning_thread_ = std::thread(&ReplanFSM::localPlanThread, this);
  this->safety_thread_ = std::thread(&ReplanFSM::safetyThread, this);
  this->vis_thread_ = std::thread(&ReplanFSM::visThread, this);
  this->progress_thread_ = std::thread(&ReplanFSM::progressThread, this);

  return;
}

void ReplanFSM::stopService()
{
  if (this->local_planning_thread_.joinable())
  {
    this->local_planning_running_ = 0;
    this->local_planning_thread_.join();
  }

  if (this->safety_thread_.joinable())
  {
    this->safety_running_ = 0;
    this->safety_thread_.join();
  }

  if (this->vis_thread_.joinable())
  {
    this->vis_running_ = 0;
    this->vis_thread_.join();
  }

  if (this->progress_thread_.joinable())
  {
    this->progress_running_ = 0;
    this->progress_thread_.join();
  }

  return;
}

void ReplanFSM::setMap(shared_ptr<SDFMap> map)
{
  this->map_ = map;
  this->vis_replan_->setMap(this->map_);
  this->traj_gen_->setMap(this->map_);

  return;
}

void ReplanFSM::setGlobalPlan(const vector<Eigen::VectorXd>& g_path, const vector<bool>& g_indi)
{
  this->local_g_path_ = g_path;
  this->local_g_indi_ = g_indi;
  this->original_g_path_ = g_path;
  this->original_fov_starts_.clear();
  this->original_fov_ends_.clear();

  const size_t point_count = min(g_path.size(), g_indi.size());
  for (size_t i = 0; i < point_count; ++i)
  {
    if (g_indi[i] || g_path[i].size() < 5) continue;

    vector<Eigen::Vector3d> fov_starts;
    vector<Eigen::Vector3d> fov_ends;
    this->percep_utils_->setPose_PY(g_path[i].head(3), g_path[i](3), g_path[i](4));
    this->percep_utils_->getFOV_PY(fov_starts, fov_ends);
    this->original_fov_starts_.push_back(fov_starts);
    this->original_fov_ends_.push_back(fov_ends);
  }

  {
    lock_guard<mutex> lock(this->vis_mtx_);
    this->buffer_cur_g_paths_.push_back(this->local_g_path_);
  }

  return;
}

void ReplanFSM::triggerLocalPlan()
{
  if (this->local_g_path_.empty())
  {
    ROS_WARN("ReplanFSM: No global path to trigger.");
    return;
  }

  if (!this->en_local_replan_)
  {
    if (this->local_state_mtx_.try_lock())
    {
      this->local_state_.push_back(LOCAL_PLAN);
      this->local_state_mtx_.unlock();
      this->en_local_replan_ = true;
    }
  }

  return;
}

void ReplanFSM::getCurGlobalPath(vector<Eigen::VectorXd>& g_path, vector<bool>& g_indi)
{
  g_path = this->local_g_path_;
  g_indi = this->local_g_indi_;

  return;
}

void ReplanFSM::localPlanThread()
{
  ros::Time t_plan, cur_time;
  bool local_finish = false, finish = false;
  quadrotor_msgs::PolynomialTrajGroup msg;

  while (this->local_planning_running_)
  {
    LOCAL_STATE local_state;

    // * Read Local State
    if (this->local_state_mtx_.try_lock())
    {
      if (this->local_state_.empty())
      {
        this->local_state_mtx_.unlock();
        this->local_planning_rate_->sleep();
        continue;
      }
      else
      {
        local_state = this->local_state_.back();
        this->local_state_mtx_.unlock();
      }
    }
    else
    {
      this->local_planning_rate_->sleep();
      continue;
    }

    ROS_INFO_STREAM_THROTTLE(2.0, "\033[1m\033[37m\033[44m[LOCAL_REPLANNING_THREAD] : \033[7m\033[37m\033[44m" << this->local_state_str_[int(local_state)].c_str());
    
    switch (local_state)
    {
      case LOCAL_SILENCE:
      {        
        this->local_planning_rate_->sleep();
        break;
      }

      case LOCAL_FINISHED:
      {
        this->local_planning_rate_->sleep();
        break;
      }

      case LOCAL_PLAN:
      {
        if (local_finish == false)
        {
          t_plan = ros::Time::now() + ros::Duration(this->replan_time_);
          if (this->traj_id_ <= 1)
          {
            this->local_start_ = this->local_g_path_.front();
            this->local_start_vel_ = Eigen::Vector3d::Zero();
            this->local_start_acc_ = Eigen::Vector3d::Zero();
            this->local_start_pyd_ = Eigen::Vector3d::Zero();
            this->local_start_pyd_dot_ = Eigen::Vector3d::Zero();
            this->cur_start_g_idx_ = 0;
          }
          else
          {
            double t_old = (t_plan - this->last_start_time_).toSec();
            double t_new = (t_plan - this->newest_start_time_).toSec();

            if (t_old > 0 && t_new < 0)
            {
              Eigen::Vector3d plan_pos = this->last_pos_traj_.getPos(t_old);
              double plan_pitch = this->last_orientation_traj_.getPitch(t_old);
              double plan_yaw = this->last_orientation_traj_.getYaw(t_old);
              this->local_start_.resize(5);
              this->local_start_ << plan_pos(0), plan_pos(1), plan_pos(2), plan_pitch, plan_yaw;
              this->local_start_vel_ = this->last_pos_traj_.getVel(t_old);
              this->local_start_acc_ = this->last_pos_traj_.getAcc(t_old);

              double pitch_vel = this->last_orientation_traj_.getPitchd(t_old);
              double yaw_vel = this->last_orientation_traj_.getYawd(t_old);
              this->local_start_pyd_ << pitch_vel, yaw_vel, 0.0;
              double pitch_acc = this->last_orientation_traj_.getPitchdd(t_old);
              double yaw_acc = this->last_orientation_traj_.getYawdd(t_old);
              this->local_start_pyd_dot_ << pitch_acc, yaw_acc, 0.0;

              int piece_id_old = this->last_pos_traj_.locatePieceIdx(t_old);
              this->cur_start_g_idx_ = this->last_ctrl_ids_[piece_id_old];
            }
            else if (t_new >= 0)
            {
              Eigen::Vector3d plan_pos = this->newest_pos_traj_.getPos(t_new);
              double plan_pitch = this->newest_orientation_traj_.getPitch(t_new);
              double plan_yaw = this->newest_orientation_traj_.getYaw(t_new);
              this->local_start_.resize(5);
              this->local_start_ << plan_pos(0), plan_pos(1), plan_pos(2), plan_pitch, plan_yaw;
              this->local_start_vel_ = this->newest_pos_traj_.getVel(t_new);
              this->local_start_acc_ = this->newest_pos_traj_.getAcc(t_new);

              double pitch_vel = this->newest_orientation_traj_.getPitchd(t_new);
              double yaw_vel = this->newest_orientation_traj_.getYawd(t_new);
              this->local_start_pyd_ << pitch_vel, yaw_vel, 0.0;
              double pitch_acc = this->newest_orientation_traj_.getPitchdd(t_new);
              double yaw_acc = this->newest_orientation_traj_.getYawdd(t_new);
              this->local_start_pyd_dot_ << pitch_acc, yaw_acc, 0.0;

              int piece_id_new = this->newest_pos_traj_.locatePieceIdx(t_new);
              this->cur_start_g_idx_ = this->newest_ctrl_ids_[piece_id_new];
            }
          }

          this->localPathPlan();

          Trajectory<7> pos_traj, ori_traj;
          vector<int> ctrl_ids;
          this->localTrajPlan(pos_traj, ori_traj, ctrl_ids, finish);

          if (finish)
          {
            if (this->local_state_mtx_.try_lock())
            {
              this->local_state_.push_back(LOCAL_FINISHED);
              this->local_state_mtx_.unlock();

              std_msgs::Bool finish_singal;
              finish_singal.data = true;
              this->finish_pub_.publish(finish_singal);
            }

            this->local_planning_rate_->sleep();
            continue;
          }

          if (!this->local_output_mtx_.try_lock())
          {
            this->local_planning_rate_->sleep();
            continue;
          }
          else
          {
            this->buffer_pos_trajs_.push_back(pos_traj);
            this->buffer_ori_trajs_.push_back(ori_traj);
            this->buffer_start_times_.push_back(t_plan);
            this->local_output_mtx_.unlock();

            this->last_pos_traj_.clear();
            this->last_orientation_traj_.clear();
            this->last_ctrl_ids_.clear();
            this->last_pos_traj_ = this->newest_pos_traj_;
            this->last_orientation_traj_ = this->newest_orientation_traj_;
            this->last_ctrl_ids_ = this->newest_ctrl_ids_;
            this->newest_pos_traj_.clear();
            this->newest_orientation_traj_.clear();
            this->newest_ctrl_ids_.clear();
            this->newest_pos_traj_ = pos_traj;
            this->newest_orientation_traj_ = ori_traj;
            this->newest_ctrl_ids_ = ctrl_ids;
            this->last_duration_ = this->last_pos_traj_.getTotalDuration();
            this->newest_duration_ = this->newest_pos_traj_.getTotalDuration();

            if (this->traj_id_ <= 1) this->newest_start_time_ = t_plan;
            else
            {
              this->last_start_time_ = this->newest_start_time_;
              this->newest_start_time_ = t_plan;
            }

            quadrotor_msgs::PolynomialTrajGroup temp_msg;
            this->trajConverter(this->newest_pos_traj_, this->newest_orientation_traj_, temp_msg, this->newest_start_time_, this->traj_id_);

            msg = temp_msg;
            this->traj_id_++;
          }

          if (this->vis_mtx_.try_lock())
          {
            this->buffer_cur_g_paths_.push_back(this->local_g_path_);
            this->vis_mtx_.unlock();
          }

          local_finish = true;
        }

        if (this->local_state_mtx_.try_lock())
        {
          this->local_state_.push_back(LOCAL_EXEC);
          this->local_state_mtx_.unlock();
          local_finish = false;
          this->local_traj_pub_.publish(msg);
        }

        this->local_planning_rate_->sleep();
        break;
      }

      case LOCAL_EXEC:
      {
        // * Replan Condition -> 1) periodic 2) traj fully executed
        bool pass = false, new_traj = false; 

        cur_time = ros::Time::now();
        double t_old = (cur_time - this->last_start_time_).toSec();
        double t_new = (cur_time - this->newest_start_time_).toSec();
        double time_to_end = 0.0;
        double t_cur = 0.0;
        double duration = 0.0;

        if (t_new >= 0)
        {
          new_traj = true;

          t_cur = t_new;
          duration = this->newest_duration_;
          time_to_end = duration - t_cur;
          if (time_to_end < this->replan_full_exec_)
          {
            if (this->local_state_mtx_.try_lock())
            {
              this->local_state_.push_back(LOCAL_PLAN);
              this->local_state_mtx_.unlock();
              pass = true;
              ROS_INFO("\033[1m\033[37m\033[44m[LOCAL_REPLANNING_THREAD] : \033[7m\033[37m\033[44m------------ Replan due to full execution! ------------");
            }
          }
        }
        else if (t_old > 0 && t_new < 0)
        {
          t_cur = t_old;
          duration = this->last_duration_;
        }

        if (pass == false && new_traj == true)
        {
          if (t_cur > this->replan_proportion_ * duration)
          // if (t_cur > this->replan_periodic_)
          {
            if (this->local_state_mtx_.try_lock())
            {
              this->local_state_.push_back(LOCAL_PLAN);
              this->local_state_mtx_.unlock();
              ROS_INFO("\033[1m\033[37m\033[44m[LOCAL_REPLANNING_THREAD] : \033[7m\033[37m\033[44m------------ Replan due to periodic call! ------------");
            }
          }
        }

        this->local_planning_rate_->sleep();
        break;
      }
    }
  }

  return;
}

void ReplanFSM::safetyThread()
{
  Trajectory<7> last_pos_traj, last_ori_traj;
  Trajectory<7> new_pos_traj, new_ori_traj;
  ros::Time last_traj_start_time = ros::Time(0.0), new_traj_start_time = ros::Time(0.0);
  bool safe = true, info_pass = false;

  while (this->safety_running_)
  {
    ROS_INFO_STREAM_THROTTLE(2.0, "\033[1m\033[37m\033[43m[SAFETY_THREAD] : \033[7m\033[37m\033[43m" << "Running......");

    LOCAL_STATE temp_local_state;
    
    if (this->local_state_mtx_.try_lock())
    {
      if (this->local_state_.empty())
      {
        this->local_state_mtx_.unlock();
        this->safety_rate_->sleep();
        continue;
      }
      else
      {
        temp_local_state = this->local_state_.back();
        this->local_state_mtx_.unlock();
      }
    }
    else
    {
      this->safety_rate_->sleep();
      continue;
    }

    if (temp_local_state == LOCAL_STATE::LOCAL_EXEC)
    {
      // * Input Interface
      if (this->local_output_mtx_.try_lock())
      {
        if (this->buffer_pos_trajs_.empty() || this->buffer_ori_trajs_.empty() || this->buffer_start_times_.empty())
        {
          this->local_output_mtx_.unlock();
          this->safety_rate_->sleep();
          continue;
        }
        else
        {
          if ((int)this->buffer_start_times_.size() == 1)
          {
            new_pos_traj = this->buffer_pos_trajs_.back();
            new_ori_traj = this->buffer_ori_trajs_.back();
            new_traj_start_time = this->buffer_start_times_.back();
            last_traj_start_time = ros::Time(0.0);
          }

          if ((int)this->buffer_start_times_.size() == 2)
          {
            last_pos_traj = this->buffer_pos_trajs_.front();
            last_ori_traj = this->buffer_ori_trajs_.front();
            new_pos_traj = this->buffer_pos_trajs_.back();
            new_ori_traj = this->buffer_ori_trajs_.back();
            last_traj_start_time = this->buffer_start_times_.front();
            new_traj_start_time = this->buffer_start_times_.back();
          }
          this->local_output_mtx_.unlock();
        }
      }
      else
      {
        this->safety_rate_->sleep();
        continue;
      }

      // * Traj Safety Check 
      if (info_pass == false)
      {
        double t_now = (ros::Time::now() - new_traj_start_time).toSec();
        double duration = new_pos_traj.getTotalDuration();
        if (t_now >= 0)
        {
          Eigen::Vector3d cur_pos = new_pos_traj.getPos(t_now);
          double radius = 0.0;
          Eigen::Vector3d future_pos;
          double future_t = 0.02;
          while (radius < this->safety_horizon_ && t_now + future_t < duration)
          {
            future_pos = new_pos_traj.getPos(t_now + future_t);
            if (!this->map_->getSafety(future_pos, this->drone_radius_, this->drone_radius_, 0.5*this->drone_radius_))
            {
              ROS_ERROR("collision is detected at (%lf, %lf, %lf)!!!", future_pos(0), future_pos(1), future_pos(2));
              safe = false;
              info_pass = true;
              break;
            }
            radius = (future_pos - cur_pos).norm();
            future_t += 0.02;
          }
        }
      }

      if (!safe)
      {
        ROS_ERROR("------------ Replan due to collision detected! ------------");        
        if (this->local_state_mtx_.try_lock())
        {
          if (this->local_output_mtx_.try_lock())
          {
            this->local_state_.push_back(LOCAL_PLAN);
            this->local_output_mtx_.unlock();
            this->local_state_mtx_.unlock();
            info_pass = false;
            safe = true;
          }
          else
          {
            this->local_state_mtx_.unlock();
          }
        }
      }
    }

    this->safety_rate_->sleep();
  }

  return;
}

void ReplanFSM::visThread()
{
  vector<Eigen::VectorXd> cur_g_path;

  while (this->vis_running_)
  {
    ROS_INFO_STREAM_THROTTLE(2.0, "\033[1m\033[37m\033[42m[VISUALIZATION_THREAD] : \033[7m\033[37m\033[42m" << "Running......");

    if (this->vis_mtx_.try_lock())
    {
      if (!this->buffer_cur_g_paths_.empty()) cur_g_path = this->buffer_cur_g_paths_.back();
      this->vis_mtx_.unlock();
    }

    if (!cur_g_path.empty()) this->vis_utils_->publishReplanGPath(cur_g_path);
    if (!this->original_g_path_.empty())
    {
      this->vis_utils_->publishHCOPPPath(this->original_g_path_);
      this->vis_utils_->publishFOV(this->original_fov_starts_, this->original_fov_ends_);
    }

    this->vis_rate_->sleep();
  }

  return;
}

void ReplanFSM::progressThread()
{
  while (this->progress_running_)
  {
    ROS_INFO_STREAM_THROTTLE(2.0, "\033[1m\033[37m\033[46m[PROGRESS_THREAD] Check\033[7m\033[37m\033[46m");

    vector<Eigen::VectorXd> temp_exec_waypts;
    int exec_size = this->process_exec_wp_idx_;
    if (this->prog_mtx_.try_lock())
    {
      auto start_it = this->exec_traj_waypts_.begin() + this->process_exec_wp_idx_;
      auto end_it   = this->exec_traj_waypts_.end();
      temp_exec_waypts.assign(start_it, end_it);
      exec_size = (int)this->exec_traj_waypts_.size();
      this->prog_mtx_.unlock();
    }

    if (!temp_exec_waypts.empty())
    {
      double max_interval = -1.0;
      for (int i=0; i<(int)temp_exec_waypts.size()-1; ++i)
      {
        max_interval = max(max_interval, (temp_exec_waypts[i+1].head(3) - temp_exec_waypts[i].head(3)).norm());
      }

      for (const auto& wp : temp_exec_waypts)
      {
        Eigen::Vector3d pos = wp.head(3);
        double pitch = wp(3), yaw = wp(4);
        this->percep_utils_->setPose_PY(pos, pitch, yaw);
        Eigen::Vector3d min_vp_b, max_vp_b;
        this->percep_utils_->getFOVAABB(min_vp_b, max_vp_b, max_interval);
        pcl::PointCloud<pcl::PointXYZ>::Ptr vp_fov_aabb(new pcl::PointCloud<pcl::PointXYZ>);
        this->map_->getLocalHCMap(vp_fov_aabb, min_vp_b, max_vp_b);

        for (auto& point : vp_fov_aabb->points)
        {
          Eigen::Vector3d p_w(point.x, point.y, point.z);
          if (this->percep_utils_->insideShrinkFOV(p_w))
          {
            bool visible = true;
            double vis_inf = 2 * this->map_->getResolution();
            vector<Eigen::Vector3d> line_samples = visibility_utils::sampleLine(pos, p_w, vis_inf, this->map_->getResolution());
            for (Eigen::Vector3d& sample_pt : line_samples)
            {
              double hc_esdf = this->map_->getDistance_hc(sample_pt);
              if (abs(hc_esdf) < 0.2)
              {
                visible = false;
                break;
              }
            }

            if (visible) this->map_->setObserved(p_w);
          }
        }
      }

      this->process_exec_wp_idx_ = exec_size;
    }

    this->progress_rate_->sleep();
  }

  return;
}

}
