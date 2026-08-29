/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Mar. 2025
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the tools functions of fsm of visibility-aware replanning in FC-Vision.
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
void ReplanFSM::localPathPlan()
{
  double path_l = this->local_path_length_;

  // * 1. find the path to be replanned in receding-horizon
  int next_idx = this->cur_start_g_idx_;

  vector<Eigen::VectorXd> cur_receding_path = {this->local_g_path_[next_idx]};
  vector<bool> cur_receding_indi = {this->local_g_indi_[next_idx]};
  double cur_receding_path_length = 0.0;
  int next_g_idx = next_idx;
  for (int i = next_idx + 1; i < (int)this->local_g_path_.size(); ++i)
  {
    cur_receding_path_length += (this->local_g_path_[i].head(3) - cur_receding_path.back().head(3)).norm();
    cur_receding_path.push_back(this->local_g_path_[i]);
    cur_receding_indi.push_back(this->local_g_indi_[i]);
    if (cur_receding_path_length > path_l && !this->local_g_indi_[i]) 
    {
      next_g_idx = min(i + 1, (int)this->local_g_path_.size() - 1);
      break;
    }
    next_g_idx = i;
  }

  vector<Eigen::VectorXd> viewpoints_set = {cur_receding_path.front()};
  for (int i=1; i<(int)cur_receding_path.size()-1; ++i)
  {
    if (!cur_receding_indi[i]) viewpoints_set.push_back(cur_receding_path[i]);
  }
  if ((cur_receding_path.back().head(3) - viewpoints_set.back().head(3)).norm() > 0.2*this->path_interval_) viewpoints_set.push_back(cur_receding_path.back());

  if (next_g_idx >= (int)this->local_g_path_.size() - 1 && (int)viewpoints_set.size() == 1)
  {
    ROS_INFO("\033[1m\033[37m\033[44m[LOCAL_REPLANNING_THREAD] : \033[7m\033[37m\033[44m------------ Path -> Reach to the end of global path! ------------");
    return;
  }

  // * 2. visibility-equivalent path replanning
  vector<Eigen::VectorXd> replan_path;
  vector<bool> replan_indi;

  this->vis_replan_->reset();
  this->vis_replan_->setInputPath(viewpoints_set);
  this->vis_replan_->replan();
  this->vis_replan_->getOutputPath(replan_path, replan_indi);
  replan_indi.front() = cur_receding_indi.front();

  double min_interval = 1.5*this->path_interval_;
  this->enforceMinIntervalCumulative(replan_path, replan_indi, min_interval);

  // * 3. update current global path
  vector<Eigen::VectorXd> new_g_path;
  vector<bool> new_g_indi;

  if (this->cur_start_g_idx_ > 0)
  {
    new_g_path.insert(new_g_path.end(), this->local_g_path_.begin(), this->local_g_path_.begin() + this->cur_start_g_idx_);
    new_g_indi.insert(new_g_indi.end(), this->local_g_indi_.begin(), this->local_g_indi_.begin() + this->cur_start_g_idx_);
  }

  new_g_path.insert(new_g_path.end(), replan_path.begin(), replan_path.end());
  new_g_indi.insert(new_g_indi.end(), replan_indi.begin(), replan_indi.end());

  if (next_g_idx < (int)this->local_g_path_.size() - 1)
  {
    new_g_path.insert(new_g_path.end(), this->local_g_path_.begin() + next_g_idx, this->local_g_path_.end());
    new_g_indi.insert(new_g_indi.end(), this->local_g_indi_.begin() + next_g_idx, this->local_g_indi_.end());
  }

  this->local_g_path_ = new_g_path;
  this->local_g_indi_ = new_g_indi;

  return;
}

void ReplanFSM::localTrajPlan(Trajectory<7>& pos_traj, Trajectory<7>& ori_traj, vector<int>& ctrl_ids, bool& finish)
{
  // * 1. prepare the control points of the trajectory
  vector<Eigen::VectorXd> traj_ctrl_pts = {this->local_start_};
  vector<bool> traj_ctrl_indi = {false};
  vector<int> traj_ctrl_ids;

  Eigen::Vector3d cur_pos, cur_vel;
  cur_pos << this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z;
  cur_vel << this->odom_.twist.twist.linear.x, this->odom_.twist.twist.linear.y, this->odom_.twist.twist.linear.z;
  
  Eigen::Vector3d next_pt = this->local_g_path_[this->cur_start_g_idx_].head(3);
  Eigen::Vector3d next_dir = next_pt - cur_pos;
  double ang = acos(cur_vel.dot(next_dir) / (cur_vel.norm() * next_dir.norm() + 1e-3));

  int next_idx = ang < M_PI / 3 ? this->cur_start_g_idx_ : this->cur_start_g_idx_ + 1;

  double traj_l = this->local_traj_length_;

  double cur_traj_length = 0.0;
  int waypt_num = 0;
  for (int i=next_idx; i<(int)this->local_g_path_.size(); ++i)
  {
    if (i == next_idx)
    {
      double l2n = (this->local_g_path_[i].head(3) - traj_ctrl_pts.back().head(3)).norm();
      if (l2n > 0.2*this->path_interval_)
      {
        traj_ctrl_pts.push_back(this->local_g_path_[i]);
        traj_ctrl_indi.push_back(this->local_g_indi_[i]);
        traj_ctrl_ids.push_back(i);
        cur_traj_length += l2n;
        waypt_num++;
      }
    }
    else
    {
      cur_traj_length += (this->local_g_path_[i].head(3) - traj_ctrl_pts.back().head(3)).norm();
      traj_ctrl_pts.push_back(this->local_g_path_[i]);
      traj_ctrl_indi.push_back(this->local_g_indi_[i]);
      traj_ctrl_ids.push_back(i);
      waypt_num++;
    }
    if (cur_traj_length > traj_l && waypt_num > 2) break;
  }

  if (traj_ctrl_ids.back() == (int)this->local_g_path_.size() - 1) this->reach_end_count_++;
  
  if (this->reach_end_count_ > 1)
  {
    ROS_INFO("\033[1m\033[37m\033[44m[LOCAL_REPLANNING_THREAD] : \033[7m\033[37m\033[44m------------ Traj -> Reach to the end of global path! ------------");
    finish = true;
    return;
  }
  traj_ctrl_indi[1] = true;

  // * 2. trajectory generation
  this->traj_gen_->reset();
  this->traj_gen_->trajGen(traj_ctrl_pts, traj_ctrl_indi, 1.7, this->local_start_vel_, this->local_start_acc_, this->local_start_pyd_, this->local_start_pyd_dot_);

  // * 3. get current trajectory
  pos_traj = this->traj_gen_->minco_traj;
  ori_traj = this->traj_gen_->minco_orientation_traj;
  ctrl_ids = traj_ctrl_ids;
  finish = false;

  return;
}

void ReplanFSM::trajConverter(const Trajectory<7> &pos, const Trajectory<7> &ori, quadrotor_msgs::PolynomialTrajGroup &msg, const ros::Time &cur_stamp, int &traj_id)
{
  msg.trajectory_id = traj_id;
  msg.header.stamp = cur_stamp;
  msg.action = msg.ACTION_ADD;
  msg.start_time = cur_stamp;

  int pn_pos = pos.getPieceNum();
  for (int i=0; i<pn_pos; ++i)
  {
    quadrotor_msgs::PolynomialMatrix piece_pos;
    piece_pos.num_dim = pos[i].getDim();
    piece_pos.num_order = pos[i].getDegree();
    piece_pos.duration = pos[i].getDuration();
    auto cMat = pos[i].getCoeffMat();
    piece_pos.data.assign(cMat.data(),cMat.data() + cMat.rows()*cMat.cols());
    msg.pos_traj.push_back(piece_pos);
  }

  int pn_ori = ori.getPieceNum();
  for (int i=0; i<pn_ori; ++i)
  {
    quadrotor_msgs::PolynomialMatrix piece_ori;
    piece_ori.num_dim = ori[i].getDim();
    piece_ori.num_order = ori[i].getDegree();
    piece_ori.duration = ori[i].getDuration();
    auto cMat = ori[i].getCoeffMat();
    piece_ori.data.assign(cMat.data(),cMat.data() + cMat.rows()*cMat.cols());
    msg.ori_traj.push_back(piece_ori);
  }

  return;
}

void ReplanFSM::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  this->odom_ = *msg;

  return;
}

void ReplanFSM::execCallback(const quadrotor_msgs::EigenVectorArrayConstPtr& msg)
{
  const quadrotor_msgs::EigenVectorArray &traj_msg = *msg;
  for (const auto& array : traj_msg.vectors)
  {
    Eigen::VectorXd vec(array.data.size());
    for (size_t i = 0; i < array.data.size(); ++i)
    {
      vec[i] = array.data[i];
    }
    this->new_exec_waypts_.push_back(vec);
  }

  if (this->prog_mtx_.try_lock())
  {
    this->exec_traj_waypts_.insert(this->exec_traj_waypts_.end(), this->new_exec_waypts_.begin(), this->new_exec_waypts_.end());
    this->already_rcv_num_ += (int)this->new_exec_waypts_.size();
    this->new_exec_waypts_.clear();
    this->prog_mtx_.unlock();
  }

  return;
}

void ReplanFSM::enforceMinIntervalCumulative(vector<Eigen::VectorXd> &replan_path, vector<bool> &replan_indi, double min_interval)
{
    int n = static_cast<int>(replan_path.size());
    if (n == 0) return;

    int    last_false_idx = -1;
    double cum_dist_since_last_false = 0.0;

    for (int i = 0; i < n; ++i)
    {
        if (i > 0 && last_false_idx >= 0)
        {
            cum_dist_since_last_false += (replan_path[i] - replan_path[i - 1]).norm();
        }

        if (!replan_indi[i])
        {
            if (last_false_idx < 0)
            {
                last_false_idx = i;
                cum_dist_since_last_false = 0.0;
            }
            else
            {
                if (cum_dist_since_last_false <= min_interval)
                {
                    replan_indi[i] = true;
                }
                else
                {
                    last_false_idx = i;
                    cum_dist_since_last_false = 0.0;
                }
            }
        }
    }

    return;
}
}
