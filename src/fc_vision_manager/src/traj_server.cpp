/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    May. 2025
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the trajectory server in FC-Vision as the bridge between
 *                   planner and controller.
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

#include "gcopter/trajectory.hpp"
#include "quadrotor_msgs/PositionCommand.h"
#include "quadrotor_msgs/PolynomialTrajGroup.h"
#include "quadrotor_msgs/EigenVectorArray.h"
#include "active_perception/perception_utils.h"
#include "airsim_ros_pkgs/GimbalAngleEulerCmd.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Eigen>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <yaml-cpp/yaml.h>
#include <tf/tf.h>
#include <memory>
#include <thread>
#include <chrono>
#include <boost/circular_buffer.hpp>
#include <iostream>
#include <fstream>

using namespace std;
using std::unique_ptr;
using fc_vision::PerceptionUtils;

/* Param */
bool traj_server_enable_;
double max_vel_;
double fov_h_, fov_w_;
double pitch_upper_, pitch_lower_, drone_radius_;
double cur_interval = 0.0, sample_dist_;
int exec_waypts_sent_num_ = 0, already_pub_num_ = 0;
Eigen::VectorXd last_pose(5);
double ctrl_interval_;

/* ROS Service */
ros::Timer traj_server_vis_timer_, cmd_timer_, exec_waypts_timer_, robot_vis_timer_;
ros::Publisher pos_traj_pub_, ori_traj_pub_, exec_traj_pub_, exec_ori_pub_, odom_traj_pub_, robot_vis_pub_, robot_ellipsoid_pub_, fov_vis_pub_, pos_cmd_pub_, gimbal_cmd_pub_, exec_traj_waypts_pub_;
ros::Subscriber traj_sub_, finish_sub_, brake_sub_, path_sub_, demo_command_sub_;

/* Data */
visualization_msgs::MarkerArray last_ori_marker_set_;
Trajectory<7> last_position_traj_, last_orientation_traj_;
Trajectory<7> newest_position_traj_, newest_orientation_traj_;
double last_dura_ = 0.0, newest_dura_ = 0.0, span_;
ros::Time last_traj_start_time_ = ros::Time(0.0), newest_traj_start_time_ = ros::Time(0.0);
int traj_id_ = -2, pub_exec_id_ = 0;
bool received_traj_ = false, end_ = false, statistics_start_ = false;
vector<Eigen::VectorXd> traj_cmds_, exec_traj_waypts_;
ros::Time flight_start_time_;
double flight_length_ = 0.0, flight_time_ = 0.0, flight_mean_vel_ = 0.0;
ros::Time end_time_;
vector<Eigen::Vector3d> odom_traj_;
nav_msgs::Odometry cur_odom_;
geometry_msgs::QuaternionStamped cur_gimbal_;
boost::circular_buffer<nav_msgs::OdometryConstPtr> buffer_odom_;
boost::circular_buffer<geometry_msgs::QuaternionStampedConstPtr> buffer_gimbal_;
boost::circular_buffer<quadrotor_msgs::EigenVectorArray> buffer_path_;

/* Controller */
quadrotor_msgs::PositionCommand cmd_;
airsim_ros_pkgs::GimbalAngleEulerCmd gimbal_cmd_;

/* Odometry */
nav_msgs::Odometry odom_;
geometry_msgs::QuaternionStamped gimbal_orientation_;

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, geometry_msgs::QuaternionStamped> SyncPolicyPoseGimbal;
typedef unique_ptr<message_filters::Synchronizer<SyncPolicyPoseGimbal>> SynchronizerPoseGimbal;

unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
unique_ptr<message_filters::Subscriber<geometry_msgs::QuaternionStamped>> gimbal_sub_;
SynchronizerPoseGimbal sync_odom_gimbal_;

/* Utils */
Eigen::Vector3d jetColorMap(double value)
{
  double r, g, b;
  if (value < 0.0) value = 0.0;
  else if (value > 1.0) value = 1.0;

  if (value < 0.25)
  {
    r = 0.0;
    g = 4.0 * value;
    b = 1.0;
  }
  else if (value < 0.5)
  {
    r = 0.0;
    g = 1.0;
    b = 1.0 - 4.0 * (value - 0.25);
  }
  else if (value < 0.75)
  {
    r = 4.0 * (value - 0.5);
    g = 1.0;
    b = 0.0;
  }
  else
  {
    r = 1.0;
    g = 1.0 - 4.0 * pow((value - 0.75), 1);
    b = 0.0;
  }

  return Eigen::Vector3d(r, g, b);
}

unique_ptr<PerceptionUtils> percepUtils_;

/* Func */
void odomCallback(const nav_msgs::OdometryConstPtr& msg, const geometry_msgs::QuaternionStampedConstPtr& gimbal)
{
  cur_odom_ = *msg;
  cur_gimbal_ = *gimbal;
  
  if (received_traj_)
  {
    buffer_odom_.push_back(msg);
    buffer_gimbal_.push_back(gimbal);

    Eigen::Vector3d pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    odom_traj_.push_back(pos);
  }

  return;
}

void trajCallback(quadrotor_msgs::PolynomialTrajGroupConstPtr tMsg)
{
  const quadrotor_msgs::PolynomialTrajGroup &traj = *tMsg;

  if (traj.action == quadrotor_msgs::PolynomialTrajGroup::ACTION_ADD)
  {
    if ((int)traj.trajectory_id > traj_id_)
    {
      traj_id_ = traj.trajectory_id;
      
      Trajectory<7> pos_traj, ori_traj;

      for (auto &piece_pos : traj.pos_traj)
      {
        pos_traj.emplace_back(piece_pos.duration, Eigen::Map<const Eigen::MatrixXd>(&piece_pos.data[0], piece_pos.num_dim, (piece_pos.num_order + 1)));
      }

      for (auto &piece_ori : traj.ori_traj)
      {
        ori_traj.emplace_back(piece_ori.duration, Eigen::Map<const Eigen::MatrixXd>(&piece_ori.data[0], piece_ori.num_dim, (piece_ori.num_order + 1)));
      }

      last_position_traj_.clear();
      last_orientation_traj_.clear();
      last_position_traj_ = newest_position_traj_;
      last_orientation_traj_ = newest_orientation_traj_;
      newest_position_traj_.clear();
      newest_orientation_traj_.clear();
      newest_position_traj_ = pos_traj;
      newest_orientation_traj_ = ori_traj;

      last_traj_start_time_ = newest_traj_start_time_;
      newest_traj_start_time_ = traj.start_time;

      last_dura_ = last_position_traj_.getTotalDuration();
      newest_dura_ = newest_position_traj_.getTotalDuration();
      received_traj_ = true;
    }
  }

  return;
}

void pathCallback(const quadrotor_msgs::EigenVectorArrayConstPtr& pathMsg)
{
  buffer_path_.push_back(*pathMsg);

  return;
}

void finishCallback(const std_msgs::BoolConstPtr& msg)
{
  if (msg->data == true)
  {
    ROS_INFO("\033[31m[Traj Server] ------------------------------> Stop! \033[32m");
  }

  return;
}

void brakeCallback(const std_msgs::BoolConstPtr& msg)
{
  if (msg->data == true)
  {
    // * For simulation
    if (end_ == false)
    {
      end_ = true;
      end_time_ = ros::Time::now();
    }
    // * For real DJI
    // DjiFlightController_ExecuteEmergencyBrakeAction();

    ROS_INFO("\033[31m[Traj Server] ------------------------------> Emergency Brake! \033[32m");
  }

  return;
}

void demoCommandCallback(const std_msgs::UInt8::ConstPtr& command)
{
  if (command && command->data == 1) statistics_start_ = true;
}

void plannedTrajVis()
{
  if (last_position_traj_.getPieceNum() == 0 && newest_position_traj_.getPieceNum() == 0)
    return;

  double pos_T = 0.01;
  double ori_T = 0.3;
  double scale = 3.0;

  // Position
  double pos_traj_scale = 0.05;
  visualization_msgs::Marker pos_traj_marker;
  pos_traj_marker.header.frame_id = "world";
  pos_traj_marker.header.stamp = ros::Time::now();
  pos_traj_marker.id = 0;
  pos_traj_marker.ns = "planned_pos_traj";
  pos_traj_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  pos_traj_marker.scale.x = pos_traj_scale;
  pos_traj_marker.scale.y = pos_traj_scale;
  pos_traj_marker.scale.z = pos_traj_scale;
  pos_traj_marker.color.a = 1.0;
  pos_traj_marker.pose.orientation.w = 1.0;

  pos_traj_marker.action = visualization_msgs::Marker::DELETEALL;
  pos_traj_pub_.publish(pos_traj_marker);

  // Orientation
  if (!last_ori_marker_set_.markers.empty())
  {
    for (auto &marker : last_ori_marker_set_.markers)
      marker.action = visualization_msgs::Marker::DELETE;
    ori_traj_pub_.publish(last_ori_marker_set_);
  }

  int counter = 0;
  visualization_msgs::MarkerArray ori_traj_marker;

  if (last_position_traj_.getPieceNum() > 0)
  {
    geometry_msgs::Point point;
    double t_dura = (newest_traj_start_time_ - last_traj_start_time_).toSec();
    for (double t = 0.0; t < t_dura; t += pos_T)
    {
      Eigen::Vector3d X = last_position_traj_.getPos(t);
      double Vel = min(last_position_traj_.getVel(t).norm(), max_vel_)/max_vel_;

      point.x = X(0);
      point.y = X(1);
      point.z = X(2);
      Eigen::Vector3d rgb = jetColorMap(Vel);
      pos_traj_marker.color.r = rgb(0);
      pos_traj_marker.color.g = rgb(1);
      pos_traj_marker.color.b = rgb(2);

      pos_traj_marker.points.push_back(point);
      pos_traj_marker.colors.push_back(pos_traj_marker.color);
    }

    for (double t = 0.0; t < t_dura; t += ori_T)
    {
      Eigen::Vector3d X = last_position_traj_.getPos(t);

      double pitchAng = last_orientation_traj_.getPitch(t);
      double yawAng = last_orientation_traj_.getYaw(t);

      if (pitchAng > pitch_upper_ * M_PI / 180.0)  pitchAng = pitch_upper_ * M_PI / 180.0;
      if (pitchAng < pitch_lower_ * M_PI / 180.0)  pitchAng = pitch_lower_ * M_PI / 180.0;

      Eigen::Matrix3d Rwb_y, Rwb_p;
      Rwb_y << cos(yawAng), -sin(yawAng), 0.0, sin(yawAng), cos(yawAng), 0.0, 0.0, 0.0, 1.0;
      Rwb_p << cos(pitchAng), 0.0, -sin(pitchAng), 0.0, 1.0, 0.0, sin(pitchAng), 0.0, cos(pitchAng);

      Eigen::Vector3d X_orientation = X + Rwb_y*Rwb_p*Eigen::Vector3d(scale, 0.0, 0.0);

      visualization_msgs::Marker mk;
      mk.header.frame_id = "world";
      mk.header.stamp = ros::Time::now();
      mk.id = counter;
      mk.ns = "planned_ori_traj";
      mk.type = visualization_msgs::Marker::ARROW;
      mk.pose.orientation.w = 1.0;
      mk.scale.x = 0.2;
      mk.scale.y = 0.4;
      mk.scale.z = 0.3;
      mk.color.r = 0.5;
      mk.color.g = 0.5;
      mk.color.b = 0.4;
      mk.color.a = 1.0;

      geometry_msgs::Point pt_;
      pt_.x = X(0);
      pt_.y = X(1);
      pt_.z = X(2);
      mk.points.push_back(pt_);

      pt_.x = X_orientation(0);
      pt_.y = X_orientation(1);
      pt_.z = X_orientation(2);
      mk.points.push_back(pt_);

      ori_traj_marker.markers.push_back(mk);
      counter++;
    }
  }

  if (newest_position_traj_.getPieceNum() > 0)
  {
    geometry_msgs::Point point;
    for (double t = 0.0; t < newest_dura_; t += pos_T)
    {
      Eigen::Vector3d X = newest_position_traj_.getPos(t);
      double Vel = min(newest_position_traj_.getVel(t).norm(), max_vel_)/max_vel_;

      point.x = X(0);
      point.y = X(1);
      point.z = X(2);
      Eigen::Vector3d rgb = jetColorMap(Vel);
      pos_traj_marker.color.r = rgb(0);
      pos_traj_marker.color.g = rgb(1);
      pos_traj_marker.color.b = rgb(2);

      pos_traj_marker.points.push_back(point);
      pos_traj_marker.colors.push_back(pos_traj_marker.color);
    }

    for (double t = 0.0; t < newest_dura_; t += ori_T)
    {
      Eigen::Vector3d X = newest_position_traj_.getPos(t);

      double pitchAng = newest_orientation_traj_.getPitch(t);
      double yawAng = newest_orientation_traj_.getYaw(t);

      if (pitchAng > pitch_upper_ * M_PI / 180.0)  pitchAng = pitch_upper_ * M_PI / 180.0;
      if (pitchAng < pitch_lower_ * M_PI / 180.0)  pitchAng = pitch_lower_ * M_PI / 180.0;

      Eigen::Matrix3d Rwb_y, Rwb_p;
      Rwb_y << cos(yawAng), -sin(yawAng), 0.0, sin(yawAng), cos(yawAng), 0.0, 0.0, 0.0, 1.0;
      Rwb_p << cos(pitchAng), 0.0, -sin(pitchAng), 0.0, 1.0, 0.0, sin(pitchAng), 0.0, cos(pitchAng);

      Eigen::Vector3d X_orientation = X + Rwb_y*Rwb_p*Eigen::Vector3d(scale, 0.0, 0.0);

      visualization_msgs::Marker mk;
      mk.header.frame_id = "world";
      mk.header.stamp = ros::Time::now();
      mk.id = counter;
      mk.ns = "planned_ori_traj";
      mk.type = visualization_msgs::Marker::ARROW;
      mk.pose.orientation.w = 1.0;
      mk.scale.x = 0.2;
      mk.scale.y = 0.4;
      mk.scale.z = 0.3;
      mk.color.r = 0.5;
      mk.color.g = 0.5;
      mk.color.b = 0.4;
      mk.color.a = 1.0;

      geometry_msgs::Point pt_;
      pt_.x = X(0);
      pt_.y = X(1);
      pt_.z = X(2);
      mk.points.push_back(pt_);

      pt_.x = X_orientation(0);
      pt_.y = X_orientation(1);
      pt_.z = X_orientation(2);
      mk.points.push_back(pt_);

      ori_traj_marker.markers.push_back(mk);
      counter++;
    }
  }

  pos_traj_marker.action = visualization_msgs::Marker::ADD;
  pos_traj_pub_.publish(pos_traj_marker);

  ori_traj_pub_.publish(ori_traj_marker);
  last_ori_marker_set_ = ori_traj_marker;

  return;
}

void execTrajVis(vector<Eigen::VectorXd> cmd_path, vector<Eigen::Vector3d> odom_path)
{
  if (cmd_path.empty())
    return;

  nav_msgs::Path traj_msg_;
  traj_msg_.header.stamp = ros::Time::now();
  traj_msg_.header.frame_id = "world";

  visualization_msgs::MarkerArray oris;
  int counter = 0;
  double arrow_scale = 0.6, thresh_dist = 0.2;

  double accum_dist = thresh_dist + 0.1;
  Eigen::Vector3d last_point = cmd_path[0].head<3>();

  for (const auto& point : cmd_path)
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

    traj_msg_.poses.push_back(pose);

    accum_dist += (Eigen::Vector3d(point(0), point(1), point(2)) - last_point).norm();
    last_point = Eigen::Vector3d(point(0), point(1), point(2));

    if (accum_dist < thresh_dist) continue;
    accum_dist = 0.0;

    double pitch = point(3);
    double yaw = point(4);

    Eigen::Matrix3d Rwb_y, Rwb_p;
    Rwb_y << cos(yaw), -sin(yaw), 0.0, sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 1.0;
    Rwb_p << cos(pitch), 0.0, -sin(pitch), 0.0, 1.0, 0.0, sin(pitch), 0.0, cos(pitch);

    Eigen::Vector3d X(point(0), point(1), point(2));
    Eigen::Vector3d X_orientation = X + Rwb_y*Rwb_p*Eigen::Vector3d(arrow_scale, 0.0, 0.0);

    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = counter;
    mk.ns = "exec_ori";
    mk.type = visualization_msgs::Marker::ARROW;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = 0.06;
    mk.scale.y = 0.12;
    mk.scale.z = 0.09;
    mk.color.r = 1.0;
    mk.color.g = 0.0;
    mk.color.b = 0.0;
    mk.color.a = 1.0;

    geometry_msgs::Point pt_;
    pt_.x = X(0);
    pt_.y = X(1);
    pt_.z = X(2);
    mk.points.push_back(pt_);
    pt_.x = X_orientation(0);
    pt_.y = X_orientation(1);
    pt_.z = X_orientation(2);
    mk.points.push_back(pt_);
    oris.markers.push_back(mk);

    counter++;
  }

  exec_traj_pub_.publish(traj_msg_);
  exec_ori_pub_.publish(oris);

  if (odom_path.empty())
    return;

  nav_msgs::Path odom_msg_;
  odom_msg_.header.stamp = ros::Time::now();
  odom_msg_.header.frame_id = "world";

  for (const auto& point : odom_path)
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

    odom_msg_.poses.push_back(pose);
  }

  odom_traj_pub_.publish(odom_msg_);

  return;
}

void visCallback(const ros::TimerEvent &e)
{
  plannedTrajVis();
  execTrajVis(traj_cmds_, odom_traj_);

  pub_exec_id_++;
  
  return;
}

void publishGimbalCmd(const airsim_ros_pkgs::GimbalAngleEulerCmd& gimbal_cmd)
{
  gimbal_cmd_pub_.publish(gimbal_cmd);
}

void delayedPublishThread(const airsim_ros_pkgs::GimbalAngleEulerCmd& gimbal_cmd)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(500)); // ! AirSim Trick delay 500ms
  publishGimbalCmd(gimbal_cmd);
}

void cmdCallback(const ros::TimerEvent &e)
{
  if (!received_traj_)
    return;

  if (statistics_start_) ROS_INFO_THROTTLE(2.0, "\033[1m\033[37m\033[45m[TRAJ_SERVER] Flight Time : %.2f s, Flight Length : %.2f m, Flight Mean Vel : %.2f m/s.\033[7m\033[37m\033[45m", flight_time_, flight_length_, flight_mean_vel_);

  if (end_)
  {
    return; // back to hover
  }
  
  ros::Time cur_time = ros::Time::now();
  double t_old = (cur_time - last_traj_start_time_).toSec();
  double t_new = (cur_time - newest_traj_start_time_).toSec();

  Trajectory<7> cur_pos_traj, cur_ori_traj;
  double duration = 0.0;

  if (t_old > 0 && t_new < 0)
  {
    cur_pos_traj = last_position_traj_;
    cur_ori_traj = last_orientation_traj_;
    duration = last_dura_;
    span_ = t_old;
  }
  else if (t_new >= 0)
  {
    cur_pos_traj = newest_position_traj_;
    cur_ori_traj = newest_orientation_traj_;
    duration = newest_dura_;
    span_ = t_new;
  }
  else
  {
    return;
  }

  if (span_ > duration)
  {
    return; // back to hover
  }

  Eigen::Vector3d pos, vel, acc, jer;
  double pitch, yaw, yaw_vel;

  pos = cur_pos_traj.getPos(span_);
  vel = cur_pos_traj.getVel(span_);
  acc = cur_pos_traj.getAcc(span_);
  jer = cur_pos_traj.getJer(span_);
  pitch = cur_ori_traj.getPitch(span_);
  yaw = cur_ori_traj.getYaw(span_);
  yaw_vel = cur_ori_traj.getYawd(span_);

  if (pitch > pitch_upper_ * M_PI / 180.0)  pitch = pitch_upper_ * M_PI / 180.0;
  if (pitch < pitch_lower_ * M_PI / 180.0)  pitch = pitch_lower_ * M_PI / 180.0;

  Eigen::VectorXd pose_cmd(5);
  pose_cmd << pos(0), pos(1), pos(2), pitch, yaw;

  // add to traj_cmds_
  if (statistics_start_)
  {
    // statistics
    if (!traj_cmds_.empty())
    {
      flight_time_ += ctrl_interval_;
      flight_length_ += (pose_cmd.head(3)-last_pose.head(3)).norm();
      flight_mean_vel_ = flight_length_/flight_time_;
    }

    traj_cmds_.push_back(pose_cmd);

    // add to exec_traj_waypts_
    if (exec_traj_waypts_.empty())
    {
      exec_traj_waypts_.push_back(pose_cmd);
    }
    else
    {
      cur_interval += (pose_cmd.head(3)-last_pose.head(3)).norm();
      if (cur_interval > sample_dist_)
      {
        exec_traj_waypts_.push_back(pose_cmd);
        cur_interval = 0.0;
      }
    }
  }

  last_pose = pose_cmd;

  cmd_.header.stamp = cur_time;
  cmd_.position.x = pos(0);
  cmd_.position.y = pos(1);
  cmd_.position.z = pos(2);
  cmd_.velocity.x = vel(0);
  cmd_.velocity.y = vel(1);
  cmd_.velocity.z = vel(2);
  cmd_.acceleration.x = acc(0);
  cmd_.acceleration.y = acc(1);
  cmd_.acceleration.z = acc(2);
  cmd_.jerk.x = jer(0);
  cmd_.jerk.y = jer(1);
  cmd_.jerk.z = jer(2);
  cmd_.yaw = yaw;
  cmd_.yaw_dot = yaw_vel;
  // gimbal pitch
  gimbal_cmd_.header.stamp = cur_time;
  gimbal_cmd_.roll = 0.0;
  gimbal_cmd_.pitch = pitch*180.0/M_PI;
  gimbal_cmd_.yaw = 0.0;

  std::thread delayedThread(delayedPublishThread, gimbal_cmd_);
  delayedThread.detach();
  pos_cmd_pub_.publish(cmd_);
}

void execTrajWayptsCallback(const ros::TimerEvent &e)
{
  if (exec_traj_waypts_.empty())
    return;

  quadrotor_msgs::EigenVectorArray waypts_msg;
  waypts_msg.header.stamp = ros::Time::now();

  int cur_exec_num = (int)exec_traj_waypts_.size();
  for (int idx = exec_waypts_sent_num_; idx < (int)exec_traj_waypts_.size(); ++idx)
  {
    Eigen::VectorXd vec = exec_traj_waypts_[idx];
    std_msgs::Float64MultiArray array;
    for (int i = 0; i < (int)vec.size(); ++i)
    {
      array.data.push_back(vec[i]);
    }
    waypts_msg.vectors.push_back(array);
    already_pub_num_++;
  }

  if (exec_waypts_sent_num_ == cur_exec_num) return;

  exec_waypts_sent_num_ = cur_exec_num;

  exec_traj_waypts_pub_.publish(waypts_msg);

  return;
}

void robotVisCallback(const ros::TimerEvent &e)
{
  if (!received_traj_)
    return;

  ros::Time cur_time = end_ == true? end_time_ : ros::Time::now();
  double t_old = (cur_time - last_traj_start_time_).toSec();
  double t_new = (cur_time - newest_traj_start_time_).toSec();

  Trajectory<7> cur_pos_traj, cur_ori_traj;
  double duration = 0.0, t_cur = 0.0;

  if (t_old > 0 && t_new < 0)
  {
    cur_pos_traj = last_position_traj_;
    cur_ori_traj = last_orientation_traj_;
    duration = last_dura_;
    t_cur = t_old;
  }
  else if (t_new >= 0)
  {
    cur_pos_traj = newest_position_traj_;
    cur_ori_traj = newest_orientation_traj_;
    duration = newest_dura_;
    t_cur = t_new;
  }
  else
  {
    return;
  }

  if (t_cur > duration)
  {
    if (t_cur > 1e6)
      return;
    else
      t_cur = duration;
  }

  Eigen::Vector3d pos;
  double pitch, yaw;
  pos = cur_pos_traj.getPos(t_cur);
  pitch = cur_ori_traj.getPitch(t_cur);
  yaw = cur_ori_traj.getYaw(t_cur);

  if (pitch > pitch_upper_ * M_PI / 180.0)  pitch = pitch_upper_ * M_PI / 180.0;
  if (pitch < pitch_lower_ * M_PI / 180.0)  pitch = pitch_lower_ * M_PI / 180.0;

  // * Robot
  visualization_msgs::Marker cmd_vis;
  cmd_vis.header.frame_id = "world";
  cmd_vis.header.stamp = ros::Time::now();
  cmd_vis.id = 0;
  cmd_vis.ns = "current_robot_vis";
  cmd_vis.type = visualization_msgs::Marker::MESH_RESOURCE;
  cmd_vis.color.r = 0.0;
  cmd_vis.color.g = 0.0;
  cmd_vis.color.b = 0.0;
  cmd_vis.color.a = 1.0;
  cmd_vis.scale.x = 2*drone_radius_;
  cmd_vis.scale.y = 2*drone_radius_;
  cmd_vis.scale.z = 2*drone_radius_;
  cmd_vis.mesh_resource = "package://vis_utils/f250.dae";

  cmd_vis.action = visualization_msgs::Marker::DELETE;
  robot_vis_pub_.publish(cmd_vis);

  double yaw_drone = yaw - M_PI/2.0;
  double roll_odom = 0.0, pitch_odom = 0.0, yaw_odom = 0.0;
  tf::Quaternion q_odom;
  tf::quaternionMsgToTF(cur_odom_.pose.pose.orientation, q_odom);
  tf::Matrix3x3(q_odom).getRPY(roll_odom, pitch_odom, yaw_odom);

  tf::Quaternion q_vis;
  q_vis.setRPY(roll_odom, pitch_odom, yaw_drone);
  q_vis.normalize();

  cmd_vis.pose.orientation.x = q_vis.x();
  cmd_vis.pose.orientation.y = q_vis.y();
  cmd_vis.pose.orientation.z = q_vis.z();
  cmd_vis.pose.orientation.w = q_vis.w();
  cmd_vis.pose.position.x = pos(0);
  cmd_vis.pose.position.y = pos(1);
  cmd_vis.pose.position.z = pos(2);
  cmd_vis.action = visualization_msgs::Marker::ADD;
  robot_vis_pub_.publish(cmd_vis);

  // * Robot ellipsoid
  visualization_msgs::Marker ellipsoid;
  ellipsoid.header.frame_id = "world";
  ellipsoid.header.stamp = ros::Time::now();
  ellipsoid.id = 0;
  ellipsoid.ns = "current_robot_ellipsoid";
  ellipsoid.type = visualization_msgs::Marker::SPHERE;
  ellipsoid.color.r = 0.0;
  ellipsoid.color.g = 1.0;
  ellipsoid.color.b = 0.6;
  ellipsoid.color.a = 0.4;
  ellipsoid.scale.x = 3*drone_radius_;
  ellipsoid.scale.y = 3*drone_radius_;
  ellipsoid.scale.z = drone_radius_;

  ellipsoid.action = visualization_msgs::Marker::DELETE;
  robot_ellipsoid_pub_.publish(ellipsoid);

  ellipsoid.pose.orientation.w = 1.0;
  ellipsoid.pose.position.x = pos(0);
  ellipsoid.pose.position.y = pos(1);
  ellipsoid.pose.position.z = pos(2);
  ellipsoid.action = visualization_msgs::Marker::ADD;
  robot_ellipsoid_pub_.publish(ellipsoid);

  // * FoV
  double fov_scale = 0.03;
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 0;
  mk.ns = "current_FOV";
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.color.r = 0.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.color.a = 1.0;
  mk.scale.x = fov_scale;
  mk.scale.y = fov_scale;
  mk.scale.z = fov_scale;
  mk.pose.orientation.w = 1.0;

  mk.action = visualization_msgs::Marker::DELETE;
  fov_vis_pub_.publish(mk);

  percepUtils_->setPose_PY(pos, pitch, yaw);
  vector<Eigen::Vector3d> list1, list2;
  percepUtils_->getFOV_PY(list1, list2);

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
  fov_vis_pub_.publish(mk);

  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle nh("~");

  nh.param("traj_server/enable", traj_server_enable_, false);

  if (!traj_server_enable_)
  {
    ROS_INFO("\033[33m[Traj Server] ------------------------------> Disabled! \033[32m");
    ros::spin();
    return 0;
  }

  // Statistics initialization
  flight_time_ = 0.0;
  flight_length_ = 0.0;
  flight_mean_vel_ = 0.0;

  nh.param("traj_server/max_vel", max_vel_, -1.0);
  nh.param("traj_server/fov_h", fov_h_, -1.0);
  nh.param("traj_server/fov_w", fov_w_, -1.0);
  nh.param("traj_server/pitch_upper", pitch_upper_, -1.0);
  nh.param("traj_server/pitch_lower", pitch_lower_, -1.0);
  nh.param("traj_server/radius", drone_radius_, 0.3);
  ctrl_interval_ = 0.01;

  percepUtils_.reset(new PerceptionUtils);
  percepUtils_->init(nh);

  // Buffer
  int num = 20;
  buffer_odom_ = boost::circular_buffer<nav_msgs::OdometryConstPtr>(num);
  buffer_gimbal_ = boost::circular_buffer<geometry_msgs::QuaternionStampedConstPtr>(num);
  buffer_path_ = boost::circular_buffer<quadrotor_msgs::EigenVectorArray>(num);

  // Control parameter
  cmd_.kx = { 5.7, 5.7, 6.2 };
  cmd_.kv = { 3.4, 3.4, 4.0 };
  cmd_.header.stamp = ros::Time::now();
  cmd_.header.frame_id = "world";
  cmd_.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd_.trajectory_id = traj_id_;
  cmd_.velocity.x = 0.0;
  cmd_.velocity.y = 0.0;
  cmd_.velocity.z = 0.0;
  cmd_.acceleration.x = 0.0;
  cmd_.acceleration.y = 0.0;
  cmd_.acceleration.z = 0.0;
  cmd_.jerk.x = 0.0;
  cmd_.jerk.y = 0.0;
  cmd_.jerk.z = 0.0;
  cmd_.yaw = 0.0;
  cmd_.yaw_dot = 0.0;
  // gimbal
  gimbal_cmd_.header.stamp = ros::Time::now();
  gimbal_cmd_.camera_name = "front_center";
  gimbal_cmd_.vehicle_name = "drone_1";
  gimbal_cmd_.roll = 0.0;
  gimbal_cmd_.pitch = 0.0;
  gimbal_cmd_.yaw = 0.0;

  /* Odometry */
  odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/traj_server/odom", 50));
  gimbal_sub_.reset(new message_filters::Subscriber<geometry_msgs::QuaternionStamped>(nh, "/traj_server/gimbal", 50));
  sync_odom_gimbal_.reset(new message_filters::Synchronizer<SyncPolicyPoseGimbal>(SyncPolicyPoseGimbal(50), *odom_sub_, *gimbal_sub_));
  sync_odom_gimbal_->registerCallback(boost::bind(&odomCallback, _1, _2));

  last_traj_start_time_ = ros::Time(0.0);
  newest_traj_start_time_ = ros::Time(0.0);

  sample_dist_ = 0.3;

  pos_traj_pub_ = nh.advertise<visualization_msgs::Marker>("/traj_server/planned_pos_traj", 1);
  ori_traj_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/traj_server/planned_ori_traj", 1);
  exec_traj_pub_ = nh.advertise<nav_msgs::Path>("/traj_server/exec_traj", 1, true);
  exec_ori_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/traj_server/exec_ori", 1, true);
  odom_traj_pub_ = nh.advertise<nav_msgs::Path>("/traj_server/odom_traj", 1, true);
  exec_traj_waypts_pub_ = nh.advertise<quadrotor_msgs::EigenVectorArray>("/traj_server/exec_traj_waypts", 1);
  pos_cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("/traj_server/pos_cmd", 50);
  gimbal_cmd_pub_ = nh.advertise<airsim_ros_pkgs::GimbalAngleEulerCmd>("/traj_server/gimbal_cmd", 50);
  robot_vis_pub_ = nh.advertise<visualization_msgs::Marker>("/traj_server/robot_vis", 1);
  robot_ellipsoid_pub_ = nh.advertise<visualization_msgs::Marker>("/traj_server/robot_ellipsoid", 1);
  fov_vis_pub_ = nh.advertise<visualization_msgs::Marker>("/traj_server/fov_vis", 1);

  traj_server_vis_timer_ = nh.createTimer(ros::Duration(0.1), visCallback);
  cmd_timer_ = nh.createTimer(ros::Duration(ctrl_interval_), cmdCallback);
  exec_waypts_timer_ = nh.createTimer(ros::Duration(0.3), execTrajWayptsCallback);
  robot_vis_timer_ = nh.createTimer(ros::Duration(0.1), robotVisCallback);

  traj_sub_ = nh.subscribe("/local_newest_traj", 1, trajCallback);
  finish_sub_ = nh.subscribe("/flight_end", 1, finishCallback);
  brake_sub_ = nh.subscribe("/emergency_brake", 1, brakeCallback);
  path_sub_ = nh.subscribe("/local_newest_path", 1, pathCallback);
  demo_command_sub_ = nh.subscribe("/fc_vision/demo_command", 10, demoCommandCallback);

  ros::Duration(1.0).sleep();

  ROS_INFO("\033[33m[Traj Server] ------------------------------> Ready! \033[32m");
  ros::spin();
  
  return 0;
}
