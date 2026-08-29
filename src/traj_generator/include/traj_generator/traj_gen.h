/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the header file of TrajGenerator class, which implements
 *                   visibility-aware trajectory generation.
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

#ifndef _TRAJ_GEN_H_
#define _TRAJ_GEN_H_

#include "quadrotor_msgs/PolynomialTraj.h"
#include "utils/gcopter/gcopter.hpp"
#include "utils/gcopter/minco.hpp"
#include "utils/gcopter/lbfgs_new.hpp"
#include "vis_utils/planning_visualization.h"
#include "plan_env/sdf_map.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <iomanip>
#include <sstream>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <chrono>

using namespace std;
using std::shared_ptr;

namespace fc_vision
{

class PlanningVisualization;
class SDFMap;

class TrajGenerator
{
public:

  TrajGenerator();
  ~TrajGenerator();
  /* Func */
  void init(ros::NodeHandle& nh);
  void reset();
  void setMap(shared_ptr<SDFMap>& map);
  void trajGen(vector<Eigen::VectorXd>& ctrl_pts, vector<bool> waypt_indi, double coeff, const Eigen::Vector3d vel, const Eigen::Vector3d acc, const Eigen::Vector3d pyd, const Eigen::Vector3d pyd_dot);
  bool visFlag;
  /* Results */
  Trajectory<7> minco_traj, minco_orientation_traj;
  
private:
  /* Func */
  void wpsTraj(Eigen::MatrixXd &wps,Eigen::Matrix3d& iniState, Eigen::Matrix3d& finState, const Eigen::Vector3d& now_odom, std::vector<Eigen::Vector3d> &given_wps, vector<bool>& given_indi, bool add_odom);
  void positionTrajOpt();
  void orientationTrajOpt(vector<double>& given_pitch, vector<double>& given_yaw);
  void run(Eigen::Vector3d& now_odom, vector<Eigen::Vector3d>& waypts, vector<bool>& waypt_indi, vector<double>& given_pitch, vector<double>& given_yaw, double coeff, const Eigen::Vector3d vel, const Eigen::Vector3d acc, const Eigen::Vector3d pyd, const Eigen::Vector3d pyd_dot);
  /* Param */
  Eigen::Vector3d pos_vel_, pos_acc_;
  Eigen::Vector3d py_vel_, py_acc_;
  Eigen::Vector3d curOdom;
  Eigen::MatrixXd opt_wps, way_wps, view_wps, fused_wps;
  vector<bool> opt_indi;
  Eigen::SparseMatrix<double> select_waypt, select_viewpt;
  int piece_nums, waypt_count;
  Eigen::Matrix3d opt_inistate, opt_finstate;
  Eigen::VectorXd opt_times;
  double rho_T,rho_RT,rho_p,rho_Rp,rho_a,rho_v,rho_j, rho_e, safe_inf_;
  Eigen::Vector3d start_, end_;
  double vel_bound, acc_bound, jerk_bound, yawd_bound, max_v_squared, max_a_squared, max_j_squared;
  Eigen::VectorXd opt_times_Gimbal;
  /* Data */
  vector<double> velBound, accBound, jerBound;
  vector<Eigen::Vector3d> waypts_;
  vector<bool> waypt_indi_, angle_indi_;
  vector<vector<int>> pieces_for_orient_;
  vector<int> view_idx_;
  vector<Eigen::Vector3d> route;
  vector<double> pitchRoute;
  vector<double> yawRoute;
  int pieceN;
  bool traj_opt_suc_ = true;
  /* Vis */
  ros::Publisher routePub;
  ros::Publisher wayPointsPub;
  ros::Publisher appliedTrajectoryPub;
  ros::Publisher textPub;
  ros::Publisher PitchPub;
  ros::Publisher YawPub;
  ros::Publisher orientation_pub;
  ros::Publisher input_path_pub_;
  /* Tools */
  void dynPieceBound(vector<Eigen::Vector3d>& waypts, vector<double>& given_pitch, vector<double>& given_yaw, double angleCoeff);
  Eigen::Vector3d jetColorMap(double value);
  void getUpdatedAngle(const double& pre, const double& cur, double& update_cur);
  void updatePitchYaw(vector<double>& given_pitch, vector<double>& given_yaw, vector<double>& new_pitch, vector<double>& new_yaw);
  void findFeasiblePitchYaw(vector<double>& given_pitch, vector<double>& given_yaw, Eigen::VectorXd& opt_durs);
  /* Utils */
  minco::MINCO_S4NU minco_anal;
  shared_ptr<PlanningVisualization> vis_utils_ = nullptr;
  shared_ptr<SDFMap> mapping_utils_ = nullptr;
  /* Vis */
  void trajVisCallback(const ros::TimerEvent& e);
  void visualize(const Trajectory<7>& appliedTraj, ros::Time timeStamp, double compT);
  void visualizePitch(const Trajectory<7>& appliedTraj, ros::Time timeStamp, double compT);
  void visualizeYaw(const Trajectory<7>& appliedTraj, ros::Time timeStamp, double compT);
  void visualizeGimbal(const Trajectory<7>& appliedTraj, ros::Time timeStamp, double compT);
  void visualizeInputPath(const vector<Eigen::Vector3d>& waypts, const vector<bool>& indi);

  static double innerCallback(void* ptrObj, const Eigen::VectorXd& x, Eigen::VectorXd& grad);
            
  void calConstrainCostGrad(double& cost, Eigen::MatrixXd& gdCxy, Eigen::VectorXd &gdTxy)
  {
    cost = 0.0;
    gdCxy.resize(8*piece_nums, 3);
    gdCxy.setZero();
    gdTxy.resize(piece_nums);
    gdTxy.setZero();

    Eigen::Vector3d totalGradPos, totalGradVel, totalGradAcc, totalGradJer;

    Eigen::Vector3d pos, vel, acc, jer, sna, outerNormal;
    Eigen::Vector3d grad_z = Eigen::Vector3d::Zero();
    Eigen::Vector3d grad_p = Eigen::Vector3d::Zero();
    Eigen::Vector3d grad_v = Eigen::Vector3d::Zero();
    Eigen::Vector3d grad_a = Eigen::Vector3d::Zero();
    Eigen::Vector3d grad_j = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 8, 1> beta0_xy, beta1_xy, beta2_xy, beta3_xy, beta4_xy;
    double s1, s2, s3, s4, s5, s6, s7;
    double step, alpha, omg, pena, smoothFactor;
    double violaPosPena, violaPosPenaD;
    double violaVelPena, violaVelPenaD;
    double violaAccPena, violaAccPenaD;
    double violaJerPena, violaJerPenaD;
    double outerNormalNorm;
    smoothFactor = 1e-2;

    Eigen::Vector3d gradPos, gradVel, gradAcc, gradJer;

    int int_K = 256;
    const double integralFrac = 1.0 / int_K;
    for (int i=0; i<piece_nums; i++)
    {
      const Eigen::Matrix<double, 8, 3> &c_xy = minco_anal.getCoeffs().block<8, 3>(i * 8, 0);
      step = minco_anal.T1(i) * integralFrac;
      s1 = 0.0;

      max_v_squared = velBound[i]*velBound[i];

      for (int j=0; j<=int_K; j++)
      {
        // analyse xy
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        s6 = s4 * s2;
        s7 = s4 * s3;
        beta0_xy << 1.0, s1, s2, s3, s4, s5, s6, s7;
        beta1_xy << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4, 6.0 * s5, 7.0 * s6;
        beta2_xy << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3, 30.0 * s4, 42.0 * s5;
        beta3_xy << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2, 120.0 * s3, 210.0 * s4;
        beta4_xy << 0.0, 0.0, 0.0, 0.0, 24.0, 120 * s1, 360.0 * s2, 840.0 * s3;
        pos = c_xy.transpose() * beta0_xy;
        vel = c_xy.transpose() * beta1_xy;
        acc = c_xy.transpose() * beta2_xy;
        jer = c_xy.transpose() * beta3_xy;
        sna = c_xy.transpose() * beta4_xy;

        pena = 0.0;
        gradPos.setZero();
        gradVel.setZero();
        gradAcc.setZero();
        gradJer.setZero();

        omg = (j == 0 || j == int_K) ? 0.5 : 1.0;

        double v_snorm = vel.squaredNorm();
        double a_snorm = acc.squaredNorm();
        double j_snorm = jer.squaredNorm();

        // pos : ESDF constraint
        Eigen::Vector3d esdf_grad;
        double esdf_v = this->mapping_utils_->getDistWithGrad(pos, esdf_grad);
        double violaPos = safe_inf_ - esdf_v;
        if (smoothedL1(violaPos, smoothFactor, violaPosPena, violaPosPenaD))
        {
          gradPos += rho_p * violaPosPenaD * (-esdf_grad);
          pena += rho_p * violaPosPena;
        }

        // vel : orientation angle constraint
        double vViola = v_snorm - max_v_squared;
        if (smoothedL1(vViola, smoothFactor, violaVelPena, violaVelPenaD))
        {
          gradVel += rho_v * violaVelPenaD * 2.0 * vel;
          pena += rho_v * violaVelPena;
        }

        // acc : constant bound constraint
        double aViola = a_snorm - max_a_squared;
        if (smoothedL1(aViola, smoothFactor, violaAccPena, violaAccPenaD)) 
        {
          gradAcc += rho_a * violaAccPenaD * 2.0 * acc;
          pena += rho_a * violaAccPena;
        }

        // jer : constant bound constraint
        double jViola = j_snorm - max_j_squared;
        if (smoothedL1(jViola, smoothFactor, violaJerPena, violaJerPenaD)) 
        {
          gradJer += rho_j * violaJerPenaD * 2.0 * jer;
          pena += rho_j * violaJerPena;
        }

        totalGradPos = gradPos;
        totalGradVel = gradVel;
        totalGradAcc = gradAcc;
        totalGradJer = gradJer;

        // add all grad into C,T
        // note that xy = Cxy*β(j/K*T_xy), yaw = Cyaw*β(i*T_xy+j/K*T_xy-yaw_idx*T_yaw)
        // ∂p/∂Cxy, ∂v/∂Cxy, ∂a/∂Cxy
        alpha = j * integralFrac;
        gdCxy.block<8, 3>(i * 8, 0) += (beta0_xy * totalGradPos.transpose() + 
                                        beta1_xy * totalGradVel.transpose() + 
                                        beta2_xy * totalGradAcc.transpose() + 
                                        beta3_xy * totalGradJer.transpose()) * omg * step;
        // ∂p/∂Txy, ∂v/∂Txy, ∂a/∂Txy
        gdTxy(i) += (totalGradPos.dot(vel) +
                                 totalGradVel.dot(acc) +
                                 totalGradAcc.dot(jer) +
                                 totalGradJer.dot(sna)) *
                                    alpha * omg * step +
                                omg * integralFrac * pena;
        
        cost += omg * step * pena;
        s1 += step;
      }
    }
  }
  // T = e^τ
  double expC2(const double& tau)
  {
    return tau > 0.0 ? ((0.5 * tau + 1.0) * tau + 1.0) : 1.0 / ((0.5 * tau - 1.0) * tau + 1.0);
  }
  // τ = ln(T)
  double logC2(const double& T)
  {
    return T > 1.0 ? (sqrt(2.0 * T - 1.0) - 1.0) : (1.0 - sqrt(2.0 / T - 1.0));
  }

  bool smoothedL1(const double &x, const double &mu, double &f, double &df)
  {
    if (x < 0.0)
    {
      return false;
    }
    else if (x > mu)
    {
      f = x - 0.5 * mu;
      df = 1.0;
      return true;
    }
    else
    {
      const double xdmu = x / mu;
      const double sqrxdmu = xdmu * xdmu;
      const double mumxd2 = mu - 0.5 * x;
      f = mumxd2 * sqrxdmu * xdmu;
      df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
      return true;
    }
  }

  static inline void forwardT(const Eigen::VectorXd &tau, Eigen::VectorXd &T)
  {
    const int sizeTau = tau.size();
    T.resize(sizeTau);
    for (int i = 0; i < sizeTau; i++)
    {
        T(i) = tau(i) > 0.0
                    ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
                    : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
    }
    return;
  }

  template <typename EIGENVEC>
  static inline void backwardT(const Eigen::VectorXd &T, EIGENVEC &tau)
  {
    const int sizeT = T.size();
    tau.resize(sizeT);
    for (int i = 0; i < sizeT; i++)
    {
        tau(i) = T(i) > 1.0
                        ? (sqrt(2.0 * T(i) - 1.0) - 1.0)
                        : (1.0 - sqrt(2.0 / T(i) - 1.0));
    }
    return;
  }

  template <typename EIGENVEC>
  static inline void backwardGradT(const Eigen::VectorXd &tau, const Eigen::VectorXd &gradT, EIGENVEC &gradTau)
  {
    const int sizeTau = tau.size();
    gradTau.resize(sizeTau);
    double denSqrt;
    for (int i = 0; i < sizeTau; i++)
    {
        if (tau(i) > 0)
        {
            gradTau(i) = gradT(i) * (tau(i) + 1.0);
        }
        else
        {
            denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
            gradTau(i) = gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);
        }
    }
    return;
  }
  /* Timer */
  ros::Timer traj_vis_timer_;

};

inline void TrajGenerator::getUpdatedAngle(const double& pre, const double& cur, double& update_cur)
{
  double gap = cur - pre;
  while (gap > M_PI)
    gap -= 2*M_PI;
  while (gap < -M_PI)
    gap += 2*M_PI;

  update_cur = pre + gap;
}

inline void TrajGenerator::findFeasiblePitchYaw(vector<double>& given_pitch, vector<double>& given_yaw, Eigen::VectorXd& opt_durs)
{
  Eigen::Matrix3d iniStateGimbal, finStateGimbal;
  Eigen::MatrixXd wpsGimbal;

  iniStateGimbal << Eigen::Vector3d(given_pitch.front(), given_yaw.front(), 0.0), this->py_vel_, this->py_acc_;
  Eigen::VectorXd Duration = minco_traj.getDurations();
  int pos_pieceNUM = minco_traj.getPieceNum();

  int pieceNUM = (int)this->pieces_for_orient_.size();
  opt_durs.resize(pieceNUM);
  for (int i=0; i<pieceNUM; ++i)
  {
    double time = 0.0;
    for (auto x:this->pieces_for_orient_[i])
      time += Duration[x];
    
    opt_durs[i] = time;
  }
  wpsGimbal.resize(3, int(given_pitch.size())-1);

  for (int i=0; i<(int)given_pitch.size()-1; ++i)
  {
    wpsGimbal(0, i) = given_pitch[i+1];
    wpsGimbal(1, i) = given_yaw[i+1];
    wpsGimbal(2, i) = 0.0;
  }

  double min_dur = 1.0;
  if ((int)opt_durs.size() <= 2)
  {
    min_dur = minco_traj.getTotalDuration() / pos_pieceNUM;
  }
  else
  {
    min_dur = opt_durs.minCoeff();
  }
  min_dur = min(min_dur, 1.0);

  finStateGimbal << Eigen::Vector3d(wpsGimbal(0, wpsGimbal.cols()-1), wpsGimbal(1, wpsGimbal.cols()-1), 0.0), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

  minco_anal.setConditions(iniStateGimbal, finStateGimbal, pieceNUM);
  minco_anal.setParameters(wpsGimbal, opt_durs);
  Trajectory<7> temp_ori_traj;
  minco_anal.getTrajectory(temp_ori_traj);

  double max_omega_pitch = -1.0, max_omega_yaw = -1.0;
  max_omega_pitch  = temp_ori_traj.getMaxPitchd();
  max_omega_yaw = temp_ori_traj.getMaxYawd();

  if (max_omega_pitch > 0.7*this->yawd_bound || max_omega_yaw > 0.7*this->yawd_bound)
  {
    ROS_WARN("Pitch and Yaw rate exceed the limit, re-optimizing...");
    vector<double> new_pitch, new_yaw, new_durs;

    for (int i=0; i<(int)given_pitch.size()-1; ++i)
    {
      new_pitch.push_back(given_pitch[i]);
      new_yaw.push_back(given_yaw[i]);

      double cur_piece_dur = opt_durs[i];
      if (cur_piece_dur > 1.5*min_dur)
      {
        int num = floor(cur_piece_dur/min_dur);
        double interval_dur = cur_piece_dur/num;

        double head_pitch = given_pitch[i], head_yaw = given_yaw[i];
        double tail_pitch = given_pitch[i+1], tail_yaw = given_yaw[i+1];
        for (int j=1; j<num; ++j)
        {
          double inter_pitch, inter_yaw;
          inter_pitch = head_pitch + (tail_pitch - head_pitch) * j / num;
          inter_yaw = head_yaw + (tail_yaw - head_yaw) * j / num;

          new_pitch.push_back(inter_pitch);
          new_yaw.push_back(inter_yaw);
          new_durs.push_back(interval_dur);
        }
        new_durs.push_back(interval_dur);
      }
      else
      {
        new_durs.push_back(cur_piece_dur);
      }
    }
    new_pitch.push_back(given_pitch.back());
    new_yaw.push_back(given_yaw.back());

    Eigen::VectorXd new_dur_vec;
    new_dur_vec.resize(new_durs.size());
    for (int i=0; i<(int)new_durs.size(); ++i)
      new_dur_vec[i] = new_durs[i];
    
    given_pitch = new_pitch;
    given_yaw = new_yaw;
    opt_durs = new_dur_vec;
  }

  return;
}

}

#endif
