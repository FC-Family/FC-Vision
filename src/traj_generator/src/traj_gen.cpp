/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the main algorithm of trajectory generation in 
 *                   in FC-Planner. Note: We specially add hard constraints on 
 *                   viewpoint pose while optimizing all remaining waypoints.
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

#include "traj_generator/traj_gen.h"

namespace fc_vision
{
  TrajGenerator::TrajGenerator(){
  }

  TrajGenerator::~TrajGenerator(){
  }

  void TrajGenerator::init(ros::NodeHandle& nh)
  {
    // * Module Initialization
    vis_utils_.reset(new PlanningVisualization(nh));
    visFlag = false;
    traj_vis_timer_ = nh.createTimer(ros::Duration(1), &TrajGenerator::trajVisCallback, this);

    // * Params Initialization
    nh.param("traj_gen/vmax_",    this->vel_bound, -1.0);
    nh.param("traj_gen/amax_",    this->acc_bound, -1.0);
    nh.param("traj_gen/jmax_",    this->jerk_bound, -1.0);
    nh.param("traj_gen/ydmax_",   this->yawd_bound, -1.0);
    nh.param("traj_gen/safe_inf", this->safe_inf_, 0.0);
    nh.param("traj_gen/rho_T",    this->rho_T, 0.0);
    nh.param("traj_gen/rho_RT",   this->rho_RT, 0.0);
    nh.param("traj_gen/rho_j",    this->rho_j, 0.0);
    nh.param("traj_gen/rho_a",    this->rho_a, 0.0);
    nh.param("traj_gen/rho_v",    this->rho_v, 0.0);
    nh.param("traj_gen/rho_p",    this->rho_p, 0.0);
    nh.param("traj_gen/rho_Rp",   this->rho_Rp, 0.0);
    nh.param("traj_gen/rho_e",    this->rho_e, 0.0);

    // * Visualization
    routePub = nh.advertise<visualization_msgs::Marker>("/hcoppTraj/route", 1);
    wayPointsPub = nh.advertise<visualization_msgs::Marker>("/hcoppTraj/waypoints", 1);
    appliedTrajectoryPub = nh.advertise<visualization_msgs::Marker>("/hcoppTraj/applied_trajectory", 1);
    textPub = nh.advertise<visualization_msgs::MarkerArray>("/hcoppTraj/drone_status", 1);
    PitchPub = nh.advertise<visualization_msgs::MarkerArray>("/hcoppTraj/PitchState", 1);
    YawPub = nh.advertise<visualization_msgs::MarkerArray>("/hcoppTraj/YawState", 1);
    orientation_pub = nh.advertise<visualization_msgs::MarkerArray>("/hcoppTraj/GimbalState", 1);
    input_path_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/hcoppTraj/input_path", 1);

    return;
  }

  void TrajGenerator::reset()
  {
    this->velBound.clear();
    this->accBound.clear();
    this->jerBound.clear();
    this->waypts_.clear();
    this->waypt_indi_.clear();
    this->angle_indi_.clear();
    this->pieces_for_orient_.clear();
    this->view_idx_.clear();
    this->route.clear();
    this->pitchRoute.clear();
    this->yawRoute.clear();

    return;
  }

  void TrajGenerator::setMap(shared_ptr<SDFMap>& map)
  {
    this->mapping_utils_ = map;

    return;
  }

  void TrajGenerator::trajGen(vector<Eigen::VectorXd>& ctrl_pts, vector<bool> waypt_indi, double coeff, const Eigen::Vector3d vel, const Eigen::Vector3d acc, const Eigen::Vector3d pyd, const Eigen::Vector3d pyd_dot)
  {
    Eigen::Vector3d now_posi = ctrl_pts.front().head(3);
    vector<Eigen::Vector3d> wps;
    vector<bool> wps_waypt_indicators;
    vector<double> pitchs, yaws;

    wps.resize((int)ctrl_pts.size()-1);
    wps_waypt_indicators.resize((int)ctrl_pts.size()-1);
    pitchs.resize((int)ctrl_pts.size());
    yaws.resize((int)ctrl_pts.size());

    for (int i=0; i<(int)ctrl_pts.size(); ++i)
    {
      if (i > 0)
      {
        wps[i-1] = ctrl_pts[i].head(3);
        wps_waypt_indicators[i-1] = waypt_indi[i];
      }  
      pitchs[i] = ctrl_pts[i](3);
      yaws[i] = ctrl_pts[i](4);
    }

    this->run(now_posi, wps, wps_waypt_indicators, pitchs, yaws, 1.7, vel, acc, pyd, pyd_dot);

    return;
  }

  /**
   * @brief Main function for trajectory optimization using MINCO representation.
   *
   * @param waypts      [N-1] points without start.
   * @param waypt_indi  [N-1] indicators without start, true for waypoint, false for viewpoint.
   * @param given_pitch [N] pitchs.
   * @param given_yaw   [N] yaws.
   */
  void TrajGenerator::run(Eigen::Vector3d& now_odom, vector<Eigen::Vector3d>& waypts, vector<bool>& waypt_indi, vector<double>& given_pitch, vector<double>& given_yaw, double coeff, const Eigen::Vector3d vel, const Eigen::Vector3d acc, const Eigen::Vector3d pyd, const Eigen::Vector3d pyd_dot)
  {
    auto hctraj_t1 = std::chrono::high_resolution_clock::now();
    
    this->traj_opt_suc_ = true;

    this->pos_vel_ = vel;
    this->pos_acc_ = acc;
    this->py_vel_ = pyd;
    this->py_acc_ = pyd_dot;

    vector<Eigen::Vector3d> allwaypts;
    allwaypts.push_back(now_odom); allwaypts.insert(allwaypts.end(), waypts.begin(), waypts.end());

    this->waypts_.push_back(now_odom);
    this->waypts_.insert(this->waypts_.end(), waypts.begin(), waypts.end());
    this->waypt_indi_.push_back(false);
    this->waypt_indi_.insert(this->waypt_indi_.end(), waypt_indi.begin(), waypt_indi.end());
    this->angle_indi_ = vector<bool>(this->waypts_.size()+1, false);

    vector<double> updated_pitch, updated_yaw;
    updatePitchYaw(given_pitch, given_yaw, updated_pitch, updated_yaw);
    dynPieceBound(allwaypts, updated_pitch, updated_yaw, coeff);
    wpsTraj(opt_wps, opt_inistate, opt_finstate, now_odom, waypts, waypt_indi, false);

    if (!this->traj_opt_suc_) return;

    positionTrajOpt();
    orientationTrajOpt(updated_pitch, updated_yaw);
    
    auto hctraj_t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> hctraj_ms = hctraj_t2 - hctraj_t1;
    double hctraj_time = (double)hctraj_ms.count();
    ROS_INFO("\033[34m[Traj] Trajectory Generation latency = %lf ms. \033[32m", hctraj_time);

    return;
  }

  void TrajGenerator::wpsTraj(Eigen::MatrixXd &wps,Eigen::Matrix3d& iniState, Eigen::Matrix3d& finState, const Eigen::Vector3d& now_odom, std::vector<Eigen::Vector3d> &given_wps, vector<bool>& given_indi, bool add_odom)
  {
    if (given_wps.size() <= 1 )
    {
      ROS_ERROR("TrajGenerator: Given waypoints are not enough! ");
      wps.resize(3, 1);
      wps.col(0) = now_odom;
      iniState << now_odom, this->pos_vel_, this->pos_acc_;
      finState << now_odom, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

      this->traj_opt_suc_ = false;
    }
    else
    {
      wps.resize(3, int(given_wps.size())-1);
      opt_indi.clear();
      opt_indi.resize(given_indi.size()-1, false);
      for (int i=0; i<int(given_wps.size())-1; i++)
      {
        if (given_indi[i] == true)
          opt_indi[i] = true;
        else
          opt_indi[i] = false;
        
        if (add_odom)
        {
          wps(0,i) = given_wps[i](0) + now_odom(0);
          wps(1,i) = given_wps[i](1) + now_odom(1);
          wps(2,i) = given_wps[i](2);
        }
        else
        {
          wps(0,i) = given_wps[i](0);
          wps(1,i) = given_wps[i](1);
          wps(2,i) = given_wps[i](2);
        }
      }
      iniState << now_odom, this->pos_vel_, this->pos_acc_;
      finState << given_wps.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
      this->start_ = now_odom;
      this->end_ = given_wps.back();
    }

    return;
  }

  /** 
  @brief N points including M waypoints and N-M viewpoints
          optimize M waypoints and freeze N-M viewpoints
   */
  void TrajGenerator::positionTrajOpt()
  { 
    Eigen::SparseMatrix<double> onehot((int)opt_wps.cols(), (int)opt_wps.cols());
    onehot.setIdentity();
    
    waypt_count = count(opt_indi.begin(), opt_indi.end(), true);
    select_waypt.resize((int)opt_wps.cols(), waypt_count);
    select_viewpt.resize((int)opt_wps.cols(), (int)opt_wps.cols()-waypt_count);
    way_wps.resize(3, waypt_count);
    view_wps.resize(3, (int)opt_wps.cols()-waypt_count);
  
    int index1 = 0, index2 = 0;
    for (int i=0; i<(int)opt_indi.size(); ++i)
    {
      if (opt_indi[i] == true)
      {
        select_waypt.col(index1) = onehot.col(i);
        way_wps.col(index1) = opt_wps.col(i);
        index1++;
      }
      else
      {
        select_viewpt.col(index2) = onehot.col(i);
        view_wps.col(index2) = opt_wps.col(i);
        index2++;
      }
    }
    
    if(opt_wps.cols()<2)
    {
      ROS_ERROR("TrajGenerator: Not enough dimension of optimization!");
      this->traj_opt_suc_ = false;
      return;   
    }

    piece_nums = opt_wps.cols() + 1;
    /* max vel, acc, and jerk */
    max_v_squared = vel_bound * vel_bound;
    max_a_squared = acc_bound * acc_bound;
    max_j_squared = jerk_bound * jerk_bound;
    /* set initial Time */
    double coeff = 2.0;
    opt_times.resize(piece_nums);
    opt_times[0]=((opt_inistate.col(0) - opt_wps.col(0)).norm()+0.1)*coeff / vel_bound;
    for (int i=1; i<opt_wps.cols(); i++)
    {
      opt_times[i]=((opt_wps.col(i) - opt_wps.col(i-1)).norm()+0.1)*coeff / vel_bound;
    }
    opt_times[piece_nums-1]=((opt_finstate.col(0) - opt_wps.col(opt_wps.cols()-1)).norm()+0.1)*coeff / vel_bound;

    double traj_time_init = 0.0;
    for (int i=0; i<(int)opt_times.size(); ++i)
      traj_time_init += opt_times(i);

    minco_anal.setConditions(opt_inistate, opt_finstate, piece_nums);

    /* set initial tau */
    Eigen::VectorXd x, init_tau, waypt_vec;
    int opt_dim = piece_nums + 3*waypt_count;
    x.resize(opt_dim);
    backwardT(opt_times,init_tau);
    waypt_vec = Eigen::Map<Eigen::VectorXd>(way_wps.data(), way_wps.size());
    x.segment(0, piece_nums) = init_tau;
    x.segment(piece_nums, 3*waypt_count) = waypt_vec;

    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs_params.mem_size = 32;
    lbfgs_params.past = 3;
    lbfgs_params.g_epsilon = 0;
    lbfgs_params.min_step = 1.0e-32;
    lbfgs_params.delta = 1.0e-5;
    lbfgs_params.max_linesearch = 64;
    lbfgs_params.max_iterations = 200;
    double inner_cost;
    
    int result = lbfgs::lbfgs_optimize(x, inner_cost, &innerCallback, nullptr, nullptr, this, lbfgs_params);

    if (result == lbfgs::LBFGS_CONVERGENCE ||
        result == lbfgs::LBFGS_CANCELED ||
        result == lbfgs::LBFGS_STOP ||
        result == lbfgs::LBFGSERR_MAXIMUMITERATION)
    {
      ROS_INFO("\033[32m[TrajAnalyzer] traj optimization success! cost = %d.\033[32m", (int)inner_cost);
    }
    else if (result == lbfgs::LBFGSERR_MAXIMUMLINESEARCH)
    {
      ROS_WARN("[TrajAnalyzer] The line-search routine reaches the maximum number of evaluations.");
    }
    else
    {
      ROS_ERROR("[TrajAnalyzer] Solver error. Return = %d, %s.", result, lbfgs::lbfgs_strerror(result));
    }

    Trajectory<7> temp_traj;
    minco_anal.getTrajectory(temp_traj);
    minco_traj = temp_traj;

    if (minco_traj.getPieceNum())
    {
      double length = 0.0, T = 0.01;
      Eigen::Vector3d lastX = minco_traj.getPos(0.0);
      for (double t = T; t < minco_traj.getTotalDuration(); t += T)
      {
        Eigen::Vector3d X = minco_traj.getPos(t);
        length += (X-lastX).norm();
        lastX = X;
      }
      int highVelNum = 0, totalNum = 0;
      double max_vel = min(minco_traj.getMaxVelRate(), vel_bound);
      for (double t = 0.0; t < minco_traj.getTotalDuration(); t += T)
      {
        if (minco_traj.getVel(t).norm() > 0.8*max_vel)
          highVelNum++;
        totalNum++;
      }
      double highVelRate = (double)highVelNum/(double)totalNum;
      double max_jerk = -1.0;
      for (double t = 0.0; t < minco_traj.getTotalDuration(); t += T)
      {
        double jerk = minco_traj.getJer(t).norm();
        if (jerk > max_jerk)
          max_jerk = jerk;
      }

      stringstream stream_vel, stream_acc, stream_jer, stream_time, stream_length, stream_highVelRate;
      double max_vel_rate = minco_traj.getMaxVelRate();
      double max_acc_rate = minco_traj.getMaxAccRate();
      double max_jer_rate = max_jerk;
      double total_time = minco_traj.getTotalDuration();
      double traj_length = length;
      double high_vel_rate = highVelRate*100.0;

      stream_vel << fixed << setprecision(2) << max_vel_rate;
      stream_acc << fixed << setprecision(2) << max_acc_rate;
      stream_jer << fixed << setprecision(2) << max_jer_rate;
      stream_time << fixed << setprecision(2) << total_time;
      stream_length << fixed << setprecision(2) << traj_length;
      stream_highVelRate << fixed << setprecision(2) << high_vel_rate;

      string vel_str = stream_vel.str();
      string acc_str = stream_acc.str();
      string jer_str = stream_jer.str();
      string time_str = stream_time.str();
      string length_str = stream_length.str();
      string highVelRate_str = stream_highVelRate.str();

      ROS_INFO("\033[32m[TrajAnalyzer] traj max vel               = %s m/s.\033[32m", vel_str.c_str());
      ROS_INFO("\033[32m[TrajAnalyzer] traj max acc               = %s m/s^2.\033[32m", acc_str.c_str());
      ROS_INFO("\033[32m[TrajAnalyzer] traj max jer               = %s m/s^3.\033[32m", jer_str.c_str());
      ROS_INFO("\033[32m[TrajAnalyzer] traj exec time             = %s s.\033[32m", time_str.c_str());
      ROS_INFO("\033[32m[TrajAnalyzer] traj length                = %s m.\033[32m", length_str.c_str());
      ROS_INFO("\033[32m[TrajAnalyzer] traj high vel rate         = %s %%.\033[32m", highVelRate_str.c_str());
    
      // print each piece time
      // Eigen::VectorXd dur = minco_traj.getDurations();
      // for (int i=0; i<dur.size(); ++i)
      // {
      //   ROS_INFO("\033[32m[TrajAnalyzer] Piece %d time = %lf s.\033[32m", i, dur(i));
      // }
    }

    return;
  }

  /**
   @brief p: [Pitch, Yaw, 0], v: [ω_θ, ω_ψ, 0], a: [α_θ, α_ψ, 0] 
          incrementally update angle in the path -> avoid singularity 
          core function : getUpdatedAngle(pre_angle, cur_angle, updated_angle)
   */
  void TrajGenerator::orientationTrajOpt(vector<double>& given_pitch, vector<double>& given_yaw)
  {
    this->findFeasiblePitchYaw(given_pitch, given_yaw, opt_times_Gimbal);
    
    this->pitchRoute.clear(); this->yawRoute.clear();
    this->pitchRoute = given_pitch;
    this->yawRoute = given_yaw;
    Eigen::Matrix3d iniStateGimbal, finStateGimbal;
    Eigen::MatrixXd wpsGimbal;

    iniStateGimbal << Eigen::Vector3d(given_pitch.front(), given_yaw.front(), 0.0), this->py_vel_, this->py_acc_;

    int pieceNUM = (int)opt_times_Gimbal.size();
    wpsGimbal.resize(3, int(this->pitchRoute.size())-1);

    for (int i=0; i<(int)this->pitchRoute.size()-1; ++i)
    {
      wpsGimbal(0, i) = this->pitchRoute[i+1];
      wpsGimbal(1, i) = this->yawRoute[i+1];
      wpsGimbal(2, i) = 0.0;
    }

    finStateGimbal << Eigen::Vector3d(wpsGimbal(0, wpsGimbal.cols()-1), wpsGimbal(1, wpsGimbal.cols()-1), 0.0), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

    // present pitch and yaw
    // for (int i=0; i<(int)this->pitchRoute.size(); ++i)
    // {
    //   double angle_dur = i < 1 ? 0.0 : opt_times_Gimbal[i-1];
    //   ROS_INFO("\033[33m[TrajAnalyzer] pitch = %lf deg, yaw = %lf deg, dur = %lf s.\033[32m", this->pitchRoute[i]*180.0/M_PI, this->yawRoute[i]*180.0/M_PI, angle_dur);
    // }

    /* MINCO */
    minco_anal.setConditions(iniStateGimbal, finStateGimbal, pieceNUM);
    minco_anal.setParameters(wpsGimbal, opt_times_Gimbal);
    Trajectory<7> temp_ori_traj;
    minco_anal.getTrajectory(temp_ori_traj);
    minco_orientation_traj = temp_ori_traj;

    double max_omega_pitch = -1.0, max_omega_yaw = -1.0;
    max_omega_pitch  = minco_orientation_traj.getMaxPitchd();
    max_omega_yaw = minco_orientation_traj.getMaxYawd();

    stringstream stream_pitch, stream_yaw, stream_duration;
    double pitch_max = 180.0*max_omega_pitch/M_PI;
    double yaw_max = 180.0*max_omega_yaw/M_PI;
    double ori_duration = minco_orientation_traj.getTotalDuration();

    stream_pitch << fixed << setprecision(2) << pitch_max;
    stream_yaw << fixed << setprecision(2) << yaw_max;
    stream_duration << fixed << setprecision(2) << ori_duration;

    string pitch_max_str = stream_pitch.str();
    string yaw_max_str = stream_yaw.str();
    string ori_duration_str = stream_duration.str();

    ROS_INFO("\033[32m[TrajAnalyzer] traj max pitch angular vel = %s deg/s.\033[32m", pitch_max_str.c_str());
    ROS_INFO("\033[32m[TrajAnalyzer] traj max yaw angular vel   = %s deg/s.\033[32m", yaw_max_str.c_str());
    ROS_INFO("\033[32m[TrajAnalyzer] orientation traj exec time = %s s.\033[32m", ori_duration_str.c_str());

    return;
  }

  double TrajGenerator::innerCallback(void* ptrObj, const Eigen::VectorXd& x, Eigen::VectorXd& grad)
  {
    TrajGenerator &obj = *(TrajGenerator *)ptrObj;
    /* get optimization variables : virual time vector -> τ & waypoints -> q */
    Eigen::Map<const Eigen::MatrixXd> tau(x.data(), obj.piece_nums, 1);
    Eigen::Map<const Eigen::MatrixXd> wps(x.data() + obj.piece_nums, 3, obj.waypt_count);
    Eigen::Map<Eigen::VectorXd> grad_tau(grad.data(), obj.piece_nums);
    Eigen::Map<Eigen::MatrixXd> grad_wps(grad.data() + obj.piece_nums, 3, obj.waypt_count);
    grad_tau.setZero();
    grad_wps.setZero();

    /* get T from τ, generate MINCO trajectory */
    Eigen::VectorXd Txy;
    obj.forwardT(tau, Txy);
    obj.fused_wps = wps*obj.select_waypt.transpose() + obj.view_wps*obj.select_viewpt.transpose();
    obj.minco_anal.setParameters(obj.fused_wps, Txy);

    /* get snap cost (energy) with grad (C,T) */
    double snap_cost = 0.0;
    Eigen::MatrixX3d gdCxy_snap;
    Eigen::VectorXd gdTxy_snap;
    obj.minco_anal.getEnergy(snap_cost);
    snap_cost = obj.rho_e*snap_cost;
    obj.minco_anal.getEnergyPartialGradByCoeffs(gdCxy_snap);  
    gdCxy_snap = obj.rho_e*gdCxy_snap;
    obj.minco_anal.getEnergyPartialGradByTimes(gdTxy_snap);  
    gdTxy_snap = obj.rho_e*gdTxy_snap;

    /* get constrain cost with grad (C,T), safety & feasibility */
    double constrain_cost = 0.0;
    Eigen::MatrixXd gdCxy_constrain;
    Eigen::VectorXd gdTxy_constrain;
    obj.calConstrainCostGrad(constrain_cost, gdCxy_constrain, gdTxy_constrain);

    /* get grad (q, T) from (C, T) */
    Eigen::MatrixXd gdCxy = gdCxy_snap + gdCxy_constrain;
    Eigen::VectorXd gdTxy = gdTxy_snap + gdTxy_constrain;
    Eigen::Matrix3Xd gradPxy_temp;
    Eigen::VectorXd gradTxy_temp;
    obj.minco_anal.propogateGrad(gdCxy, gdTxy, gradPxy_temp, gradTxy_temp);
    
    /* get tau cost with grad (τ) */
    double tau_cost = obj.rho_T * Txy.sum();
    gradTxy_temp.array() += obj.rho_T;
    obj.backwardGradT(tau, gradTxy_temp, grad_tau);

    /* add regularization term for even distance distribution of waypoints */
    int inner_num = obj.fused_wps.cols() + 1;
    double waypoint_regularization_cost = 0.0;
    Eigen::MatrixXd waypoint_regularization_grad = Eigen::MatrixXd::Zero(3, obj.fused_wps.cols());

    Eigen::VectorXd distances(inner_num);
    distances(0) = (obj.fused_wps.col(0) - obj.start_).norm();
    for (int i=0; i<inner_num-2; ++i)
    {
      distances(i+1) = (obj.fused_wps.col(i+1) - obj.fused_wps.col(i)).norm();
    }
    distances(inner_num-1) = (obj.end_ - obj.fused_wps.col(inner_num-2)).norm();
    double mean_distance = distances.mean();
    
    double diff_start = distances(0) - mean_distance;
    waypoint_regularization_cost += diff_start * diff_start;
    Eigen::Vector3d grad_diff_start = 2.0 * diff_start * (obj.fused_wps.col(0) - obj.start_).normalized();
    waypoint_regularization_grad.col(0) += grad_diff_start;

    for (int i=0; i<inner_num-2; ++i)
    {
      double diff = distances(i+1) - mean_distance;
      waypoint_regularization_cost += diff * diff;

      Eigen::Vector3d grad_diff = 2.0 * diff * (obj.fused_wps.col(i + 1) - obj.fused_wps.col(i)).normalized();
      waypoint_regularization_grad.col(i) -= grad_diff;
      waypoint_regularization_grad.col(i + 1) += grad_diff;
    }

    double diff_end = distances(inner_num-1) - mean_distance;
    waypoint_regularization_cost += diff_end * diff_end;
    Eigen::Vector3d grad_diff_end = 2.0 * diff_end * (obj.end_ - obj.fused_wps.col(inner_num-2)).normalized();
    waypoint_regularization_grad.col(inner_num-2) -= grad_diff_end;

    waypoint_regularization_cost *= obj.rho_Rp;
    waypoint_regularization_grad *= obj.rho_Rp;

    /* update grad of waypoints q */
    grad_wps = (gradPxy_temp + waypoint_regularization_grad)*obj.select_waypt;

    /* add regularization term for even time allocation */
    double mean_tau = tau.mean();
    double regularization_cost = obj.rho_RT * (tau.array() - mean_tau).square().sum();
    Eigen::VectorXd regularization_grad = obj.rho_RT * 2 * (tau.array() - mean_tau).matrix();

    /* update grad of virtual time vector τ --> T */
    grad_tau += regularization_grad;

    // * Total cost
    double total_cost = snap_cost + constrain_cost + tau_cost + regularization_cost + waypoint_regularization_cost;

    return total_cost;
  }
  
  void TrajGenerator::dynPieceBound(vector<Eigen::Vector3d>& waypts, vector<double>& given_pitch, vector<double>& given_yaw, double angleCoeff)
  {
    double pos_time, yaw_time, pitch_time, bound_time;
    double bound_velocity;
    int pieces = (int)waypts.size()-1;
    vector<bool> under_flag(pieces, false);
    this->velBound.clear();
    this->velBound.resize(pieces);
    
    for (int i=0; i<(int)this->pieces_for_orient_.size(); ++i)
    {
      pos_time = 0.0; yaw_time = 0.0; pitch_time = 0.0, bound_time = 0.0;

      auto min_piece_it = min_element(this->pieces_for_orient_[i].begin(), this->pieces_for_orient_[i].end());
      auto max_piece_it = max_element(this->pieces_for_orient_[i].begin(), this->pieces_for_orient_[i].end());
      int min_piece = static_cast<int>(*min_piece_it);
      int max_piece = static_cast<int>(*max_piece_it);

      int min_idx = min_piece;
      int max_idx = max_piece+1;

      double length = 0.0;
      for (int j=min_idx; j<max_idx; ++j)
      {
        length += (waypts[j+1]-waypts[j]).norm();
      }
      pos_time = length / vel_bound;
      yaw_time = angleCoeff*min(abs(given_yaw[i+1]-given_yaw[i]), 2*M_PI-abs(given_yaw[i+1]-given_yaw[i]))/yawd_bound;
      pitch_time = angleCoeff*min(abs(given_pitch[i+1]-given_pitch[i]), 2*M_PI-abs(given_pitch[i+1]-given_pitch[i]))/yawd_bound;
      bound_time = max(max(pos_time, yaw_time), pitch_time);

      bound_velocity = min(vel_bound, length / bound_time);

      for (auto x:this->pieces_for_orient_[i])
      {
        velBound[x] = bound_velocity;
        if (bound_velocity < vel_bound)
          under_flag[x] = true;
      }
    }

    // * Add velocity bound smoothing
    for (int i=0; i<pieces; ++i)
    {
      if (under_flag[i] == true)
      {
        if (i-1>0)
        {
          if (under_flag[i-1] == false)
            velBound[i-1] = 0.5*(velBound[i-1] + velBound[i]);
          else
            velBound[i-1] = min(0.5*(velBound[i-1] + velBound[i]), velBound[i-1]);
        }
        
        if (i+1<pieces)
        {
          if (under_flag[i+1] == false)
            velBound[i+1] = 0.5*(velBound[i] + velBound[i+1]);
          else
            velBound[i+1] = min(0.5*(velBound[i] + velBound[i+1]), velBound[i+1]);
        }
      }
    }

    return;
  }

  void TrajGenerator::visualize(const Trajectory<7>& appliedTraj, ros::Time timeStamp, double compT)
  {
    visualization_msgs::Marker routeMarker, wayPointsMarker, appliedTrajMarker;
    visualization_msgs::MarkerArray Order;
    double max_vel = appliedTraj.getMaxVelRate();

    double waypt_scale = 0.1;
    double traj_scale = 0.05;

    routeMarker.id = 0;
    routeMarker.type = visualization_msgs::Marker::LINE_LIST;
    routeMarker.header.stamp = timeStamp;
    routeMarker.header.frame_id = "world";
    routeMarker.pose.orientation.w = 1.00;
    routeMarker.action = visualization_msgs::Marker::ADD;
    routeMarker.ns = "route";
    routeMarker.color.r = 1.00;
    routeMarker.color.g = 0.00;
    routeMarker.color.b = 0.00;
    routeMarker.color.a = 1.00;
    routeMarker.scale.x = 0.4;

    wayPointsMarker = routeMarker;
    wayPointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
    wayPointsMarker.ns = "waypoints";
    wayPointsMarker.color.r = 0.00;
    wayPointsMarker.color.g = 1.00;
    wayPointsMarker.color.b = 0.00;
    wayPointsMarker.scale.x = waypt_scale;
    wayPointsMarker.scale.y = waypt_scale;
    wayPointsMarker.scale.z = waypt_scale;

    appliedTrajMarker.header.frame_id = "world";
    appliedTrajMarker.header.stamp = ros::Time::now();
    appliedTrajMarker.id = 0;
    appliedTrajMarker.ns = "applied_trajectory";
    appliedTrajMarker.type = visualization_msgs::Marker::SPHERE_LIST;
    appliedTrajMarker.action = visualization_msgs::Marker::ADD;
    appliedTrajMarker.scale.x = traj_scale;
    appliedTrajMarker.scale.y = traj_scale;
    appliedTrajMarker.scale.z = traj_scale;
    appliedTrajMarker.pose.orientation.w = 1.0;

    Eigen::MatrixXd route = appliedTraj.getPositions();

    if (route.cols() > 0)
    {
      bool first = true;
      Eigen::Vector3d last;
      for (int i = 0; i < route.cols(); i++)
      {
        if (first)
        {
          first = false;
          last = route.col(i);
          continue;
        }

        geometry_msgs::Point point;

        point.x = last(0);
        point.y = last(1);
        point.z = last(2);
        routeMarker.points.push_back(point);
        point.x = route.col(i)(0);
        point.y = route.col(i)(1);
        point.z = route.col(i)(2);
        routeMarker.points.push_back(point);
        last = route.col(i);
      }

      routePub.publish(routeMarker);
    }

    if (route.cols() > 0)
    {
      for (int i = 0; i < route.cols(); i++)
      {
        geometry_msgs::Point point;
        point.x = route.col(i)(0);
        point.y = route.col(i)(1);
        point.z = route.col(i)(2);
        wayPointsMarker.points.push_back(point);
      }

      wayPointsPub.publish(wayPointsMarker);
    }

    if (appliedTraj.getPieceNum() > 0)
    {
      double T = 0.01;
      geometry_msgs::Point point;
      for (double t = 0.0; t < appliedTraj.getTotalDuration(); t += T)
      {
        Eigen::Vector3d X = appliedTraj.getPos(t);
        double Vel = appliedTraj.getVel(t).norm()/max_vel;
        
        point.x = X(0);
        point.y = X(1);
        point.z = X(2);
        Eigen::Vector3d rgb = jetColorMap(Vel);
        appliedTrajMarker.color.r = rgb(0);
        appliedTrajMarker.color.g = rgb(1);
        appliedTrajMarker.color.b = rgb(2);
        appliedTrajMarker.color.a = 1.0;

        appliedTrajMarker.points.push_back(point);
        appliedTrajMarker.colors.push_back(appliedTrajMarker.color);
      }
      appliedTrajectoryPub.publish(appliedTrajMarker);
    }

    if (route.cols() > 0)
    {
      int count = 0;
      for (int i=0; i< route.cols(); ++i)
      {
        visualization_msgs::Marker textMarker;
        textMarker.header.frame_id = "world";
        textMarker.header.stamp = timeStamp;
        textMarker.ns = "text";
        textMarker.id = count;
        textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        textMarker.action = visualization_msgs::Marker::ADD;

        textMarker.pose.position.x = route.col(i)(0);
        textMarker.pose.position.y = route.col(i)(1);
        textMarker.pose.position.z = route.col(i)(2);
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
        textMarker.text = to_string(i);
        
        Order.markers.push_back(textMarker);
        count++;
      }

      textPub.publish(Order);
    }

    return;
  }

  void TrajGenerator::visualizePitch(const Trajectory<7>& appliedTraj, ros::Time timeStamp, double compT)
  {
    visualization_msgs::MarkerArray pitch_traj;
    int counter = 0;
    double scale = 3.0; 
    double T = 0.3;
    for (double t = 0.0; t < appliedTraj.getTotalDuration(); t += T)
    {
      Eigen::Vector3d X = minco_traj.getPos(t);
      double pitchAng = appliedTraj.getPitch(t);

      visualization_msgs::Marker mk;
      mk.header.frame_id = "world";
      mk.header.stamp = ros::Time::now();
      mk.id = counter;
      mk.ns = "pitch_traj";
      mk.type = visualization_msgs::Marker::ARROW;
      mk.pose.orientation.w = 1.0;
      mk.scale.x = 0.2;
      mk.scale.y = 0.4;
      mk.scale.z = 0.3;
      mk.color.r = 0.8;
      mk.color.g = 0.0;
      mk.color.b = 0.4;
      mk.color.a = 1.0;

      geometry_msgs::Point pt_;
      pt_.x = X(0);
      pt_.y = X(1);
      pt_.z = X(2);
      mk.points.push_back(pt_);

      pt_.x = X(0) + scale*cos(pitchAng);
      pt_.y = X(1);
      pt_.z = X(2) + scale*sin(pitchAng);
      mk.points.push_back(pt_);

      pitch_traj.markers.push_back(mk);
      counter++;
    }

    PitchPub.publish(pitch_traj);

    return;
  }

  void TrajGenerator::visualizeYaw(const Trajectory<7>& appliedTraj, ros::Time timeStamp, double compT)
  {
    visualization_msgs::MarkerArray yaw_traj;
    int counter = 0;
    double scale = 3.0; 
    double T = 0.2;
    for (double t = 0.0; t < appliedTraj.getTotalDuration(); t += T)
    {
      Eigen::Vector3d X = minco_traj.getPos(t);
      double yawAng = appliedTraj.getYaw(t);

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
      mk.color.g = 0.8;
      mk.color.b = 0.4;
      mk.color.a = 1.0;

      geometry_msgs::Point pt_;
      pt_.x = X(0);
      pt_.y = X(1);
      pt_.z = X(2);
      mk.points.push_back(pt_);

      pt_.x = X(0) + scale*cos(yawAng);
      pt_.y = X(1) + scale*sin(yawAng);
      pt_.z = X(2);
      mk.points.push_back(pt_);

      yaw_traj.markers.push_back(mk);
      counter++;
    }

    YawPub.publish(yaw_traj);

    return;
  } 

  void TrajGenerator::visualizeGimbal(const Trajectory<7>& appliedTraj, ros::Time timeStamp, double compT)
  {
    visualization_msgs::MarkerArray orientation_traj;
    int counter = 0;
    double scale = 3.0; 
    double T = 0.5;
    double duration = appliedTraj.getTotalDuration();

    for (double t = 0.0; t <= duration; t += T)
    {
      Eigen::Vector3d X = minco_traj.getPos(t);

      double pitchAng = appliedTraj.getPitch(t);
      double yawAng = appliedTraj.getYaw(t);

      Eigen::Matrix3d Rwb_y, Rwb_p;
      Rwb_y << cos(yawAng), -sin(yawAng), 0.0, sin(yawAng), cos(yawAng), 0.0, 0.0, 0.0, 1.0;
      Rwb_p << cos(pitchAng), 0.0, -sin(pitchAng), 0.0, 1.0, 0.0, sin(pitchAng), 0.0, cos(pitchAng);

      Eigen::Vector3d X_orientation = X + Rwb_y*Rwb_p*Eigen::Vector3d(scale, 0.0, 0.0);

      visualization_msgs::Marker mk;
      mk.header.frame_id = "world";
      mk.header.stamp = ros::Time::now();
      mk.id = counter;
      mk.ns = "orientation_traj";
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

      orientation_traj.markers.push_back(mk);
      counter++;
    }

    orientation_pub.publish(orientation_traj);

    return;
  }

  void TrajGenerator::visualizeInputPath(const vector<Eigen::Vector3d>& waypts, const vector<bool>& indi)
  {
    if (waypts.empty()) return;
    
    visualization_msgs::MarkerArray path_marker;

    int counter = 0;

    if ((int)waypts.size() > 1)
    {

    visualization_msgs::Marker path;
    path.header.frame_id = "world";
    path.header.stamp = ros::Time::now();
    path.ns = "input_path";
    path.id = counter;
    path.type = visualization_msgs::Marker::LINE_LIST;
    path.action = visualization_msgs::Marker::ADD;
    path.pose.orientation.w = 1.0;
    path.scale.x = 0.1;
    path.color.a = 0.5;
    path.color.r = 0.0;
    path.color.g = 0.0;
    path.color.b = 0.0;
    for (int i=0; i<(int)waypts.size()-1; ++i)
    {
      geometry_msgs::Point p1, p2;
      p1.x = waypts[i](0);
      p1.y = waypts[i](1);
      p1.z = waypts[i](2);
      p2.x = waypts[i+1](0);
      p2.y = waypts[i+1](1);
      p2.z = waypts[i+1](2);
      path.points.push_back(p1);
      path.points.push_back(p2);
    }
    path_marker.markers.push_back(path);
    counter++;

    }

    // ! red for waypoints
    // ? blue for fixed-waypoints
    for (int i=0; i<(int)waypts.size(); ++i)
    {
      visualization_msgs::Marker wp;
      wp.header.frame_id = "world";
      wp.header.stamp = ros::Time::now();
      wp.ns = "input_waypoints";
      wp.id = counter;
      wp.type = visualization_msgs::Marker::SPHERE;
      wp.action = visualization_msgs::Marker::ADD;
      wp.pose.position.x = waypts[i](0);
      wp.pose.position.y = waypts[i](1);
      wp.pose.position.z = waypts[i](2);
      wp.pose.orientation.w = 1.0;
      wp.scale.x = 0.3;
      wp.scale.y = 0.3;
      wp.scale.z = 0.3;
      wp.color.a = 1.0;
      wp.color.r = indi[i] == true ? 1.0 : 0.0;
      wp.color.g = 0.0;
      wp.color.b = indi[i] == false ? 1.0 : 0.0;
      path_marker.markers.push_back(wp);
      counter++;
    }

    input_path_pub_.publish(path_marker);

    return;
  }

  void TrajGenerator::trajVisCallback(const ros::TimerEvent& e)
  {
    if (visFlag == true)
    {

    if (!this->waypts_.empty())
      visualizeInputPath(this->waypts_, this->waypt_indi_);
    if (minco_traj.getTotalDuration() > 0.0)
      visualize(minco_traj, ros::Time::now(), 0.0);
    if (minco_orientation_traj.getTotalDuration() > 0.0)
    {
      visualizePitch(minco_orientation_traj, ros::Time::now(), 0.0);
      visualizeYaw(minco_orientation_traj, ros::Time::now(), 0.0);
      visualizeGimbal(minco_orientation_traj, ros::Time::now(), 0.0);
    }
    if ((int)route.size() > 0) vis_utils_->publishYawTraj(route, yawRoute);
    }

    return;
  }
  // ! ------------------------------------- Utils -------------------------------------
  Eigen::Vector3d TrajGenerator::jetColorMap(double value)
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

  void TrajGenerator::updatePitchYaw(vector<double>& given_pitch, vector<double>& given_yaw, vector<double>& new_pitch, vector<double>& new_yaw)
  {
    vector<double> view_pitchs, view_yaws;
    for (int i=0; i<(int)given_pitch.size(); ++i)
    {
      if (this->angle_indi_[i] == false)
      {
        view_pitchs.push_back(given_pitch[i]);
        view_yaws.push_back(given_yaw[i]);
        this->view_idx_.push_back(i);
      }
    }

    for (int j=0; j<(int)this->view_idx_.size()-1; ++j)
    {
      int diff = this->view_idx_[j+1] - this->view_idx_[j];
      vector<int> pieces;
      for (int k=0; k<diff; ++k)
      {
        pieces.push_back(this->view_idx_[j]+k);
      }
      this->pieces_for_orient_.push_back(pieces);
    }
    
    this->pitchRoute = view_pitchs;
    this->yawRoute = view_yaws;

    new_pitch.push_back(view_pitchs.front());
    new_yaw.push_back(view_yaws.front());

    double updated_pitch_front = 0.0, updated_yaw_front = 0.0;
    getUpdatedAngle(pitchRoute.front(), pitchRoute[1], updated_pitch_front);
    getUpdatedAngle(yawRoute.front(), yawRoute[1], updated_yaw_front);
    new_pitch.push_back(updated_pitch_front);
    new_yaw.push_back(updated_yaw_front);

    for (int i=1; i<(int)pitchRoute.size()-1; ++i)
    {
      double updated_pitch = 0.0, updated_yaw = 0.0;
      getUpdatedAngle(new_pitch.back(), pitchRoute[i+1], updated_pitch);
      getUpdatedAngle(new_yaw.back(), yawRoute[i+1], updated_yaw);
      
      new_pitch.push_back(updated_pitch);
      new_yaw.push_back(updated_yaw);
    }

    return;
  }
}