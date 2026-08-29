/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the main file of airsim simulator core functions.
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

#include <airsim_ros_wrapper.h>
#include <boost/make_shared.hpp>
#include "common/AirSimSettings.hpp"
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <algorithm>
#include <cmath>
constexpr char AirsimROSWrapper::CAM_YML_NAME[];
constexpr char AirsimROSWrapper::WIDTH_YML_NAME[];
constexpr char AirsimROSWrapper::HEIGHT_YML_NAME[];
constexpr char AirsimROSWrapper::K_YML_NAME[];
constexpr char AirsimROSWrapper::D_YML_NAME[];
constexpr char AirsimROSWrapper::R_YML_NAME[];
constexpr char AirsimROSWrapper::P_YML_NAME[];
constexpr char AirsimROSWrapper::DMODEL_YML_NAME[];

const std::unordered_map<int, std::string> AirsimROSWrapper::image_type_int_to_string_map_ = {
    { 0, "Scene" },
    { 1, "DepthPlanner" },
    { 2, "DepthPerspective" },
    { 3, "DepthVis" },
    { 4, "DisparityNormalized" },
    { 5, "Segmentation" },
    { 6, "SurfaceNormals" },
    { 7, "Infrared" }
};

AirsimROSWrapper::AirsimROSWrapper(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, const std::string& host_ip) : 
    nh_(nh), 
    nh_private_(nh_private),
    img_async_spinner_(1, &img_timer_cb_queue_), // a thread for image callbacks to be 'spun' by img_async_spinner_ 
    lidar_async_spinner_(1, &lidar_timer_cb_queue_), // same as above, but for lidar
    traj_async_spinner_(1, &traj_timer_cb_queue_),
    host_ip_(host_ip),
    airsim_client_images_(host_ip),
    airsim_client_lidar_(host_ip),
    airsim_client_traj_(host_ip),
    tf_listener_(tf_buffer_)
{
    ros_clock_.clock.fromSec(0);
    is_used_lidar_timer_cb_queue_ = false;
    is_used_img_timer_cb_queue_ = false;
    is_used_traj_timer_cb_queue_ = false;

    if (AirSimSettings::singleton().simmode_name != "Car")
    {
        airsim_mode_ = AIRSIM_MODE::DRONE;
        ROS_INFO("Setting ROS wrapper to DRONE mode");
    }
    else
    {
        airsim_mode_ = AIRSIM_MODE::CAR;
        ROS_INFO("Setting ROS wrapper to CAR mode");
    }
    
    initialize_ros();

    std::cout << "AirsimROSWrapper Initialized!\n";
}

void AirsimROSWrapper::initialize_airsim()
{
    // todo do not reset if already in air?
    try
    {
        
        if (airsim_mode_ == AIRSIM_MODE::DRONE)
        {
            airsim_client_ = std::unique_ptr<msr::airlib::RpcLibClientBase>(new msr::airlib::MultirotorRpcLibClient(host_ip_));
        }
        else
        {
            airsim_client_ = std::move(std::unique_ptr<msr::airlib::RpcLibClientBase>(new msr::airlib::CarRpcLibClient(host_ip_)));
        }
        airsim_client_->confirmConnection();
        airsim_client_images_.confirmConnection();
        airsim_client_lidar_.confirmConnection();
        airsim_client_traj_.confirmConnection();

        for (const auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_)
        {
            airsim_client_->enableApiControl(true, vehicle_name_ptr_pair.first); // todo expose as rosservice?
            airsim_client_->armDisarm(true, vehicle_name_ptr_pair.first); // todo exposes as rosservice?
        }

/*----------------------takeoff function------------------------------*/
        static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->takeoffAsync(20, "drone_1");

//         static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())
//     ->takeoffAsync(20, "drone_1")->waitOnLastTask();

// float takeoff_height = 0.4f;   // 想要的高度（0.4 m）

// msr::airlib::YawMode yaw_mode(false, 90.0f);  // 不转向，yaw = 0

// static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())
//     ->moveToZAsync(
//         -takeoff_height,   // z (NED，向上为负)
//         1.0f,              // velocity
//         20.0f,             // timeout_sec
//         yaw_mode,          // yaw_mode
//         -1.0f,             // lookahead (保持默认习惯用 -1)
//         1.0f,              // adaptive_lookahead
//         "drone_1"          // vehicle_name
//     )->waitOnLastTask();

/*-----This part below is added by myself, which function is used to publish the global map in the formation of pointcloud---------*/

        if(acquire_origin_z)  //This part is used to make the drone_1 tf just on the ground
        {
            auto get_original_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
            msr::airlib::MultirotorState original_drone_state;
            original_drone_state = get_original_client->getMultirotorState("drone_1");
            origin_z = original_drone_state.getPosition().z();
            acquire_origin_z = false;
        }
        ros::Duration(3.0).sleep();


        // sensor_msgs::PointCloud2 globalMap_pcd;
        // pcl::PointCloud<pcl::PointXYZ> cloudMap;
        // Eigen::Matrix3d enutoned;
        // enutoned << 0,1,0,
        //             1,0,0,
        //             0,0,-1;
        // vector<string> objects_list;
        // for(int k=0;k<1;k++)
        // {
        //     cloudMap.points.clear();
        //     objects_list = airsim_client_->simListSceneObjects("Cube.*");
        //     for(int i=0; i<objects_list.size(); i++)
        //     {
        //         msr::airlib::Pose cube_pose;
        //         msr::airlib::Vector3r cube_scale;
        //         string object_name;
        //         object_name = objects_list[i];
        //         cube_pose = airsim_client_->simGetObjectPose(object_name);
        //         cube_scale = airsim_client_->simGetObjectScale(object_name);
        //         Eigen::Vector3d position;
        //         Eigen::Quaterniond q;
        //         Eigen::Matrix3d body2world;
        //         q.w() = cube_pose.orientation.w();
        //         q.x() = cube_pose.orientation.x();
        //         q.y() = cube_pose.orientation.y();
        //         q.z() = cube_pose.orientation.z();
        //         position<<cube_pose.position.x(), cube_pose.position.y(), cube_pose.position.z() - origin_z;
        //         body2world = q.toRotationMatrix();
        //         if(isENU_)
        //         {
        //             position<<cube_pose.position.y(),cube_pose.position.x(),-cube_pose.position.z() + origin_z;
        //             body2world = enutoned*q.toRotationMatrix();
        //         }
        //         double lx, ly, lz;
        //         for(lx = -cube_scale.x()/2; lx<cube_scale.x()/2+resolution_;lx+=resolution_)
        //         {
        //             for(ly = -cube_scale.y()/2; ly<cube_scale.y()/2+resolution_;ly+=resolution_)
        //             {
        //                 for(lz = -cube_scale.z()/2; lz<cube_scale.z()/2+resolution_;lz+=resolution_)
        //                 {
        //                     Eigen::Vector3d obs_body;
        //                     obs_body << lx,ly,lz;
        //                     Eigen::Vector3d obs_world;
        //                     obs_world = body2world*obs_body+position;
        //                     pcl::PointXYZ pt;
        //                     pt.x = obs_world[0];
        //                     pt.y = obs_world[1];
        //                     pt.z = obs_world[2];
        //                     cloudMap.points.push_back(pt); 
        //                 }
        //             }
        //         }
        //         cloudMap.width = cloudMap.points.size();
        //         cloudMap.height = 1;
        //         cloudMap.is_dense = true;
        //         pcl::toROSMsg(cloudMap, globalMap_pcd);
        //         globalMap_pcd.header.frame_id = world_frame_id_; 
        //         airsim_map_pub.publish(globalMap_pcd);
        //     }
        // }


/*********************************************************************************************************/

        origin_geo_point_ = airsim_client_->getHomeGeoPoint("");
        // todo there's only one global origin geopoint for environment. but airsim API accept a parameter vehicle_name? inside carsimpawnapi.cpp, there's a geopoint being assigned in the constructor. by? 
        origin_geo_point_msg_ = get_gps_msg_from_airsim_geo_point(origin_geo_point_);
    }
    catch (rpc::rpc_error&  e)
    {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
    }
}

void AirsimROSWrapper::initialize_ros()
{

    // ros params
    double update_airsim_control_every_n_sec;
    nh_private_.getParam("is_vulkan", is_vulkan_);
    nh_private_.getParam("update_airsim_control_every_n_sec", update_airsim_control_every_n_sec);
    nh_private_.getParam("publish_clock", publish_clock_);
    nh_private_.param("world_frame_id", world_frame_id_, world_frame_id_);
    odom_frame_id_ = world_frame_id_ == AIRSIM_FRAME_ID ? AIRSIM_ODOM_FRAME_ID : ENU_ODOM_FRAME_ID;
    nh_private_.param("odom_frame_id", odom_frame_id_, odom_frame_id_);
    isENU_ = !(odom_frame_id_ == AIRSIM_ODOM_FRAME_ID);
    nh_private_.param("coordinate_system_enu", isENU_, isENU_);
    vel_cmd_duration_ = 0.05; // todo rosparam
    nh_private_.param("map_resolution", resolution_, resolution_);
    // todo enforce dynamics constraints in this node as well?
    // nh_.getParam("max_vert_vel_", max_vert_vel_);
    // nh_.getParam("max_horz_vel", max_horz_vel_)

    double ex, ey, ez, eroll, epitch, eyaw;

    nh_private_.param("lidar/extrinsic_x", ex, 0.0);
    nh_private_.param("lidar/extrinsic_y", ey, 0.0);
    nh_private_.param("lidar/extrinsic_z", ez, 0.0);
    nh_private_.param("lidar/extrinsic_roll", eroll, 0.0);
    nh_private_.param("lidar/extrinsic_pitch", epitch, 0.0);
    nh_private_.param("lidar/extrinsic_yaw", eyaw, 0.0);
    lidar_d2body_t_vector_ << ex, ey, ez;
    lidar_d2body_R_matrix_ = Eigen::AngleAxisd(eyaw, Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd(epitch, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(eroll, Eigen::Vector3d::UnitX());

    double ex_cam, ey_cam, ez_cam, eroll_cam, epitch_cam, eyaw_cam;

    nh_private_.param("cam/extrinsic_x", ex_cam, 0.0);
    nh_private_.param("cam/extrinsic_y", ey_cam, 0.0);
    nh_private_.param("cam/extrinsic_z", ez_cam, 0.0);
    nh_private_.param("cam/extrinsic_roll", eroll_cam, 0.0);
    nh_private_.param("cam/extrinsic_pitch", epitch_cam, 0.0);
    nh_private_.param("cam/extrinsic_yaw", eyaw_cam, 0.0);
    cam_d2body_t_vector_ << ex_cam, ey_cam, ez_cam;
    cam_d2body_R_matrix_ = Eigen::AngleAxisd(eyaw_cam, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(epitch_cam, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(eroll_cam, Eigen::Vector3d::UnitX());
    nh_private_.param("cam/max_depth", cam_max_depth_, 30.0f);
    nh_private_.param("cam/skip_pixels", skip_pixels_, 10);
    nh_private_.param("traj/enable", enable_traj_viz_, enable_traj_viz_);
    nh_private_.param("traj/window", traj_window_sec_, traj_window_sec_);
    nh_private_.param("traj/buckets", traj_buckets_, traj_buckets_);
    nh_private_.param("traj/thickness", traj_thickness_, traj_thickness_);
    nh_private_.param("traj/duration", traj_duration_, traj_duration_);
    nh_private_.param("traj/publish_interval", traj_publish_interval_sec_, traj_publish_interval_sec_);
    int traj_max_points_param = static_cast<int>(traj_max_points_);
    nh_private_.param("traj/max_points", traj_max_points_param, traj_max_points_param);
    traj_max_points_ = std::max(2, traj_max_points_param);
    nh_private_.param("traj/alpha_pow", traj_alpha_pow_, traj_alpha_pow_);
    nh_private_.param("traj/min_alpha", traj_min_alpha_, traj_min_alpha_);
    nh_private_.param("traj/min_size", traj_min_size_, traj_min_size_);
    nh_private_.param("traj/min_spacing", traj_min_spacing_, traj_min_spacing_);
    nh_private_.param("traj/max_segment", traj_max_segment_, traj_max_segment_);

    int rows, cols;
    double fx, fy, cx, cy;
    nh_private_.param("cam/rows", rows, 480);
    nh_private_.param("cam/cols", cols, 640);
    nh_private_.param("cam/fx", fx, 0.0);
    nh_private_.param("cam/fy", fy, 0.0);
    nh_private_.param("cam/cx", cx, 0.0);
    nh_private_.param("cam/cy", cy, 0.0);

    scale_direction_table_.resize(rows, std::vector<Eigen::Vector3d>(cols));
    for (int v = 0; v < rows; ++v) 
    {
        for (int u = 0; u < cols; ++u) 
        {
            double x = (u - cx) / fx;
            double y = (v - cy) / fy;
            double z = 1.0;
            double norm = std::sqrt(x * x + y * y + z * z);
            scale_direction_table_[v][u] = Eigen::Vector3d(x / norm, y / norm, z / norm);
        }
    }

    gimbal_quat_pub_ = nh_private_.advertise<geometry_msgs::QuaternionStamped>("/gimbal_pose", 50);
    create_ros_pubs_from_settings_json();
    airsim_control_update_timer_ = nh_private_.createTimer(ros::Duration(update_airsim_control_every_n_sec), &AirsimROSWrapper::drone_state_timer_cb, this);
}

// XmlRpc::XmlRpcValue can't be const in this case
void AirsimROSWrapper::create_ros_pubs_from_settings_json()
{
    // subscribe to control commands on global nodehandle
    gimbal_angle_quat_cmd_sub_ = nh_private_.subscribe("gimbal_angle_quat_cmd", 50, &AirsimROSWrapper::gimbal_angle_quat_cmd_cb, this);
    gimbal_angle_euler_cmd_sub_ = nh_private_.subscribe("/gimbal_angle_euler_cmd", 50, &AirsimROSWrapper::gimbal_angle_euler_cmd_cb, this);
    rpy_thrust_sub_ = nh_private_.subscribe("/setpoint_raw/attitude",50,&AirsimROSWrapper::rpy_thrust_cmd_cb,this,ros::TransportHints().tcpNoDelay());

    origin_geo_point_pub_ = nh_private_.advertise<airsim_ros_pkgs::GPSYaw>("origin_geo_point", 10);       

    airsim_img_request_vehicle_name_pair_vec_.clear();
    image_pub_vec_.clear();
    cam_info_pub_vec_.clear();
    camera_info_msg_vec_.clear();
    vehicle_name_ptr_map_.clear();
    size_t lidar_cnt = 0;

    image_transport::ImageTransport image_transporter(nh_private_);

    // iterate over std::map<std::string, std::unique_ptr<VehicleSetting>> vehicles;
    for (const auto& curr_vehicle_elem : AirSimSettings::singleton().vehicles)
    {
        auto& vehicle_setting = curr_vehicle_elem.second;
        auto curr_vehicle_name = curr_vehicle_elem.first;
        set_nans_to_zeros_in_pose(*vehicle_setting);
       
        std::unique_ptr<VehicleROS> vehicle_ros = nullptr;
        
        if (airsim_mode_ == AIRSIM_MODE::DRONE)
        {
            vehicle_ros = std::unique_ptr<MultiRotorROS>(new MultiRotorROS());
        }
        else
        {
            vehicle_ros = std::unique_ptr<CarROS>(new CarROS());
        }

        world_img_rgb_pcd_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>(curr_vehicle_name + "/world_img_rgb_pcd", 1);

        vehicle_ros->odom_frame_id = curr_vehicle_name + "/" + odom_frame_id_;
        vehicle_ros->vehicle_name = curr_vehicle_name;

        append_static_vehicle_tf(vehicle_ros.get(), *vehicle_setting);
        
        vehicle_ros->odom_local_pub = nh_private_.advertise<nav_msgs::Odometry>(curr_vehicle_name + "/" + odom_frame_id_, 10);
        vehicle_ros->control_odom_pub = nh_private_.advertise<nav_msgs::Odometry>("control_odom", 10);

        vehicle_ros->env_pub = nh_private_.advertise<airsim_ros_pkgs::Environment>(curr_vehicle_name + "/environment", 10);

        vehicle_ros->global_gps_pub = nh_private_.advertise<sensor_msgs::NavSatFix>(curr_vehicle_name + "/global_gps", 10);

        vehicle_ros->lio_odom_pub = nh_private_.advertise<nav_msgs::Odometry>("/lio_odom", 10);
        vehicle_ros->world_lidar_pub = nh_private_.advertise<sensor_msgs::PointCloud2>(curr_vehicle_name + "/world_lidar_cloud", 1);

        airsim_map_pub = nh_private_.advertise<sensor_msgs::PointCloud2>("/airsim_global_map", 1);

        if (airsim_mode_ == AIRSIM_MODE::DRONE)
        {
            auto drone = static_cast<MultiRotorROS*>(vehicle_ros.get());
            
            // bind to a single callback. todo optimal subs queue length
            // bind multiple topics to a single callback, but keep track of which vehicle name it was by passing curr_vehicle_name as the 2nd argument 
            drone->vel_cmd_body_frame_sub = nh_private_.subscribe<airsim_ros_pkgs::VelCmd>(curr_vehicle_name + "/vel_cmd_body_frame", 1, 
                boost::bind(&AirsimROSWrapper::vel_cmd_body_frame_cb, this, _1, vehicle_ros->vehicle_name)); // todo ros::TransportHints().tcpNoDelay();
            drone->vel_cmd_world_frame_sub = nh_private_.subscribe<airsim_ros_pkgs::VelCmd>(curr_vehicle_name + "/vel_cmd_world_frame", 1, 
                boost::bind(&AirsimROSWrapper::vel_cmd_world_frame_cb, this, _1, vehicle_ros->vehicle_name));

            drone->takeoff_srvr = nh_private_.advertiseService<airsim_ros_pkgs::Takeoff::Request, airsim_ros_pkgs::Takeoff::Response>(curr_vehicle_name + "/takeoff", 
                boost::bind(&AirsimROSWrapper::takeoff_srv_cb, this, _1, _2, vehicle_ros->vehicle_name) );
            drone->land_srvr = nh_private_.advertiseService<airsim_ros_pkgs::Land::Request, airsim_ros_pkgs::Land::Response>(curr_vehicle_name + "/land", 
                boost::bind(&AirsimROSWrapper::land_srv_cb, this, _1, _2, vehicle_ros->vehicle_name) );
            // vehicle_ros.reset_srvr = nh_private_.advertiseService(curr_vehicle_name + "/reset",&AirsimROSWrapper::reset_srv_cb, this);
        }
        else
        {
            auto car = static_cast<CarROS*>(vehicle_ros.get());
            car->car_cmd_sub = nh_private_.subscribe<airsim_ros_pkgs::CarControls>(curr_vehicle_name + "/car_cmd", 1,
                boost::bind(&AirsimROSWrapper::car_cmd_cb, this, _1, vehicle_ros->vehicle_name));
            car->car_state_pub = nh_private_.advertise<airsim_ros_pkgs::CarState>(curr_vehicle_name + "/car_state", 10);
        }

        // iterate over camera map std::map<std::string, CameraSetting> .cameras;
        for (auto& curr_camera_elem : vehicle_setting->cameras)
        {
            auto& camera_setting = curr_camera_elem.second;
            cam_offset_x_ = camera_setting.position.x();
            cam_offset_y_ = camera_setting.position.y();
            cam_offset_z_ = camera_setting.position.z();
            std::cout << "Camera name: " << curr_camera_elem.first << std::endl;
            std::cout << "Camera position: " << cam_offset_x_ << " " << cam_offset_y_ << " " << cam_offset_z_ << std::endl;

            auto& curr_camera_name = curr_camera_elem.first;
            // vehicle_setting_vec_.push_back(*vehicle_setting.get());
            set_nans_to_zeros_in_pose(*vehicle_setting, camera_setting);
            append_static_camera_tf(vehicle_ros.get(), curr_camera_name, camera_setting);
            // camera_setting.gimbal
            std::vector<ImageRequest> current_image_request_vec;
            current_image_request_vec.clear();

            // iterate over capture_setting std::map<int, CaptureSetting> capture_settings
            for (const auto& curr_capture_elem : camera_setting.capture_settings)
            {
                auto& capture_setting = curr_capture_elem.second;

                // todo why does AirSimSettings::loadCaptureSettings calls AirSimSettings::initializeCaptureSettings()
                // which initializes default capture settings for _all_ NINE msr::airlib::ImageCaptureBase::ImageType
                if ( !(std::isnan(capture_setting.fov_degrees)) )
                {
                    ImageType curr_image_type = msr::airlib::Utils::toEnum<ImageType>(capture_setting.image_type);
                    // if scene / segmentation / surface normals / infrared, get uncompressed image with pixels_as_floats = false
                    if (capture_setting.image_type == 0 || capture_setting.image_type == 5 || capture_setting.image_type == 6 || capture_setting.image_type == 7)
                    {
                        current_image_request_vec.push_back(ImageRequest(curr_camera_name, curr_image_type, false, false));
                    }
                    // if {DepthPlanner, DepthPerspective,DepthVis, DisparityNormalized}, get float image
                    else
                    {
                        current_image_request_vec.push_back(ImageRequest(curr_camera_name, curr_image_type, true));
                    }

                    image_pub_vec_.push_back(image_transporter.advertise(curr_vehicle_name + "/" + curr_camera_name + "/" + image_type_int_to_string_map_.at(capture_setting.image_type), 1));
                    cam_info_pub_vec_.push_back(nh_private_.advertise<sensor_msgs::CameraInfo> (curr_vehicle_name + "/" + curr_camera_name + "/" + image_type_int_to_string_map_.at(capture_setting.image_type) + "/camera_info", 10));
                    camera_info_msg_vec_.push_back(generate_cam_info(curr_camera_name, camera_setting, capture_setting));
                }
            }
            // push back pair (vector of image captures, current vehicle name) 
            airsim_img_request_vehicle_name_pair_vec_.push_back(std::make_pair(current_image_request_vec, curr_vehicle_name));

        }

        // iterate over sensors
        std::vector<SensorPublisher> sensors;
        for (auto& curr_sensor_map : vehicle_setting->sensors)
        {
            auto& sensor_name = curr_sensor_map.first;
            auto& sensor_setting = curr_sensor_map.second;

            if (sensor_setting->enabled)
            {             
                SensorPublisher sensor_publisher;
                sensor_publisher.sensor_name = sensor_setting->sensor_name;
                sensor_publisher.sensor_type = sensor_setting->sensor_type;
                switch (sensor_setting->sensor_type)
                {
                    case SensorBase::SensorType::Barometer:
                    {
                        std::cout << "Barometer" << std::endl; 
                        sensor_publisher.publisher = nh_private_.advertise<airsim_ros_pkgs::Altimeter>(curr_vehicle_name + "/altimeter/" + sensor_name, 10);
                        break;
                    }
                    case SensorBase::SensorType::Imu:
                    {
                        std::cout << "Imu" << std::endl;
                        sensor_publisher.publisher = nh_private_.advertise<sensor_msgs::Imu>(curr_vehicle_name + "/imu/" + sensor_name, 10);
                        break;
                    }
                    case SensorBase::SensorType::Gps:
                    {
                        std::cout << "Gps" << std::endl; 
                        sensor_publisher.publisher = nh_private_.advertise<sensor_msgs::NavSatFix>(curr_vehicle_name + "/gps/" + sensor_name, 10);
                        break;
                    }
                    case SensorBase::SensorType::Magnetometer:
                    {
                        std::cout << "Magnetometer" << std::endl; 
                        sensor_publisher.publisher = nh_private_.advertise<sensor_msgs::MagneticField>(curr_vehicle_name + "/magnetometer/" + sensor_name, 10);
                        break;
                    }
                    case SensorBase::SensorType::Distance:
                    {
                        std::cout << "Distance" << std::endl; 
                        sensor_publisher.publisher = nh_private_.advertise<sensor_msgs::Range>(curr_vehicle_name + "/distance/" + sensor_name, 10);
                        break;
                    }
                    case SensorBase::SensorType::Lidar:
                    {
                        std::cout << "Lidar" << std::endl;
                        auto lidar_setting = *static_cast<LidarSetting*>(sensor_setting.get());
                        set_nans_to_zeros_in_pose(*vehicle_setting, lidar_setting);
                        append_static_lidar_tf(vehicle_ros.get(), sensor_name, lidar_setting);
                        sensor_publisher.publisher = nh_private_.advertise<sensor_msgs::PointCloud2>(curr_vehicle_name + "/lidar/" + sensor_name, 10);
                        break;
                    }
                    default:
                    {
                        throw std::invalid_argument("Unexpected sensor type");
                    }
                }
                sensors.emplace_back(sensor_publisher);
            }
        }

        // we want fast access to the lidar sensors for callback handling, sort them out now
        auto isLidar = std::function<bool(const SensorPublisher& pub)>([](const SensorPublisher& pub)
        {
            return pub.sensor_type == SensorBase::SensorType::Lidar;
        });
        size_t cnt = std::count_if( sensors.begin(), sensors.end(), isLidar);
        lidar_cnt+=cnt;
        vehicle_ros->lidar_pubs.resize(cnt);
        vehicle_ros->sensor_pubs.resize(sensors.size() - cnt);
        std::partition_copy (sensors.begin(), sensors.end(), vehicle_ros->lidar_pubs.begin(), vehicle_ros->sensor_pubs.begin(), isLidar);

        vehicle_name_ptr_map_.emplace(curr_vehicle_name, std::move(vehicle_ros)); // allows fast lookup in command callbacks in case of a lot of drones
    }

    // add takeoff and land all services if more than 2 drones
    if (vehicle_name_ptr_map_.size() > 1 && airsim_mode_ == AIRSIM_MODE::DRONE)
    {
        takeoff_all_srvr_ = nh_private_.advertiseService("all_robots/takeoff", &AirsimROSWrapper::takeoff_all_srv_cb, this);
        land_all_srvr_ = nh_private_.advertiseService("all_robots/land", &AirsimROSWrapper::land_all_srv_cb, this);

        // gimbal_angle_quat_cmd_sub_ = nh_.subscribe("gimbal_angle_quat_cmd", 50, &AirsimROSWrapper::gimbal_angle_quat_cmd_cb, this);

        vel_cmd_all_body_frame_sub_ = nh_private_.subscribe("all_robots/vel_cmd_body_frame", 1, &AirsimROSWrapper::vel_cmd_all_body_frame_cb, this);
        vel_cmd_all_world_frame_sub_ = nh_private_.subscribe("all_robots/vel_cmd_world_frame", 1, &AirsimROSWrapper::vel_cmd_all_world_frame_cb, this);

        vel_cmd_group_body_frame_sub_ = nh_private_.subscribe("group_of_robots/vel_cmd_body_frame", 1, &AirsimROSWrapper::vel_cmd_group_body_frame_cb, this);
        vel_cmd_group_world_frame_sub_ = nh_private_.subscribe("group_of_obots/vel_cmd_world_frame", 1, &AirsimROSWrapper::vel_cmd_group_world_frame_cb, this);

        takeoff_group_srvr_ = nh_private_.advertiseService("group_of_robots/takeoff", &AirsimROSWrapper::takeoff_group_srv_cb, this);
        land_group_srvr_ = nh_private_.advertiseService("group_of_robots/land", &AirsimROSWrapper::land_group_srv_cb, this);
    }

    // todo add per vehicle reset in AirLib API
    reset_srvr_ = nh_private_.advertiseService("reset",&AirsimROSWrapper::reset_srv_cb, this);

    if (publish_clock_)
    {
        clock_pub_ = nh_private_.advertise<rosgraph_msgs::Clock>("clock", 1);
    }

    // if >0 cameras, add one more thread for img_request_timer_cb
    if(!airsim_img_request_vehicle_name_pair_vec_.empty())
    {
        double update_airsim_img_response_every_n_sec;
        nh_private_.getParam("update_airsim_img_response_every_n_sec", update_airsim_img_response_every_n_sec);

        ros::TimerOptions timer_options(ros::Duration(update_airsim_img_response_every_n_sec), boost::bind(&AirsimROSWrapper::img_response_timer_cb, this, _1), &img_timer_cb_queue_);
        airsim_img_response_timer_ = nh_private_.createTimer(timer_options);
        is_used_img_timer_cb_queue_ = true;   
    }

    // lidars update on their own callback/thread at a given rate
    if (lidar_cnt > 0)
    {    
        double update_lidar_every_n_sec;
        nh_private_.getParam("update_lidar_every_n_sec", update_lidar_every_n_sec);
        // nh_private_.setCallbackQueue(&lidar_timer_cb_queue_);
        ros::TimerOptions timer_options(ros::Duration(update_lidar_every_n_sec), boost::bind(&AirsimROSWrapper::lidar_timer_cb, this, _1), &lidar_timer_cb_queue_);
        airsim_lidar_update_timer_ = nh_private_.createTimer(timer_options);            
        is_used_lidar_timer_cb_queue_ = true;
    }

    const double traj_timer_period = std::max(0.01, traj_publish_interval_sec_);
    airsim_traj_viz_timer_ = nh_private_.createTimer(ros::Duration(traj_timer_period), &AirsimROSWrapper::traj_viz_timer_cb, this);
    is_used_traj_timer_cb_queue_ = true;
    ROS_INFO_STREAM("Trajectory timer started, interval=" << traj_timer_period
                    << "s, launch traj/enable=" << (enable_traj_viz_ ? "true" : "false"));

    initialize_airsim();
}

// todo: error check. if state is not landed, return error. 
bool AirsimROSWrapper::takeoff_srv_cb(airsim_ros_pkgs::Takeoff::Request& request, airsim_ros_pkgs::Takeoff::Response& response, const std::string& vehicle_name)
{
    std::lock_guard<std::mutex> guard(drone_control_mutex_);

    if (request.waitOnLastTask)
        static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->takeoffAsync(20, vehicle_name)->waitOnLastTask(); // todo value for timeout_sec? 
        // response.success = 
    else
        static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->takeoffAsync(20, vehicle_name);
        // response.success = 

    return true;
}

bool AirsimROSWrapper::takeoff_group_srv_cb(airsim_ros_pkgs::TakeoffGroup::Request& request, airsim_ros_pkgs::TakeoffGroup::Response& response)
{
    std::lock_guard<std::mutex> guard(drone_control_mutex_);

    if (request.waitOnLastTask)
        for(const auto& vehicle_name : request.vehicle_names)
            static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->takeoffAsync(20, vehicle_name)->waitOnLastTask(); // todo value for timeout_sec? 
        // response.success = 
    else
        for(const auto& vehicle_name : request.vehicle_names)
            static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->takeoffAsync(20, vehicle_name);
        // response.success = 
    
    return true;
}

bool AirsimROSWrapper::takeoff_all_srv_cb(airsim_ros_pkgs::Takeoff::Request& request, airsim_ros_pkgs::Takeoff::Response& response)
{
    std::lock_guard<std::mutex> guard(drone_control_mutex_);

    if (request.waitOnLastTask)
        for(const auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_)
            static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->takeoffAsync(20, vehicle_name_ptr_pair.first)->waitOnLastTask(); // todo value for timeout_sec? 
        // response.success = 
    else
        for(const auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_)
            static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->takeoffAsync(20, vehicle_name_ptr_pair.first);
        // response.success = 

    return true;
}

bool AirsimROSWrapper::land_srv_cb(airsim_ros_pkgs::Land::Request& request, airsim_ros_pkgs::Land::Response& response, const std::string& vehicle_name)
{
    std::lock_guard<std::mutex> guard(drone_control_mutex_);

    if (request.waitOnLastTask)
        static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->landAsync(60, vehicle_name)->waitOnLastTask();
    else
        static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->landAsync(60, vehicle_name);

    return true; //todo
}

bool AirsimROSWrapper::land_group_srv_cb(airsim_ros_pkgs::LandGroup::Request& request, airsim_ros_pkgs::LandGroup::Response& response)
{
    std::lock_guard<std::mutex> guard(drone_control_mutex_);

    if (request.waitOnLastTask)
        for(const auto& vehicle_name : request.vehicle_names)
            static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->landAsync(60, vehicle_name)->waitOnLastTask();
    else
        for(const auto& vehicle_name : request.vehicle_names)
            static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->landAsync(60, vehicle_name);

    return true; //todo
}

bool AirsimROSWrapper::land_all_srv_cb(airsim_ros_pkgs::Land::Request& request, airsim_ros_pkgs::Land::Response& response)
{
    std::lock_guard<std::mutex> guard(drone_control_mutex_);

    if (request.waitOnLastTask)
        for(const auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_)
            static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->landAsync(60, vehicle_name_ptr_pair.first)->waitOnLastTask();
    else
        for(const auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_)
            static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->landAsync(60, vehicle_name_ptr_pair.first);

    return true; //todo
}

// todo add reset by vehicle_name API to airlib
// todo not async remove waitonlasttask
bool AirsimROSWrapper::reset_srv_cb(airsim_ros_pkgs::Reset::Request& request, airsim_ros_pkgs::Reset::Response& response)
{
    std::lock_guard<std::mutex> guard(drone_control_mutex_);

    airsim_client_.reset();
    return true; //todo
}

tf2::Quaternion AirsimROSWrapper::get_tf2_quat(const msr::airlib::Quaternionr& airlib_quat) const
{
    return tf2::Quaternion(airlib_quat.x(), airlib_quat.y(), airlib_quat.z(), airlib_quat.w());
}

msr::airlib::Quaternionr AirsimROSWrapper::get_airlib_quat(const geometry_msgs::Quaternion& geometry_msgs_quat) const
{
    return msr::airlib::Quaternionr(geometry_msgs_quat.w, geometry_msgs_quat.x, geometry_msgs_quat.y, geometry_msgs_quat.z); 
}

msr::airlib::Quaternionr AirsimROSWrapper::get_airlib_quat(const tf2::Quaternion& tf2_quat) const
{
    return msr::airlib::Quaternionr(tf2_quat.w(), tf2_quat.x(), tf2_quat.y(), tf2_quat.z()); 
}

void AirsimROSWrapper::car_cmd_cb(const airsim_ros_pkgs::CarControls::ConstPtr& msg, const std::string& vehicle_name)
{
    std::lock_guard<std::mutex> guard(drone_control_mutex_);

    auto car = static_cast<CarROS*>(vehicle_name_ptr_map_[vehicle_name].get());
    car->car_cmd.throttle = msg->throttle;
    car->car_cmd.steering = msg->steering;
    car->car_cmd.brake = msg->brake;
    car->car_cmd.handbrake = msg->handbrake;
    car->car_cmd.is_manual_gear = msg->manual;
    car->car_cmd.manual_gear = msg->manual_gear;
    car->car_cmd.gear_immediate = msg->gear_immediate;

    car->has_car_cmd = true;
}

msr::airlib::Pose AirsimROSWrapper::get_airlib_pose(const float& x, const float& y, const float& z, const msr::airlib::Quaternionr& airlib_quat) const
{
    return msr::airlib::Pose(msr::airlib::Vector3r(x, y, z), airlib_quat);
}

// void AirsimROSWrapper::vel_cmd_body_frame_cb(const airsim_ros_pkgs::VelCmd& msg, const std::string& vehicle_name)
void AirsimROSWrapper::vel_cmd_body_frame_cb(const airsim_ros_pkgs::VelCmd::ConstPtr& msg, const std::string& vehicle_name)
{
    std::lock_guard<std::mutex> guard(drone_control_mutex_);

    auto drone = static_cast<MultiRotorROS*>(vehicle_name_ptr_map_[vehicle_name].get());
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(get_tf2_quat(drone->curr_drone_state.kinematics_estimated.pose.orientation)).getRPY(roll, pitch, yaw); // ros uses xyzw

    // todo do actual body frame?
    drone->vel_cmd.x = (msg->twist.linear.x * cos(yaw)) - (msg->twist.linear.y * sin(yaw)); //body frame assuming zero pitch roll
    drone->vel_cmd.y = (msg->twist.linear.x * sin(yaw)) + (msg->twist.linear.y * cos(yaw)); //body frame
    drone->vel_cmd.z = msg->twist.linear.z;
    drone->vel_cmd.drivetrain = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
    drone->vel_cmd.yaw_mode.is_rate = true;
    // airsim uses degrees
    drone->vel_cmd.yaw_mode.yaw_or_rate = math_common::rad2deg(msg->twist.angular.z);
    drone->has_vel_cmd = true;
}

void AirsimROSWrapper::vel_cmd_group_body_frame_cb(const airsim_ros_pkgs::VelCmdGroup& msg)
{
    std::lock_guard<std::mutex> guard(drone_control_mutex_);

    for(const auto& vehicle_name : msg.vehicle_names)
    {
        auto drone = static_cast<MultiRotorROS*>(vehicle_name_ptr_map_[vehicle_name].get());

        double roll, pitch, yaw;
        tf2::Matrix3x3(get_tf2_quat(drone->curr_drone_state.kinematics_estimated.pose.orientation)).getRPY(roll, pitch, yaw); // ros uses xyzw
        
        // todo do actual body frame?
        drone->vel_cmd.x = (msg.twist.linear.x * cos(yaw)) - (msg.twist.linear.y * sin(yaw)); //body frame assuming zero pitch roll
        drone->vel_cmd.y = (msg.twist.linear.x * sin(yaw)) + (msg.twist.linear.y * cos(yaw)); //body frame
        drone->vel_cmd.z = msg.twist.linear.z;
        drone->vel_cmd.drivetrain = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
        drone->vel_cmd.yaw_mode.is_rate = true;
        // airsim uses degrees
        drone->vel_cmd.yaw_mode.yaw_or_rate = math_common::rad2deg(msg.twist.angular.z);
        drone->has_vel_cmd = true;
    }
}

// void AirsimROSWrapper::vel_cmd_all_body_frame_cb(const airsim_ros_pkgs::VelCmd::ConstPtr& msg)
void AirsimROSWrapper::vel_cmd_all_body_frame_cb(const airsim_ros_pkgs::VelCmd& msg)
{
    std::lock_guard<std::mutex> guard(drone_control_mutex_);

    // todo expose waitOnLastTask or nah?
    for(auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_)
    {
        auto drone = static_cast<MultiRotorROS*>(vehicle_name_ptr_pair.second.get());

        double roll, pitch, yaw;
        tf2::Matrix3x3(get_tf2_quat(drone->curr_drone_state.kinematics_estimated.pose.orientation)).getRPY(roll, pitch, yaw); // ros uses xyzw

        // todo do actual body frame?
        drone->vel_cmd.x = (msg.twist.linear.x * cos(yaw)) - (msg.twist.linear.y * sin(yaw)); //body frame assuming zero pitch roll
        drone->vel_cmd.y = (msg.twist.linear.x * sin(yaw)) + (msg.twist.linear.y * cos(yaw)); //body frame
        drone->vel_cmd.z = msg.twist.linear.z;
        drone->vel_cmd.drivetrain = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
        drone->vel_cmd.yaw_mode.is_rate = true;
        // airsim uses degrees
        drone->vel_cmd.yaw_mode.yaw_or_rate = math_common::rad2deg(msg.twist.angular.z);
        drone->has_vel_cmd = true;
    }
}

void AirsimROSWrapper::vel_cmd_world_frame_cb(const airsim_ros_pkgs::VelCmd::ConstPtr& msg, const std::string& vehicle_name)
{
    std::lock_guard<std::mutex> guard(drone_control_mutex_);

    auto drone = static_cast<MultiRotorROS*>(vehicle_name_ptr_map_[vehicle_name].get());

    drone->vel_cmd.x = msg->twist.linear.x;
    drone->vel_cmd.y = msg->twist.linear.y;
    drone->vel_cmd.z = msg->twist.linear.z;
    drone->vel_cmd.drivetrain = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
    drone->vel_cmd.yaw_mode.is_rate = true;
    drone->vel_cmd.yaw_mode.yaw_or_rate = math_common::rad2deg(msg->twist.angular.z);
    drone->has_vel_cmd = true;
}

// this is kinda unnecessary but maybe it makes life easier for the end user. 
void AirsimROSWrapper::vel_cmd_group_world_frame_cb(const airsim_ros_pkgs::VelCmdGroup& msg)
{
    std::lock_guard<std::mutex> guard(drone_control_mutex_);

    for(const auto& vehicle_name : msg.vehicle_names)
    {
        auto drone = static_cast<MultiRotorROS*>(vehicle_name_ptr_map_[vehicle_name].get());

        drone->vel_cmd.x = msg.twist.linear.x;
        drone->vel_cmd.y = msg.twist.linear.y;
        drone->vel_cmd.z = msg.twist.linear.z;
        drone->vel_cmd.drivetrain = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
        drone->vel_cmd.yaw_mode.is_rate = true;
        drone->vel_cmd.yaw_mode.yaw_or_rate = math_common::rad2deg(msg.twist.angular.z);
        drone->has_vel_cmd = true;
    }
}

void AirsimROSWrapper::vel_cmd_all_world_frame_cb(const airsim_ros_pkgs::VelCmd& msg)
{
    std::lock_guard<std::mutex> guard(drone_control_mutex_);

    // todo expose waitOnLastTask or nah?
    for(auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_)
    {
        auto drone = static_cast<MultiRotorROS*>(vehicle_name_ptr_pair.second.get());

        drone->vel_cmd.x = msg.twist.linear.x;
        drone->vel_cmd.y = msg.twist.linear.y;
        drone->vel_cmd.z = msg.twist.linear.z;
        drone->vel_cmd.drivetrain = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
        drone->vel_cmd.yaw_mode.is_rate = true;
        drone->vel_cmd.yaw_mode.yaw_or_rate = math_common::rad2deg(msg.twist.angular.z);
        drone->has_vel_cmd = true;
    }
}

// todo support multiple gimbal commands
void AirsimROSWrapper::gimbal_angle_quat_cmd_cb(const airsim_ros_pkgs::GimbalAngleQuatCmd& gimbal_angle_quat_cmd_msg)
{
    tf2::Quaternion quat_control_cmd;
    try
    {
        tf2::convert(gimbal_angle_quat_cmd_msg.orientation, quat_control_cmd);
        quat_control_cmd.normalize();
        gimbal_cmd_.target_quat = get_airlib_quat(quat_control_cmd); // airsim uses wxyz
        gimbal_cmd_.camera_name = gimbal_angle_quat_cmd_msg.camera_name;
        gimbal_cmd_.vehicle_name = gimbal_angle_quat_cmd_msg.vehicle_name;
        has_gimbal_cmd_ = true; 
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("%s",ex.what());
    }
}

// todo support multiple gimbal commands
// 1. find quaternion of default gimbal pose
// 2. forward multiply with quaternion equivalent to desired euler commands (in degrees)
// 3. call airsim client's setCameraPose which sets camera pose wrt world (or takeoff?) ned frame. todo 
void AirsimROSWrapper::gimbal_angle_euler_cmd_cb(const airsim_ros_pkgs::GimbalAngleEulerCmd& gimbal_angle_euler_cmd_msg)
{
    try
    {
        tf2::Quaternion quat_control_cmd;
        quat_control_cmd.setRPY(math_common::deg2rad(gimbal_angle_euler_cmd_msg.roll), math_common::deg2rad(gimbal_angle_euler_cmd_msg.pitch), math_common::deg2rad(gimbal_angle_euler_cmd_msg.yaw));
        quat_control_cmd.normalize();
        gimbal_cmd_.target_quat = get_airlib_quat(quat_control_cmd);
        gimbal_cmd_.camera_name = gimbal_angle_euler_cmd_msg.camera_name;
        gimbal_cmd_.vehicle_name = gimbal_angle_euler_cmd_msg.vehicle_name;
        has_gimbal_cmd_ = true; 
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("%s",ex.what());
    }
}

/*----------------------Position command callback---------------*/
void AirsimROSWrapper::rpy_thrust_cmd_cb(const mavros_msgs::AttitudeTarget& attitude_cmd_msg){

    if(isENU_){
        double thrust = attitude_cmd_msg.thrust;

        if (attitude_cmd_msg.type_mask == 1)
        {
        // Angle control
        Eigen::Quaterniond q;
        q.w() = attitude_cmd_msg.orientation.w;
        q.x() = attitude_cmd_msg.orientation.x;
        q.y() = attitude_cmd_msg.orientation.y;
        q.z() = attitude_cmd_msg.orientation.z;

        Eigen::Vector3d ypr;
        ypr(2) = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y()));
        ypr(1) = asin(2 * (q.w() * q.y() - q.z() * q.x()));
        ypr(0) = atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
        // AirSim Trick
        ypr(0) = ypr(0) + M_PI/2.0;

        // static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->moveByRollPitchYawThrottleAsync(0.0, 0
        // , 0.0, thrust, 0.01, "drone_1");
        
        /*pitch大于0，机头上仰；yaw大于0，机头右偏；row大于0，机身左高右低；因此在用角度控制的时候，要把pitch和yaw取个负值*/

        static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->moveByRollPitchYawThrottleAsync(ypr(2), ypr(1)
        , ypr(0), thrust, 0.01, "drone_1");
        // static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->moveByRollPitchYawThrottleAsync(0, 0
        // , 0, thrust, 0.01, "drone_1");
        }

        else
        {
        // ! /* Planner */ --> Angular velocity control
        // static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->moveByAngleRatesThrottleAsync(attitude_cmd_msg.body_rate.x,
        // -attitude_cmd_msg.body_rate.y,-attitude_cmd_msg.body_rate.z,thrust,0.01);
        
        // ! /* Planner */ --> Roll Pitch YawRate control
        static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->moveByRollPitchYawrateThrottleAsync(attitude_cmd_msg.body_rate.x,
        -attitude_cmd_msg.body_rate.y,-attitude_cmd_msg.body_rate.z,thrust,0.01);

        // static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->simPrintLogMessage()
        }

    }
    else{
        // static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->moveByRollPitchYawThrottleAsync(roll,pitch,yaw,thrust,0.005,"drone_1");
        static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->moveByAngleRatesThrottleAsync(attitude_cmd_msg.body_rate.x,
        attitude_cmd_msg.body_rate.y,attitude_cmd_msg.body_rate.z,attitude_cmd_msg.thrust,0.01);
    }
    return;//hzchzchzc
}

airsim_ros_pkgs::CarState AirsimROSWrapper::get_roscarstate_msg_from_car_state(const msr::airlib::CarApiBase::CarState& car_state) const
{
    airsim_ros_pkgs::CarState state_msg;
    const auto odo = get_odom_msg_from_car_state(car_state);

    state_msg.pose = odo.pose;
    state_msg.twist = odo.twist;
    state_msg.speed = car_state.speed;
    state_msg.gear = car_state.gear;
    state_msg.rpm = car_state.rpm;
    state_msg.maxrpm = car_state.maxrpm;
    state_msg.handbrake = car_state.handbrake;
    state_msg.header.stamp = airsim_timestamp_to_ros(car_state.timestamp);

    return state_msg;
}

nav_msgs::Odometry AirsimROSWrapper::get_odom_msg_from_car_state(const msr::airlib::CarApiBase::CarState& car_state) const
{
    nav_msgs::Odometry odom_msg;

    odom_msg.pose.pose.position.x = car_state.getPosition().x();
    odom_msg.pose.pose.position.y = car_state.getPosition().y();
    odom_msg.pose.pose.position.z = car_state.getPosition().z();
    odom_msg.pose.pose.orientation.x = car_state.getOrientation().x();
    odom_msg.pose.pose.orientation.y = car_state.getOrientation().y();
    odom_msg.pose.pose.orientation.z = car_state.getOrientation().z();
    odom_msg.pose.pose.orientation.w = car_state.getOrientation().w();

    odom_msg.twist.twist.linear.x = car_state.kinematics_estimated.twist.linear.x();
    odom_msg.twist.twist.linear.y = car_state.kinematics_estimated.twist.linear.y();
    odom_msg.twist.twist.linear.z = car_state.kinematics_estimated.twist.linear.z();
    odom_msg.twist.twist.angular.x = car_state.kinematics_estimated.twist.angular.x();
    odom_msg.twist.twist.angular.y = car_state.kinematics_estimated.twist.angular.y();
    odom_msg.twist.twist.angular.z = car_state.kinematics_estimated.twist.angular.z();

    if (isENU_)
    {
        std::swap(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y);
        odom_msg.pose.pose.position.z = -odom_msg.pose.pose.position.z;
        std::swap(odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y);        
        odom_msg.pose.pose.orientation.z = -odom_msg.pose.pose.orientation.z;
        std::swap(odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y);
        odom_msg.twist.twist.linear.z = -odom_msg.twist.twist.linear.z;
        std::swap(odom_msg.twist.twist.angular.x, odom_msg.twist.twist.angular.y);
        odom_msg.twist.twist.angular.z = -odom_msg.twist.twist.angular.z;
    }

    return odom_msg;
}

nav_msgs::Odometry AirsimROSWrapper::get_odom_msg_from_truth_state(const msr::airlib::MultirotorState& truth_state) const
{
    nav_msgs::Odometry odom_msg;
    // odom_msg.pose.pose.position.x = truth_state.pose.position.x();
    // odom_msg.pose.pose.position.y = truth_state.pose.position.y();
    // odom_msg.pose.pose.position.z = truth_state.pose.position.z() - origin_z;
    // odom_msg.pose.pose.orientation.x = truth_state.pose.orientation.x();
    // odom_msg.pose.pose.orientation.y = truth_state.pose.orientation.y();
    // odom_msg.pose.pose.orientation.z = truth_state.pose.orientation.z();
    // odom_msg.pose.pose.orientation.w = truth_state.pose.orientation.w();
    // odom_msg.twist.twist.linear.x = truth_state.twist.linear.x();
    // odom_msg.twist.twist.linear.y = truth_state.twist.linear.y();
    // odom_msg.twist.twist.linear.z = truth_state.twist.linear.z();
    // odom_msg.twist.twist.angular.x = truth_state.twist.angular.x();
    // odom_msg.twist.twist.angular.y = truth_state.twist.angular.y();
    // odom_msg.twist.twist.angular.z = truth_state.twist.angular.z();

    odom_msg.pose.pose.position.x = truth_state.getPosition().x();
    odom_msg.pose.pose.position.y = truth_state.getPosition().y();
    odom_msg.pose.pose.position.z = truth_state.getPosition().z() - origin_z;
    odom_msg.pose.pose.orientation.x = truth_state.getOrientation().x();
    odom_msg.pose.pose.orientation.y = truth_state.getOrientation().y();
    odom_msg.pose.pose.orientation.z = truth_state.getOrientation().z();
    odom_msg.pose.pose.orientation.w = truth_state.getOrientation().w();

    odom_msg.twist.twist.linear.x = truth_state.kinematics_estimated.twist.linear.x();
    odom_msg.twist.twist.linear.y = truth_state.kinematics_estimated.twist.linear.y();
    odom_msg.twist.twist.linear.z = truth_state.kinematics_estimated.twist.linear.z();
    odom_msg.twist.twist.angular.x = truth_state.kinematics_estimated.twist.angular.x();
    odom_msg.twist.twist.angular.y = truth_state.kinematics_estimated.twist.angular.y();
    odom_msg.twist.twist.angular.z = truth_state.kinematics_estimated.twist.angular.z();

    if (isENU_)
    {
        std::swap(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y);
        odom_msg.pose.pose.position.z = -odom_msg.pose.pose.position.z;
        // std::swap(odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y);        
        // odom_msg.pose.pose.orientation.z = -odom_msg.pose.pose.orientation.z;
        //hzchzc
        double roll,pitch,yaw;
        Eigen::Quaterniond attitude_quater;
        attitude_quater.w() = odom_msg.pose.pose.orientation.w;
        attitude_quater.x() = odom_msg.pose.pose.orientation.x;
        attitude_quater.y() = odom_msg.pose.pose.orientation.y;
        attitude_quater.z() = odom_msg.pose.pose.orientation.z;//NED
        Eigen::Matrix3d rota = attitude_quater.toRotationMatrix();
        Eigen::Matrix3d nedtoenu;
        nedtoenu << 0,1,0,
                    1,0,0,
                    0,0,-1;//enu
        Eigen::Matrix3d body1tobody2;
        body1tobody2<<1,0,0,
                      0,-1,0,
                      0,0,-1;
        rota = nedtoenu*rota;
        rota = rota*body1tobody2;
        Eigen::Quaterniond enuq(rota);

        odom_msg.pose.pose.orientation.w = enuq.w();
        odom_msg.pose.pose.orientation.x = enuq.x();
        odom_msg.pose.pose.orientation.y = enuq.y();
        odom_msg.pose.pose.orientation.z = enuq.z();

        // roll = atan2(2 * (attitude_quater.w() * attitude_quater.x() + attitude_quater.y() * attitude_quater.z()), 1 - 2 * (attitude_quater.x() * attitude_quater.x() + attitude_quater.y() * attitude_quater.y()));
        // pitch = asin(2 * (attitude_quater.w() * attitude_quater.y() - attitude_quater.z() * attitude_quater.x()));
        // yaw = atan2(2 * (attitude_quater.w() * attitude_quater.z() + attitude_quater.x() * attitude_quater.y()), 1 - 2 * (attitude_quater.y() * attitude_quater.y() + attitude_quater.z() * attitude_quater.z()));
        // //NED
        // double enu_roll,enu_pitch,enu_yaw;
        // enu_roll = pitch;
        // enu_pitch = roll;
        // enu_yaw = M_PI_2-yaw;
        // Eigen::Vector3d uAngle(enu_yaw,enu_pitch,enu_roll);
        // Eigen::Quaterniond quaternion = ypr_to_quaternion(uAngle);
        // odom_msg.pose.pose.orientation.w = quaternion.w();
        // odom_msg.pose.pose.orientation.x = quaternion.x();
        // odom_msg.pose.pose.orientation.y = quaternion.y(); 
        // odom_msg.pose.pose.orientation.z = quaternion.z();

        std::swap(odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y);
        odom_msg.twist.twist.linear.z = -odom_msg.twist.twist.linear.z;
        // std::swap(odom_msg.twist.twist.angular.x, odom_msg.twist.twist.angular.y);
        // odom_msg.twist.twist.angular.z = -odom_msg.twist.twist.angular.z;
        odom_msg.twist.twist.angular.y = -odom_msg.twist.twist.angular.y;
        odom_msg.twist.twist.angular.z = -odom_msg.twist.twist.angular.z;
    }
    return odom_msg;

}

nav_msgs::Odometry AirsimROSWrapper::get_odom_msg_from_multirotor_state(const msr::airlib::MultirotorState& drone_state) const
{
    nav_msgs::Odometry odom_msg;

    // ! Update Drone state -> Angle
    Eigen::Quaterniond q_drone, q_offset_do, q_adjust;
    q_drone.x() = drone_state.getOrientation().x();
    q_drone.y() = drone_state.getOrientation().y();
    q_drone.z() = drone_state.getOrientation().z();
    q_drone.w() = drone_state.getOrientation().w();
    q_offset_do.w() = cos(0.25*M_PI);
    q_offset_do.x() = 0.0;
    q_offset_do.y() = 0.0;
    q_offset_do.z() = -sin(0.25*M_PI);
    q_drone = q_offset_do*q_drone;

    q_adjust.x() = -q_drone.y();
    q_adjust.y() = q_drone.x();
    q_adjust.z() = q_drone.z();
    q_adjust.w() = q_drone.w();
    q_drone = q_adjust;

    // ! Update Drone state -> Angular velocity
    Eigen::Vector3d angular_velocity_enu
    (
        drone_state.kinematics_estimated.twist.angular.x(),
        drone_state.kinematics_estimated.twist.angular.y(),
        drone_state.kinematics_estimated.twist.angular.z()
    );
    Eigen::Matrix3d rotation_matrix = Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Vector3d angular_velocity_adjust = rotation_matrix * angular_velocity_enu;

    Eigen::Matrix3d q_drone_rotation_matrix = q_drone.toRotationMatrix();
    Eigen::Vector3d angular_velocity_transformed = q_drone_rotation_matrix * angular_velocity_adjust;

    odom_msg.pose.pose.position.x = drone_state.getPosition().x();
    odom_msg.pose.pose.position.y = drone_state.getPosition().y();
    odom_msg.pose.pose.position.z = drone_state.getPosition().z() - origin_z;

    odom_msg.pose.pose.orientation.x = q_drone.x();
    odom_msg.pose.pose.orientation.y = q_drone.y();
    odom_msg.pose.pose.orientation.z = q_drone.z();
    odom_msg.pose.pose.orientation.w = q_drone.w();

    odom_msg.twist.twist.linear.x = drone_state.kinematics_estimated.twist.linear.x();
    odom_msg.twist.twist.linear.y = drone_state.kinematics_estimated.twist.linear.y();
    odom_msg.twist.twist.linear.z = drone_state.kinematics_estimated.twist.linear.z();
    odom_msg.twist.twist.angular.x = angular_velocity_transformed(0);
    odom_msg.twist.twist.angular.y = angular_velocity_transformed(1);
    odom_msg.twist.twist.angular.z = angular_velocity_transformed(2);
    

    if (isENU_)
    {
        std::swap(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y);
        odom_msg.pose.pose.position.z = -odom_msg.pose.pose.position.z;
        std::swap(odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y);        
        odom_msg.pose.pose.orientation.z = -odom_msg.pose.pose.orientation.z;
        std::swap(odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y);
        odom_msg.twist.twist.linear.z = -odom_msg.twist.twist.linear.z;
        std::swap(odom_msg.twist.twist.angular.x, odom_msg.twist.twist.angular.y);
        odom_msg.twist.twist.angular.z = -odom_msg.twist.twist.angular.z;

        // ! ENU to NWU transform
        // nav_msgs::Odometry odom_msg_nwu = odom_msg;
        // // Position
        // // odom_msg_nwu.pose.pose.position.x = odom_msg.pose.pose.position.y;
        // // odom_msg_nwu.pose.pose.position.y = odom_msg.pose.pose.position.x;

        // // Orientation
        // Eigen::Quaterniond q_enu(
        //     odom_msg.pose.pose.orientation.w,
        //     odom_msg.pose.pose.orientation.x,
        //     odom_msg.pose.pose.orientation.y,
        //     odom_msg.pose.pose.orientation.z
        // );

        // Eigen::Quaterniond q_nwu = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()) * q_enu;

        // odom_msg_nwu.pose.pose.orientation.x = q_nwu.x();
        // odom_msg_nwu.pose.pose.orientation.y = q_nwu.y();
        // odom_msg_nwu.pose.pose.orientation.z = q_nwu.z();
        // odom_msg_nwu.pose.pose.orientation.w = q_nwu.w();

        // // Linear velocity
        // // odom_msg_nwu.twist.twist.linear.x = odom_msg.twist.twist.linear.y;
        // // odom_msg_nwu.twist.twist.linear.y = odom_msg.twist.twist.linear.x;

        // // Angular velocity
        // Eigen::Vector3d angular_velocity_enu(
        //     odom_msg.twist.twist.angular.x,
        //     odom_msg.twist.twist.angular.y,
        //     odom_msg.twist.twist.angular.z
        // );

        // Eigen::Matrix3d rotation_matrix = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        // Eigen::Vector3d angular_velocity_nwu = rotation_matrix * angular_velocity_enu;

        // odom_msg_nwu.twist.twist.angular.x = angular_velocity_nwu.x();
        // odom_msg_nwu.twist.twist.angular.y = angular_velocity_nwu.y();
        // odom_msg_nwu.twist.twist.angular.z = angular_velocity_nwu.z();

        // odom_msg = std::move(odom_msg_nwu);
    }

    return odom_msg;
}

// https://docs.ros.org/jade/api/sensor_msgs/html/point__cloud__conversion_8h_source.html#l00066
// look at UnrealLidarSensor.cpp UnrealLidarSensor::getPointCloud() for math
// read this carefully https://docs.ros.org/kinetic/api/sensor_msgs/html/msg/PointCloud2.html
sensor_msgs::PointCloud2 AirsimROSWrapper::get_lidar_msg_from_airsim(const msr::airlib::LidarData& lidar_data, const std::string& vehicle_name) const
{
    sensor_msgs::PointCloud2 lidar_msg;
    // lidar_msg.header.stamp = ros::Time::now();
    lidar_msg.header.stamp = airsim_timestamp_to_ros(lidar_data.time_stamp);
    // ! NED frame
    lidar_msg.header.frame_id = vehicle_name;
    if (lidar_data.point_cloud.size() > 3)
    {
        lidar_msg.height = 1;
        lidar_msg.width = lidar_data.point_cloud.size() / 3;

        lidar_msg.fields.resize(3);
        lidar_msg.fields[0].name = "x"; 
        lidar_msg.fields[1].name = "y"; 
        lidar_msg.fields[2].name = "z";

        int offset = 0;

        for (size_t d = 0; d < lidar_msg.fields.size(); ++d, offset += 4)
        {
            lidar_msg.fields[d].offset = offset;
            lidar_msg.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
            lidar_msg.fields[d].count  = 1;
        }

        lidar_msg.is_bigendian = false;
        lidar_msg.point_step = offset; // 4 * num fields
        lidar_msg.row_step = lidar_msg.point_step * lidar_msg.width;

        lidar_msg.is_dense = true; // todo
        std::vector<float> data_std = lidar_data.point_cloud;

        const unsigned char* bytes = reinterpret_cast<const unsigned char*>(data_std.data());
        vector<unsigned char> lidar_msg_data(bytes, bytes + sizeof(float) * data_std.size());
        lidar_msg.data = std::move(lidar_msg_data);

        if (isENU_)
        {
            try
            {
                // ! ENU frame
                sensor_msgs::PointCloud2 lidar_msg_enu;
                auto transformStampedENU = tf_buffer_.lookupTransform(AIRSIM_FRAME_ID, vehicle_name, ros::Time(0), ros::Duration(1));
                tf2::doTransform(lidar_msg, lidar_msg_enu, transformStampedENU);

                lidar_msg_enu.header.stamp = lidar_msg.header.stamp;
                lidar_msg_enu.header.frame_id = lidar_msg.header.frame_id;

                lidar_msg = std::move(lidar_msg_enu);
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("LiDAR -> This frame is empty.");
                ROS_WARN("%s", ex.what());
                ros::Duration(0.1).sleep();
            }
        }
    }

    else 
    {
        // msg = []
    }

    return lidar_msg;
}

nav_msgs::Odometry AirsimROSWrapper::get_lidar_odom_from_airsim(const msr::airlib::LidarData& lidar_data, const std::string& vehicle_name) const
{
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = airsim_timestamp_to_ros(lidar_data.time_stamp);
  // ! NED frame
  auto pose = lidar_data.pose;
  odom_msg.header.frame_id = vehicle_name;
  odom_msg.child_frame_id = "/lio_odom";
  odom_msg.pose.pose.position.x = pose.position.x();
  odom_msg.pose.pose.position.y = pose.position.y();
  odom_msg.pose.pose.position.z = pose.position.z();
  odom_msg.pose.pose.orientation.x = pose.orientation.x();
  odom_msg.pose.pose.orientation.y = pose.orientation.y();
  odom_msg.pose.pose.orientation.z = pose.orientation.z();
  odom_msg.pose.pose.orientation.w = pose.orientation.w();
  if (isENU_) {
    std::swap(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y);
    odom_msg.pose.pose.position.z = -odom_msg.pose.pose.position.z;
    double roll, pitch, yaw;
    Eigen::Quaterniond attitude_quater;
    attitude_quater.w() = odom_msg.pose.pose.orientation.w;
    attitude_quater.x() = odom_msg.pose.pose.orientation.x;
    attitude_quater.y() = odom_msg.pose.pose.orientation.y;
    attitude_quater.z() = odom_msg.pose.pose.orientation.z; // NED
    Eigen::Matrix3d rota = attitude_quater.toRotationMatrix();
    Eigen::Matrix3d nedtoenu;
    nedtoenu << 0, 1, 0, 1, 0, 0, 0, 0, -1; // enu
    Eigen::Matrix3d body1tobody2;
    body1tobody2 << 1, 0, 0, 0, -1, 0, 0, 0, -1;
    rota = nedtoenu * rota;
    rota = rota * body1tobody2;
    Eigen::Quaterniond enuq(rota);

    odom_msg.pose.pose.orientation.w = enuq.w();
    odom_msg.pose.pose.orientation.x = enuq.x();
    odom_msg.pose.pose.orientation.y = enuq.y();
    odom_msg.pose.pose.orientation.z = enuq.z();

    std::swap(odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y);
    odom_msg.twist.twist.linear.z = -odom_msg.twist.twist.linear.z;
    odom_msg.twist.twist.angular.y = -odom_msg.twist.twist.angular.y;
    odom_msg.twist.twist.angular.z = -odom_msg.twist.twist.angular.z;
  }

  return odom_msg;
}

sensor_msgs::PointCloud2 AirsimROSWrapper::get_world_lidar_msg_from_airsim(const sensor_msgs::PointCloud2& local_lidar_msg, const nav_msgs::Odometry& lidar_odom) const
{
    if (local_lidar_msg.data.empty())
    {
        return local_lidar_msg;
    }

    // * Process Odometry
    tf::Quaternion tfQuat_body;
    tf::quaternionMsgToTF(lidar_odom.pose.pose.orientation, tfQuat_body);
    tf::Matrix3x3 m_body(tfQuat_body);
    double roll_body, pitch_body, yaw_body;
    m_body.getRPY(roll_body, pitch_body, yaw_body);

    Eigen::Matrix3d body2world_R_matrix;

    double yaw_cur = yaw_body - M_PI/2.0;
    body2world_R_matrix = Eigen::AngleAxisd(yaw_cur, Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(-pitch_body, Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(roll_body, Eigen::Vector3d::UnitY());
    
    Eigen::Vector3d body2world_t_vector;
    body2world_t_vector << lidar_odom.pose.pose.position.x, lidar_odom.pose.pose.position.y, lidar_odom.pose.pose.position.z;

    Eigen::Matrix3d sensor2world_R_matrix;
    Eigen::Vector3d sensor2world_t_vector;

    sensor2world_R_matrix = body2world_R_matrix * lidar_d2body_R_matrix_;
    sensor2world_t_vector = body2world_R_matrix * lidar_d2body_t_vector_ + body2world_t_vector;

    // * Process Point Cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(local_lidar_msg, *temp_cloud);

    // * Transform Point Cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = sensor2world_R_matrix;
    T.block<3,1>(0,3) = sensor2world_t_vector;
    pcl::transformPointCloud(*temp_cloud, *transformed_cloud, T);

    pcl::PointCloud<pcl::PointXYZ> cloud_publish;
    cloud_publish = *transformed_cloud;

    cloud_publish.width = cloud_publish.points.size();
    cloud_publish.height = 1;
    cloud_publish.is_dense = true;
    cloud_publish.header.frame_id = "world";

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud_publish, cloud_msg);

    return cloud_msg;
}

airsim_ros_pkgs::Environment AirsimROSWrapper::get_environment_msg_from_airsim(const msr::airlib::Environment::State& env_data) const
{
    airsim_ros_pkgs::Environment env_msg;
    env_msg.position.x = env_data.position.x();
    env_msg.position.y = env_data.position.y();
    env_msg.position.z = env_data.position.z();
    env_msg.geo_point.latitude = env_data.geo_point.latitude;
    env_msg.geo_point.longitude = env_data.geo_point.longitude;
    env_msg.geo_point.altitude = env_data.geo_point.altitude;
    env_msg.gravity.x = env_data.gravity.x();
    env_msg.gravity.y = env_data.gravity.y();
    env_msg.gravity.z = env_data.gravity.z();
    env_msg.air_pressure = env_data.air_pressure;
    env_msg.temperature = env_data.temperature;
    env_msg.air_density = env_data.temperature;

    return env_msg;
}

sensor_msgs::MagneticField AirsimROSWrapper::get_mag_msg_from_airsim(const msr::airlib::MagnetometerBase::Output& mag_data) const
{
    sensor_msgs::MagneticField mag_msg;
    mag_msg.magnetic_field.x = mag_data.magnetic_field_body.x();
    mag_msg.magnetic_field.y = mag_data.magnetic_field_body.y();
    mag_msg.magnetic_field.z = mag_data.magnetic_field_body.z();
    std::copy(std::begin(mag_data.magnetic_field_covariance), 
        std::end(mag_data.magnetic_field_covariance), 
        std::begin(mag_msg.magnetic_field_covariance));
     mag_msg.header.stamp = airsim_timestamp_to_ros(mag_data.time_stamp);

    return mag_msg;
}

// todo covariances
sensor_msgs::NavSatFix AirsimROSWrapper::get_gps_msg_from_airsim(const msr::airlib::GpsBase::Output& gps_data) const
{
    sensor_msgs::NavSatFix gps_msg;
    gps_msg.header.stamp = airsim_timestamp_to_ros(gps_data.time_stamp);
    gps_msg.latitude = gps_data.gnss.geo_point.latitude;
    gps_msg.longitude = gps_data.gnss.geo_point.longitude; 
    gps_msg.altitude = gps_data.gnss.geo_point.altitude;
    gps_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GLONASS;
    gps_msg.status.status = gps_data.gnss.fix_type;
    // gps_msg.position_covariance_type = 
    // gps_msg.position_covariance = 

    return gps_msg;
}

sensor_msgs::Range AirsimROSWrapper::get_range_from_airsim(const msr::airlib::DistanceSensorData& dist_data) const
{
    sensor_msgs::Range dist_msg;
    dist_msg.header.stamp = airsim_timestamp_to_ros(dist_data.time_stamp);
    dist_msg.range = dist_data.distance;
    dist_msg.min_range = dist_data.min_distance;
    dist_msg.max_range = dist_data.min_distance;

    return dist_msg;
}

airsim_ros_pkgs::Altimeter AirsimROSWrapper::get_altimeter_msg_from_airsim(const msr::airlib::BarometerBase::Output& alt_data) const
{
    airsim_ros_pkgs::Altimeter alt_msg;
    alt_msg.header.stamp = airsim_timestamp_to_ros(alt_data.time_stamp);
    alt_msg.altitude = alt_data.altitude;
    alt_msg.pressure = alt_data.pressure;
    alt_msg.qnh = alt_data.qnh;

    return alt_msg;
}

// todo covariances
sensor_msgs::Imu AirsimROSWrapper::get_imu_msg_from_airsim(const msr::airlib::ImuBase::Output& imu_data) const
{
    sensor_msgs::Imu imu_msg;
    // imu_msg.header.frame_id = "/airsim/odom_local_ned";// todo multiple drones
    imu_msg.header.stamp = airsim_timestamp_to_ros(imu_data.time_stamp);
    imu_msg.orientation.x = imu_data.orientation.x();
    imu_msg.orientation.y = imu_data.orientation.y();
    imu_msg.orientation.z = imu_data.orientation.z();
    imu_msg.orientation.w = imu_data.orientation.w();

    // todo radians per second
    imu_msg.angular_velocity.x = imu_data.angular_velocity.x();
    imu_msg.angular_velocity.y = imu_data.angular_velocity.y();
    imu_msg.angular_velocity.z = imu_data.angular_velocity.z();

    // meters/s2^m 
    imu_msg.linear_acceleration.x = imu_data.linear_acceleration.x();
    imu_msg.linear_acceleration.y = imu_data.linear_acceleration.y();
    imu_msg.linear_acceleration.z = imu_data.linear_acceleration.z();

    if (isENU_)
    {
        // std::swap(imu_msg.angular_velocity.x, imu_msg.angular_velocity.y);
        // imu_msg.angular_velocity.z = -imu_msg.angular_velocity.z;
        std::swap(imu_msg.orientation.x, imu_msg.orientation.y);        
        imu_msg.orientation.z = -imu_msg.orientation.z;
        // std::swap(imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y);
        imu_msg.linear_acceleration.z = -imu_msg.linear_acceleration.z;
    }

    // imu_msg.orientation_covariance = ;
    // imu_msg.angular_velocity_covariance = ;
    // imu_msg.linear_acceleration_covariance = ;

    return imu_msg;
}

void AirsimROSWrapper::publish_odom_tf(const nav_msgs::Odometry& odom_msg)
{
    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header = odom_msg.header;
    odom_tf.child_frame_id = odom_msg.child_frame_id; 
    odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
    odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
    odom_tf.transform.translation.z = odom_msg.pose.pose.position.z;

    odom_tf.transform.rotation.x = odom_msg.pose.pose.orientation.x;
    odom_tf.transform.rotation.y = odom_msg.pose.pose.orientation.y;
    odom_tf.transform.rotation.z = odom_msg.pose.pose.orientation.z;
    odom_tf.transform.rotation.w = odom_msg.pose.pose.orientation.w;

    tf_broadcaster_.sendTransform(odom_tf);
}

airsim_ros_pkgs::GPSYaw AirsimROSWrapper::get_gps_msg_from_airsim_geo_point(const msr::airlib::GeoPoint& geo_point) const
{
    airsim_ros_pkgs::GPSYaw gps_msg;
    gps_msg.latitude = geo_point.latitude;
    gps_msg.longitude = geo_point.longitude; 
    gps_msg.altitude = geo_point.altitude;
    return gps_msg;
}

sensor_msgs::NavSatFix AirsimROSWrapper::get_gps_sensor_msg_from_airsim_geo_point(const msr::airlib::GeoPoint& geo_point) const
{
    sensor_msgs::NavSatFix gps_msg;
    gps_msg.latitude = geo_point.latitude;
    gps_msg.longitude = geo_point.longitude; 
    gps_msg.altitude = geo_point.altitude;
    return gps_msg;
}

ros::Time AirsimROSWrapper::chrono_timestamp_to_ros(const std::chrono::system_clock::time_point& stamp) const
{
    auto dur = std::chrono::duration<double>(stamp.time_since_epoch());
    ros::Time cur_time;
    cur_time.fromSec(dur.count());
    return cur_time;
}

ros::Time AirsimROSWrapper::airsim_timestamp_to_ros(const msr::airlib::TTimePoint& stamp) const
{
    // airsim appears to use chrono::system_clock with nanosecond precision
    std::chrono::nanoseconds dur(stamp);
    std::chrono::time_point<std::chrono::system_clock> tp(dur);
    ros::Time cur_time = chrono_timestamp_to_ros(tp);
    return cur_time;
}

void AirsimROSWrapper::drone_state_timer_cb(const ros::TimerEvent& event)
{
    try
    { 
        // todo this is global origin
        origin_geo_point_pub_.publish(origin_geo_point_msg_);

        // get the basic vehicle pose and environmental state
        const auto now = update_state();

        // on init, will publish 0 to /clock as expected for use_sim_time compatibility
        if (!airsim_client_->simIsPaused())
        {
            // airsim_client needs to provide the simulation time in a future version of the API
            ros_clock_.clock = now;
        }
        // publish the simulation clock
        if (publish_clock_)
        {
            clock_pub_.publish(ros_clock_);
        }

        // publish vehicle state, odom, and all basic sensor types
        publish_vehicle_state();

        // send any commands out to the vehicles
        update_commands();
    }
    catch (rpc::rpc_error& e)
    {
        std::cout << "error" << std::endl;
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API:" << std::endl << msg << std::endl;
    }
}

void AirsimROSWrapper::update_and_publish_static_transforms(VehicleROS* vehicle_ros)
{
    if (vehicle_ros && !vehicle_ros->static_tf_msg_vec.empty())
    {
        for (auto& static_tf_msg : vehicle_ros->static_tf_msg_vec)
        {
            static_tf_msg.header.stamp = vehicle_ros->stamp;
            static_tf_pub_.sendTransform(static_tf_msg);
        }
    }
}

ros::Time AirsimROSWrapper::update_state()
{
    bool got_sim_time = false;
    ros::Time curr_ros_time = ros::Time::now();

    //should be easier way to get the sim time through API, something like:
    //msr::airlib::Environment::State env = airsim_client_->simGetGroundTruthEnvironment("");
    //curr_ros_time = airsim_timestamp_to_ros(env.clock().nowNanos());

    // iterate over drones
    for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_)
    {
        ros::Time vehicle_time;
        // get drone state from airsim 
        auto& vehicle_ros = vehicle_name_ptr_pair.second;

        // vehicle environment, we can get ambient temperature here and other truths
        auto env_data = airsim_client_->simGetGroundTruthEnvironment(vehicle_ros->vehicle_name);

        if (airsim_mode_ == AIRSIM_MODE::DRONE)
        {
            auto drone = static_cast<MultiRotorROS*>(vehicle_ros.get());
            auto rpc = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
            drone->curr_drone_state = rpc->getMultirotorState(vehicle_ros->vehicle_name);
            // drone->truth_state = rpc->simGetGroundTruthKinematics(vehicle_ros->vehicle_name);
            drone->truth_state = rpc->getMultirotorState(vehicle_ros->vehicle_name);

            vehicle_time = airsim_timestamp_to_ros(drone->curr_drone_state.timestamp);
            // vehicle_time = curr_ros_time;
            if (!got_sim_time)
            {
                curr_ros_time = vehicle_time;
                got_sim_time = true;
            }

            // vehicle_ros->gps_sensor_msg = get_gps_sensor_msg_from_airsim_geo_point(drone->curr_drone_state.gps_location);
            // vehicle_ros->gps_sensor_msg.header.stamp = vehicle_time;

            vehicle_ros->curr_odom = get_odom_msg_from_multirotor_state(drone->curr_drone_state);
            vehicle_ros->control_odom = get_odom_msg_from_truth_state(drone->truth_state);
            update_traj_visualization(vehicle_ros->vehicle_name, drone->curr_drone_state.getPosition(), ros::Time::now());

            // if(acquire_origin_z)  //This part is used to make the drone_1 tf just on the ground
            // {
            //     origin_z = drone->curr_drone_state.getPosition().z();
            //     // origin_z = drone->truth_state.pose.position.z();
            //     acquire_origin_z = false;
            // }
        }
        else
        {
            auto car = static_cast<CarROS*>(vehicle_ros.get());
            auto rpc = static_cast<msr::airlib::CarRpcLibClient*>(airsim_client_.get());
            car->curr_car_state = rpc->getCarState(vehicle_ros->vehicle_name);

            vehicle_time = airsim_timestamp_to_ros(car->curr_car_state.timestamp);
            // vehicle_time = curr_ros_time;
            if (!got_sim_time)
            {
                curr_ros_time = vehicle_time;
                got_sim_time = true;
            }

            // vehicle_ros->gps_sensor_msg = get_gps_sensor_msg_from_airsim_geo_point(env_data.geo_point);
            // vehicle_ros->gps_sensor_msg.header.stamp = vehicle_time; // ! FC: NO GPS msg

            vehicle_ros->curr_odom = get_odom_msg_from_car_state(car->curr_car_state);
            update_traj_visualization(vehicle_ros->vehicle_name, car->curr_car_state.getPosition(), ros::Time::now());
            
            airsim_ros_pkgs::CarState state_msg = get_roscarstate_msg_from_car_state(car->curr_car_state);
            state_msg.header.frame_id = vehicle_ros->vehicle_name;
            car->car_state_msg = state_msg;
        }

        vehicle_ros->stamp = vehicle_time;
        
        airsim_ros_pkgs::Environment env_msg = get_environment_msg_from_airsim(env_data);
        env_msg.header.frame_id = vehicle_ros->vehicle_name;
        env_msg.header.stamp = vehicle_time;
        vehicle_ros->env_msg = env_msg;

        // convert airsim drone state to ROS msgs            
        vehicle_ros->curr_odom.header.frame_id = vehicle_ros->vehicle_name;
        // vehicle_ros->curr_odom.header.frame_id = "nwu"; // ! parent -> world
        vehicle_ros->curr_odom.child_frame_id = vehicle_ros->odom_frame_id;
        vehicle_ros->curr_odom.header.stamp = vehicle_time;

        vehicle_ros->control_odom.header.frame_id = vehicle_ros->vehicle_name;
        vehicle_ros->control_odom.child_frame_id = vehicle_ros->odom_frame_id;
        vehicle_ros->control_odom.header.stamp = vehicle_time;

    }

    return curr_ros_time;
}

void AirsimROSWrapper::update_traj_visualization(const std::string& vehicle_name,
                                                 const msr::airlib::Vector3r& position,
                                                 const ros::Time& vehicle_time)
{
    std::lock_guard<std::mutex> lock(traj_viz_mutex_);
    auto& traj = traj_points_map_[vehicle_name];

    if (!traj.empty())
    {
        const auto& prev = traj.back().position;
        const double dx = position.x() - prev.x();
        const double dy = position.y() - prev.y();
        const double dz = position.z() - prev.z();
        const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (d < traj_min_spacing_)
        {
            return;
        }
    }

    traj.push_back({position, vehicle_time});
    while (traj.size() > traj_max_points_)
    {
        traj.pop_front();
    }
    if (traj_window_sec_ > 0.0)
    {
        while (!traj.empty() && (vehicle_time - traj.front().stamp).toSec() > traj_window_sec_)
        {
            traj.pop_front();
        }
    }
}

void AirsimROSWrapper::plot_traj_visualization(const std::string& vehicle_name, const ros::Time& now)
{
    std::vector<msr::airlib::Vector3r> traj_points;
    static bool traj_points_seen = false;
    {
        std::lock_guard<std::mutex> lock(traj_viz_mutex_);
        auto traj_it = traj_points_map_.find(vehicle_name);
        if (traj_it == traj_points_map_.end())
        {
            return;
        }

        auto& traj = traj_it->second;
        if (traj_window_sec_ > 0.0)
        {
            while (!traj.empty() && (now - traj.front().stamp).toSec() > traj_window_sec_)
            {
                traj.pop_front();
            }
        }
        if (traj.empty())
        {
            return;
        }

        const auto last_it = last_traj_pub_time_.find(vehicle_name);
        if (last_it != last_traj_pub_time_.end() &&
            (now - last_it->second).toSec() < traj_publish_interval_sec_)
        {
            return;
        }
        last_traj_pub_time_[vehicle_name] = now;

        traj_points.reserve(traj.size());
        for (const auto& pt : traj)
        {
            traj_points.push_back(pt.position);
        }
    }
    if (!traj_points_seen)
    {
        ROS_INFO_STREAM("Trajectory points available for " << vehicle_name << ", count=" << traj_points.size());
        traj_points_seen = true;
    }

    if (traj_points.size() == 1)
    {
        airsim_client_traj_.simPlotPoints({traj_points.front()}, traj_color_rgba_,
                                          static_cast<float>(std::max(traj_min_size_, traj_thickness_)),
                                          static_cast<float>(traj_duration_), false);
        return;
    }

    const int point_count = static_cast<int>(traj_points.size());
    const int min_bucket_count = std::max(1, traj_buckets_);
    // Keep launch param as a lower bound, then adaptively increase levels for smoother size fading.
    const int adaptive_bucket_count = std::max(min_bucket_count, static_cast<int>(std::ceil(point_count / 20.0)));
    const int bucket_count = std::max(1, std::min(point_count, std::min(adaptive_bucket_count, 48)));

    for (int b = 0; b < bucket_count; ++b)
    {
        const int idx_start = (point_count * b) / bucket_count;
        const int idx_end = (point_count * (b + 1)) / bucket_count;
        if (idx_start >= idx_end)
        {
            continue;
        }

        std::vector<msr::airlib::Vector3r> bucket_points;
        bucket_points.reserve(static_cast<size_t>(idx_end - idx_start));
        for (int i = idx_start; i < idx_end; ++i)
        {
            bucket_points.push_back(traj_points[static_cast<size_t>(i)]);
        }

        if (bucket_points.empty())
        {
            continue;
        }

        const double t = static_cast<double>(b + 1) / static_cast<double>(bucket_count);
        const double scale = std::pow(t, traj_alpha_pow_);
        auto color = traj_color_rgba_;
        if (color.size() < 4)
        {
            color = {1.0f, 0.0f, 0.0f, 1.0f};
        }
        color[0] = 1.0f;
        color[1] = 0.0f;
        color[2] = 0.0f;
        color[3] = static_cast<float>(std::max(traj_min_alpha_, static_cast<double>(color[3]) * scale));

        const float bucket_size = static_cast<float>(std::max(traj_min_size_, traj_thickness_ * scale));
        airsim_client_traj_.simPlotPoints(bucket_points, color, bucket_size,
                                          static_cast<float>(traj_duration_), false);
    }
}

void AirsimROSWrapper::traj_viz_timer_cb(const ros::TimerEvent& event)
{
    static bool traj_timer_cb_seen = false;
    if (!traj_timer_cb_seen)
    {
        ROS_INFO("Trajectory timer callback is running.");
        traj_timer_cb_seen = true;
    }

    nh_private_.param("traj/enable", enable_traj_viz_, enable_traj_viz_);
    if (!enable_traj_viz_)
    {
        return;
    }

    try
    {
        const ros::Time now = ros::Time::now();
        for (const auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_)
        {
            plot_traj_visualization(vehicle_name_ptr_pair.first, now);
        }
    }
    catch (rpc::rpc_error& e)
    {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by trajectory visualization API." << std::endl << msg << std::endl;
    }
}

void AirsimROSWrapper::publish_vehicle_state()
{
    for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_)
    {
        auto& vehicle_ros = vehicle_name_ptr_pair.second;

        // simulation environment truth
        vehicle_ros->env_pub.publish(vehicle_ros->env_msg);

        if (airsim_mode_ == AIRSIM_MODE::CAR)
        {
            // dashboard reading from car, RPM, gear, etc
            auto car = static_cast<CarROS*>(vehicle_ros.get());
            car->car_state_pub.publish(car->car_state_msg);
        }

        // odom and transforms
        vehicle_ros->odom_local_pub.publish(vehicle_ros->curr_odom);
        vehicle_ros->control_odom_pub.publish(vehicle_ros->control_odom);
        // publish_odom_tf(vehicle_ros->curr_odom); // ! This dynamic TF will conflict with vehicle_ros->odom_local_pub.

        // ground truth GPS position from sim/HITL
        vehicle_ros->global_gps_pub.publish(vehicle_ros->gps_sensor_msg);

        for (auto& sensor_publisher : vehicle_ros->sensor_pubs)
        {
            switch (sensor_publisher.sensor_type)
            {
                case SensorBase::SensorType::Barometer:
                {
                    // auto baro_data = airsim_client_->getBarometerData(sensor_publisher.sensor_name, vehicle_ros->vehicle_name);
                    // airsim_ros_pkgs::Altimeter alt_msg = get_altimeter_msg_from_airsim(baro_data);
                    // alt_msg.header.frame_id = vehicle_ros->vehicle_name;
                    // sensor_publisher.publisher.publish(alt_msg); // ! FC: NO barometer sensor                       
                    break;
                }
                case SensorBase::SensorType::Imu:
                {
                    auto imu_data = airsim_client_->getImuData(sensor_publisher.sensor_name, vehicle_ros->vehicle_name);
                    sensor_msgs::Imu imu_msg = get_imu_msg_from_airsim(imu_data);
                    imu_msg.header.frame_id = vehicle_ros->vehicle_name;
                    sensor_publisher.publisher.publish(imu_msg);
                    break;
                }
                case SensorBase::SensorType::Distance:
                {
                    // auto distance_data = airsim_client_->getDistanceSensorData(sensor_publisher.sensor_name, vehicle_ros->vehicle_name);
                    // sensor_msgs::Range dist_msg = get_range_from_airsim(distance_data);
                    // dist_msg.header.frame_id = vehicle_ros->vehicle_name;
                    // sensor_publisher.publisher.publish(dist_msg); // ! FC: NO distance sensor
                    break;
                }
                case SensorBase::SensorType::Gps:
                {
                    // auto gps_data = airsim_client_->getGpsData(sensor_publisher.sensor_name, vehicle_ros->vehicle_name);
                    // sensor_msgs::NavSatFix gps_msg = get_gps_msg_from_airsim(gps_data);
                    // gps_msg.header.frame_id = vehicle_ros->vehicle_name;
                    // sensor_publisher.publisher.publish(gps_msg); // ! FC: NO GPS sensor
                    break;
                }
                case SensorBase::SensorType::Lidar:
                {
                    // handled via callback
                    break;
                }
                case SensorBase::SensorType::Magnetometer:
                {
                    auto mag_data = airsim_client_->getMagnetometerData(sensor_publisher.sensor_name, vehicle_ros->vehicle_name);
                    sensor_msgs::MagneticField mag_msg = get_mag_msg_from_airsim(mag_data);
                    mag_msg.header.frame_id = vehicle_ros->vehicle_name;
                    sensor_publisher.publisher.publish(mag_msg);
                    break;
                }
            }
        }

        update_and_publish_static_transforms(vehicle_ros.get());
    }
}

void AirsimROSWrapper::update_commands()
{
    for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_)
    {
        auto& vehicle_ros = vehicle_name_ptr_pair.second;

        if (airsim_mode_ == AIRSIM_MODE::DRONE)
        {
            auto drone = static_cast<MultiRotorROS*>(vehicle_ros.get());

            // send control commands from the last callback to airsim
            if (drone->has_vel_cmd)
            {
                std::lock_guard<std::mutex> guard(drone_control_mutex_);
                static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->moveByVelocityAsync(drone->vel_cmd.x, drone->vel_cmd.y, drone->vel_cmd.z, vel_cmd_duration_, 
                    msr::airlib::DrivetrainType::MaxDegreeOfFreedom, drone->vel_cmd.yaw_mode, drone->vehicle_name);
            }
            drone->has_vel_cmd = false;
        }
        else
        {
            // send control commands from the last callback to airsim
            auto car = static_cast<CarROS*>(vehicle_ros.get());
            if (car->has_car_cmd)
            {
                std::lock_guard<std::mutex> guard(drone_control_mutex_);
                static_cast<msr::airlib::CarRpcLibClient*>(airsim_client_.get())->setCarControls(car->car_cmd, vehicle_ros->vehicle_name);
            }
            car->has_car_cmd = false;
        }
    }

    // Only camera rotation, no translation movement of camera 
    if (has_gimbal_cmd_)
    {
        std::lock_guard<std::mutex> guard(drone_control_mutex_);
        airsim_client_->simSetCameraPose(gimbal_cmd_.camera_name, get_airlib_pose(cam_offset_x_, cam_offset_y_, cam_offset_z_, gimbal_cmd_.target_quat), gimbal_cmd_.vehicle_name);
    }

    has_gimbal_cmd_ = false;
}

// airsim uses nans for zeros in settings.json. we set them to zeros here for handling tfs in ROS 
void AirsimROSWrapper::set_nans_to_zeros_in_pose(VehicleSetting& vehicle_setting) const
{
    if (std::isnan(vehicle_setting.position.x()))
        vehicle_setting.position.x() = 0.0;

    if (std::isnan(vehicle_setting.position.y()))
        vehicle_setting.position.y() = 0.0;

    if (std::isnan(vehicle_setting.position.z()))
        vehicle_setting.position.z() = 0.0;

    if (std::isnan(vehicle_setting.rotation.yaw))
        vehicle_setting.rotation.yaw = 0.0;

    if (std::isnan(vehicle_setting.rotation.pitch))
        vehicle_setting.rotation.pitch = 0.0;

    if (std::isnan(vehicle_setting.rotation.roll))
        vehicle_setting.rotation.roll = 0.0;
}

// if any nan's in camera pose, set them to match vehicle pose (which has already converted any potential nans to zeros)
void AirsimROSWrapper::set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, CameraSetting& camera_setting) const
{
    if (std::isnan(camera_setting.position.x()))
        camera_setting.position.x() = vehicle_setting.position.x();

    if (std::isnan(camera_setting.position.y()))
        camera_setting.position.y() = vehicle_setting.position.y();

    if (std::isnan(camera_setting.position.z()))
        camera_setting.position.z() = vehicle_setting.position.z();

    if (std::isnan(camera_setting.rotation.yaw))
        camera_setting.rotation.yaw = vehicle_setting.rotation.yaw;

    if (std::isnan(camera_setting.rotation.pitch))
        camera_setting.rotation.pitch = vehicle_setting.rotation.pitch;

    if (std::isnan(camera_setting.rotation.roll))
        camera_setting.rotation.roll = vehicle_setting.rotation.roll;

}

void AirsimROSWrapper::set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, LidarSetting& lidar_setting) const
{
    if (std::isnan(lidar_setting.position.x()))
        lidar_setting.position.x() = vehicle_setting.position.x();

    if (std::isnan(lidar_setting.position.y()))
        lidar_setting.position.y() = vehicle_setting.position.y();

    if (std::isnan(lidar_setting.position.z()))
        lidar_setting.position.z() = vehicle_setting.position.z();

    if (std::isnan(lidar_setting.rotation.yaw))
        lidar_setting.rotation.yaw = vehicle_setting.rotation.yaw;

    if (std::isnan(lidar_setting.rotation.pitch))
        lidar_setting.rotation.pitch = vehicle_setting.rotation.pitch;

    if (std::isnan(lidar_setting.rotation.roll))
        lidar_setting.rotation.roll = vehicle_setting.rotation.roll;
}

void AirsimROSWrapper::append_static_vehicle_tf(VehicleROS* vehicle_ros, const VehicleSetting& vehicle_setting)
{
    geometry_msgs::TransformStamped vehicle_tf_msg;
    vehicle_tf_msg.header.frame_id = world_frame_id_;
    vehicle_tf_msg.header.stamp = ros::Time::now();
    vehicle_tf_msg.child_frame_id = vehicle_ros->vehicle_name;
    vehicle_tf_msg.transform.translation.x = vehicle_setting.position.x();
    vehicle_tf_msg.transform.translation.y = vehicle_setting.position.y();
    vehicle_tf_msg.transform.translation.z = vehicle_setting.position.z();    
    tf2::Quaternion quat;
    quat.setRPY(vehicle_setting.rotation.roll, vehicle_setting.rotation.pitch, vehicle_setting.rotation.yaw);
    vehicle_tf_msg.transform.rotation.x = quat.x();
    vehicle_tf_msg.transform.rotation.y = quat.y();
    vehicle_tf_msg.transform.rotation.z = quat.z();
    vehicle_tf_msg.transform.rotation.w = quat.w();

    if (isENU_)
    {
        std::swap(vehicle_tf_msg.transform.translation.x, vehicle_tf_msg.transform.translation.y);
        std::swap(vehicle_tf_msg.transform.rotation.x, vehicle_tf_msg.transform.rotation.y);
        vehicle_tf_msg.transform.translation.z = -vehicle_tf_msg.transform.translation.z;
        vehicle_tf_msg.transform.rotation.z = -vehicle_tf_msg.transform.rotation.z;
    }

    vehicle_ros->static_tf_msg_vec.emplace_back(vehicle_tf_msg);
}

void AirsimROSWrapper::append_static_lidar_tf(VehicleROS* vehicle_ros, const std::string& lidar_name, const LidarSetting& lidar_setting)
{
    geometry_msgs::TransformStamped lidar_tf_msg;
    lidar_tf_msg.header.frame_id = vehicle_ros->vehicle_name + "/" + odom_frame_id_;
    lidar_tf_msg.child_frame_id = vehicle_ros->vehicle_name + "/lidar/" + lidar_name;
    lidar_tf_msg.transform.translation.x = lidar_setting.position.x();
    lidar_tf_msg.transform.translation.y = lidar_setting.position.y();
    lidar_tf_msg.transform.translation.z = lidar_setting.position.z();
    tf2::Quaternion quat;
    quat.setRPY(lidar_setting.rotation.roll, lidar_setting.rotation.pitch, lidar_setting.rotation.yaw);
    lidar_tf_msg.transform.rotation.x = quat.x();
    lidar_tf_msg.transform.rotation.y = quat.y();
    lidar_tf_msg.transform.rotation.z = quat.z();
    lidar_tf_msg.transform.rotation.w = quat.w();

    if (isENU_)
    {
        std::swap(lidar_tf_msg.transform.translation.x, lidar_tf_msg.transform.translation.y);
        std::swap(lidar_tf_msg.transform.rotation.x, lidar_tf_msg.transform.rotation.y);
        lidar_tf_msg.transform.translation.z = -lidar_tf_msg.transform.translation.z;
        lidar_tf_msg.transform.rotation.z = -lidar_tf_msg.transform.rotation.z;
    }

    vehicle_ros->static_tf_msg_vec.emplace_back(lidar_tf_msg);
}

void AirsimROSWrapper::append_static_camera_tf(VehicleROS* vehicle_ros, const std::string& camera_name, const CameraSetting& camera_setting)
{
    geometry_msgs::TransformStamped static_cam_tf_body_msg;
    static_cam_tf_body_msg.header.frame_id = vehicle_ros->vehicle_name + "/" + odom_frame_id_;
    static_cam_tf_body_msg.child_frame_id = camera_name + "_body/static";
    // static_cam_tf_body_msg.transform.translation.x = camera_setting.position.x();
    // static_cam_tf_body_msg.transform.translation.y = camera_setting.position.y();
    static_cam_tf_body_msg.transform.translation.x = camera_setting.position.y();
    static_cam_tf_body_msg.transform.translation.y = camera_setting.position.x();
    static_cam_tf_body_msg.transform.translation.z = camera_setting.position.z();
    tf2::Quaternion quat;
    quat.setRPY(camera_setting.rotation.roll, camera_setting.rotation.pitch, camera_setting.rotation.yaw);
    static_cam_tf_body_msg.transform.rotation.x = quat.x();
    static_cam_tf_body_msg.transform.rotation.y = quat.y();
    static_cam_tf_body_msg.transform.rotation.z = quat.z();
    static_cam_tf_body_msg.transform.rotation.w = quat.w();

    if (isENU_)
    {
        std::swap(static_cam_tf_body_msg.transform.translation.x, static_cam_tf_body_msg.transform.translation.y);
        std::swap(static_cam_tf_body_msg.transform.rotation.x, static_cam_tf_body_msg.transform.rotation.y);
        static_cam_tf_body_msg.transform.translation.z = -static_cam_tf_body_msg.transform.translation.z;
        static_cam_tf_body_msg.transform.rotation.z = -static_cam_tf_body_msg.transform.rotation.z;
    }

    geometry_msgs::TransformStamped static_cam_tf_optical_msg = static_cam_tf_body_msg;
    static_cam_tf_optical_msg.child_frame_id = camera_name + "_optical/static";

    tf2::Quaternion quat_cam_body;
    tf2::Quaternion quat_cam_optical;
    tf2::convert(static_cam_tf_body_msg.transform.rotation, quat_cam_body);

    // ! z -> forward, x -> right, y -> down
    tf2::Quaternion quat_cam_body_ref, q_offset_3;
    quat_cam_body_ref = quat_cam_body;
    q_offset_3.setRotation(tf2::Vector3(1, 0, 0), M_PI); // x y z w
    quat_cam_body_ref = q_offset_3*quat_cam_body_ref;

    tf2::Matrix3x3 mat_cam_body(quat_cam_body_ref); 
    tf2::Matrix3x3 mat_cam_optical;
    mat_cam_optical.setValue(mat_cam_body.getColumn(1).getX(), mat_cam_body.getColumn(2).getX(), mat_cam_body.getColumn(0).getX(), 
                             mat_cam_body.getColumn(1).getY(), mat_cam_body.getColumn(2).getY(), mat_cam_body.getColumn(0).getY(),
                             mat_cam_body.getColumn(1).getZ(), mat_cam_body.getColumn(2).getZ(), mat_cam_body.getColumn(0).getZ()); 
    mat_cam_optical.getRotation(quat_cam_optical);
    quat_cam_optical.normalize();
    tf2::convert(quat_cam_optical, static_cam_tf_optical_msg.transform.rotation);

    vehicle_ros->static_tf_msg_vec.emplace_back(static_cam_tf_body_msg);
    vehicle_ros->static_tf_msg_vec.emplace_back(static_cam_tf_optical_msg);
}

void AirsimROSWrapper::img_response_timer_cb(const ros::TimerEvent& event)
{    
    try
    {
        int image_response_idx = 0;
        for (const auto& airsim_img_request_vehicle_name_pair : airsim_img_request_vehicle_name_pair_vec_)
        {
            const std::vector<ImageResponse>& img_response = airsim_client_images_.simGetImages(airsim_img_request_vehicle_name_pair.first, airsim_img_request_vehicle_name_pair.second);

            if (img_response.size() == airsim_img_request_vehicle_name_pair.first.size()) 
            {
                process_and_publish_img_response(img_response, image_response_idx, airsim_img_request_vehicle_name_pair.second);
                image_response_idx += img_response.size();
            }            
        }
    }

    catch (rpc::rpc_error& e)
    {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, didn't get image response." << std::endl << msg << std::endl;
    }

}

void AirsimROSWrapper::lidar_timer_cb(const ros::TimerEvent& event)
{    
    try 
    {
        for (auto &vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
        if (!vehicle_name_ptr_pair.second->lidar_pubs.empty()) {
            for (auto &lidar_publisher : vehicle_name_ptr_pair.second->lidar_pubs) {
            auto lidar_data = airsim_client_lidar_.getLidarData(
                lidar_publisher.sensor_name, vehicle_name_ptr_pair.first);
            sensor_msgs::PointCloud2 lidar_msg = get_lidar_msg_from_airsim(
                lidar_data, vehicle_name_ptr_pair.first);

            // * LiDAR Odometry
            nav_msgs::Odometry lidar_odom = get_lidar_odom_from_airsim(
                lidar_data, vehicle_name_ptr_pair.first);

            // * World LiDAR Points
            sensor_msgs::PointCloud2 world_lidar_msg = get_world_lidar_msg_from_airsim(
                lidar_msg, lidar_odom);

            // * Publish
            lidar_publisher.publisher.publish(lidar_msg);
            vehicle_name_ptr_pair.second->lio_odom_pub.publish(lidar_odom);
            vehicle_name_ptr_pair.second->world_lidar_pub.publish(world_lidar_msg);
 
            }
        }
        }
    } 
    catch (rpc::rpc_error &e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, didn't get image response."
                << std::endl
                << msg << std::endl;
    }

}

cv::Mat AirsimROSWrapper::manual_decode_depth(const ImageResponse& img_response) const
{
    cv::Mat mat(img_response.height, img_response.width, CV_32FC1, cv::Scalar(0));
    int img_width = img_response.width;

    for (int row = 0; row < img_response.height; row++)
        for (int col = 0; col < img_width; col++)
            mat.at<float>(row, col) = img_response.image_data_float[row * img_width + col];
    return mat;
}

sensor_msgs::ImagePtr AirsimROSWrapper::get_img_msg_from_response(const ImageResponse& img_response,
                                                                const ros::Time curr_ros_time, 
                                                                const std::string frame_id)
{
    sensor_msgs::ImagePtr img_msg_ptr = boost::make_shared<sensor_msgs::Image>();
    img_msg_ptr->data = img_response.image_data_uint8;
    img_msg_ptr->step = img_response.width * 3; // todo un-hardcode. image_width*num_bytes
    img_msg_ptr->header.stamp = airsim_timestamp_to_ros(img_response.time_stamp);
    img_msg_ptr->header.frame_id = frame_id;
    img_msg_ptr->height = img_response.height;
    img_msg_ptr->width = img_response.width;
    img_msg_ptr->encoding = "bgr8";
    if (is_vulkan_)
        img_msg_ptr->encoding = "rgb8";
    img_msg_ptr->is_bigendian = 0;
    return img_msg_ptr;
}

sensor_msgs::ImagePtr AirsimROSWrapper::get_depth_img_msg_from_response(const ImageResponse& img_response,
                                                                        const ros::Time curr_ros_time,
                                                                        const std::string frame_id)
{
    // todo using img_response.image_data_float direclty as done get_img_msg_from_response() throws an error, 
    // hence the dependency on opencv and cv_bridge. however, this is an extremely fast op, so no big deal.
    cv::Mat depth_img = manual_decode_depth(img_response);
    sensor_msgs::ImagePtr depth_img_msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", depth_img).toImageMsg();
    depth_img_msg->header.stamp = airsim_timestamp_to_ros(img_response.time_stamp);
    depth_img_msg->header.frame_id = frame_id;
    return depth_img_msg;
}

// todo have a special stereo pair mode and get projection matrix by calculating offset wrt drone body frame?
sensor_msgs::CameraInfo AirsimROSWrapper::generate_cam_info(const std::string& camera_name,
                                                            const CameraSetting& camera_setting,
                                                            const CaptureSetting& capture_setting) const
{
    sensor_msgs::CameraInfo cam_info_msg;
    cam_info_msg.header.frame_id = camera_name + "/" + image_type_int_to_string_map_.at(capture_setting.image_type) + "_optical";
    cam_info_msg.height = capture_setting.height;
    cam_info_msg.width = capture_setting.width;
    float f_x = (capture_setting.width / 2.0) / tan(math_common::deg2rad(capture_setting.fov_degrees / 2.0));
    // todo focal length in Y direction should be same as X it seems. this can change in future a scene capture component which exactly correponds to a cine camera
    // float f_y = (capture_setting.height / 2.0) / tan(math_common::deg2rad(fov_degrees / 2.0));
    cam_info_msg.K = {f_x, 0.0, capture_setting.width / 2.0, 
                        0.0, f_x, capture_setting.height / 2.0, 
                        0.0, 0.0, 1.0};
    cam_info_msg.P = {f_x, 0.0, capture_setting.width / 2.0, 0.0,
                        0.0, f_x, capture_setting.height / 2.0, 0.0, 
                        0.0, 0.0, 1.0, 0.0};
    return cam_info_msg;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr AirsimROSWrapper::imageToPointCloud(const sensor_msgs::ImagePtr& depth_msg, const sensor_msgs::ImagePtr& rgb_msg, const CameraIntrinsics& intrinsics, float depth_scale, float max_depth)
{
    // auto t1 = std::chrono::high_resolution_clock::now();

    int depth_type;
    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
        depth_type = CV_32FC1;
    else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
        depth_type = CV_16UC1;
    else
        throw std::runtime_error("Unsupported depth encoding");

    cv_bridge::CvImageConstPtr depth_ptr = cv_bridge::toCvShare(depth_msg, depth_msg->encoding);
    cv::Mat depth = depth_ptr->image;

    bool is_rgb8 = (rgb_msg->encoding == "rgb8");
    cv_bridge::CvImageConstPtr rgb_ptr = cv_bridge::toCvShare(rgb_msg, is_rgb8 ? "rgb8" : "bgr8");
    cv::Mat rgb = rgb_ptr->image;

    int rows = depth.rows;
    int cols = depth.cols;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    cloud->points.reserve(rows * cols);

    int step = 1;

    Eigen::Vector3d xyz;
    for (int v = skip_pixels_; v < rows - skip_pixels_; v += step) {
        const float* depth_row_f = nullptr;
        // const uint16_t* depth_row_u = nullptr;
        // if (depth_type == CV_32FC1)
        depth_row_f = depth.ptr<float>(v);
        // else
        //     depth_row_u = depth.ptr<uint16_t>(v);

        const cv::Vec3b* rgb_row = rgb.ptr<cv::Vec3b>(v);

        for (int u = skip_pixels_; u < cols - skip_pixels_; u += step) {
            float d;
            // if (depth_type == CV_32FC1)
            d = depth_row_f[u] * depth_scale;
            // else
            //     d = depth_row_u[u] * depth_scale;

            if (d <= 0.5f || d > max_depth || std::isnan(d) || std::isinf(d))
                continue;

            xyz = d * scale_direction_table_[v][u];

            pcl::PointXYZRGB pt;
            pt.x = xyz[0];
            pt.y = xyz[1];
            pt.z = xyz[2];

            const cv::Vec3b& color = rgb_row[u];
            // if (is_rgb8) {
            //     pt.r = color[0];
            //     pt.g = color[1];
            //     pt.b = color[2];
            // }
            // else {
            pt.b = color[0];
            pt.g = color[1];
            pt.r = color[2];
            // }
            cloud->points.push_back(pt);
        }
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    // auto t2 = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double, std::milli> g_ms = t2 - t1;
    // double g_time = (double)g_ms.count();
    // std::cout << "Point cloud generation time: " << g_time << " ms" << std::endl;

    return cloud;
}

void AirsimROSWrapper::process_and_publish_img_response(const std::vector<ImageResponse>& img_response_vec, const int img_response_idx, const std::string& vehicle_name)
{    
    // todo add option to use airsim time (image_response.TTimePoint) like Gazebo /use_sim_time param
    ros::Time curr_ros_time = ros::Time::now(); 
    int img_response_idx_internal = img_response_idx;

    sensor_msgs::ImagePtr depth_img_msg_ptr, rgb_img_msg_ptr;
    sensor_msgs::CameraInfo cam_info_msg;
    Eigen::Vector3d body2world_t_vector;
    Eigen::Matrix3d body2world_R_matrix;

    for (const auto& curr_img_response : img_response_vec)
    {
        // todo publishing a tf for each capture type seems stupid. but it foolproofs us against render thread's async stuff, I hope. 
        // Ideally, we should loop over cameras and then captures, and publish only one tf.  
        publish_camera_tf(curr_img_response, curr_ros_time, vehicle_name, curr_img_response.camera_name);

        // todo simGetCameraInfo is wrong + also it's only for image type -1.  
        // msr::airlib::CameraInfo camera_info = airsim_client_.simGetCameraInfo(curr_img_response.camera_name);

        // update timestamp of saved cam info msgs
        camera_info_msg_vec_[img_response_idx_internal].header.stamp = curr_ros_time;
        cam_info_pub_vec_[img_response_idx_internal].publish(camera_info_msg_vec_[img_response_idx_internal]);

        // DepthPlanner / DepthPerspective / DepthVis / DisparityNormalized
        if (curr_img_response.pixels_as_float)
        {
            depth_img_msg_ptr = get_depth_img_msg_from_response(curr_img_response, 
                                                    curr_ros_time, 
                                                    curr_img_response.camera_name + "_optical");
            cam_info_msg = camera_info_msg_vec_[img_response_idx_internal];
            image_pub_vec_[img_response_idx_internal].publish(depth_img_msg_ptr);

            geometry_msgs::QuaternionStamped gimbal_quat_msg;
            Eigen::Quaterniond quat = Eigen::Quaterniond(curr_img_response.camera_orientation.w(), curr_img_response.camera_orientation.x(), 
                                                          curr_img_response.camera_orientation.y(), curr_img_response.camera_orientation.z());
            
            // ! Right-hand Coordinate System
            tf::Quaternion img_quat_tf(curr_img_response.camera_orientation.x(), 
                                       curr_img_response.camera_orientation.y(), 
                                       curr_img_response.camera_orientation.z(),
                                       curr_img_response.camera_orientation.w());
            tf::Quaternion quat_offset;
            double angle = 0.25 * M_PI;
            quat_offset.setValue(0.0, 0.0, -sin(angle), cos(angle));
            img_quat_tf = quat_offset * img_quat_tf;

            gimbal_quat_msg.quaternion.w = img_quat_tf.w();
            gimbal_quat_msg.quaternion.x = img_quat_tf.x();
            gimbal_quat_msg.quaternion.y = img_quat_tf.y();
            gimbal_quat_msg.quaternion.z = img_quat_tf.z();

            tf::Matrix3x3 m_body(img_quat_tf);
            double roll_body, pitch_body, yaw_body;
            m_body.getRPY(roll_body, pitch_body, yaw_body);
            body2world_R_matrix = Eigen::AngleAxisd(-yaw_body, Eigen::Vector3d::UnitZ()) *
                                  Eigen::AngleAxisd(-pitch_body, Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(roll_body, Eigen::Vector3d::UnitX());

            body2world_t_vector << curr_img_response.camera_position.y(), 
                                   curr_img_response.camera_position.x(), 
                                   -curr_img_response.camera_position.z();
            
            // ! ENU Coordinate System
            gimbal_quat_msg.header.stamp = airsim_timestamp_to_ros(curr_img_response.time_stamp);
            gimbal_quat_msg.header.frame_id = curr_img_response.camera_name+"_gimbal";
            gimbal_quat_pub_.publish(gimbal_quat_msg);
        }
        // Scene / Segmentation / SurfaceNormals / Infrared
        else
        {
            rgb_img_msg_ptr = get_img_msg_from_response(curr_img_response, 
                                                    curr_ros_time, 
                                                    curr_img_response.camera_name + "_optical");
            image_pub_vec_[img_response_idx_internal].publish(rgb_img_msg_ptr);
        }
        img_response_idx_internal++;
    }

    if (depth_img_msg_ptr == nullptr || rgb_img_msg_ptr == nullptr)
    {
        ROS_ERROR("No depth or rgb image received. Cannot publish rgbd point cloud.");
        return;
    }

    // * transformation matrix
    Eigen::Matrix3d sensor2world_R_matrix;
    Eigen::Vector3d sensor2world_t_vector;

    Eigen::Matrix3d img2cam_R_matrix;
    img2cam_R_matrix << 0.0, 0.0, 1.0,
                        -1.0, 0.0, 0.0,
                        0.0, -1.0, 0.0;
    Eigen::Vector3d img2cam_t_vector;
    img2cam_t_vector << 0.0, 0.0, 0.0;

    sensor2world_R_matrix = body2world_R_matrix * cam_d2body_R_matrix_ * img2cam_R_matrix;
    sensor2world_t_vector = body2world_R_matrix * cam_d2body_R_matrix_ * img2cam_t_vector
                            + body2world_R_matrix * cam_d2body_t_vector_
                            + body2world_t_vector;

    // * get rgb cloud from gimbal camera in cam frame to world frame
    CameraIntrinsics intrin;
    intrin.fx = cam_info_msg.K[0];
    intrin.fy = cam_info_msg.K[4];
    intrin.cx = cam_info_msg.K[2];
    intrin.cy = cam_info_msg.K[5];
    rgb_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    rgb_cloud_ = imageToPointCloud(depth_img_msg_ptr, rgb_img_msg_ptr, intrin, 1.0f, cam_max_depth_);

    transformed_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = sensor2world_R_matrix;
    T.block<3, 1>(0, 3) = sensor2world_t_vector;
    pcl::transformPointCloud(*rgb_cloud_, *transformed_cloud_, T);

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*transformed_cloud_, output_msg);
    output_msg.header.stamp = rgb_img_msg_ptr->header.stamp;
    output_msg.header.frame_id = "world";

    world_img_rgb_pcd_pub_.publish(output_msg);
}

// publish camera transforms
// camera poses are obtained from airsim's client API which are in (local) NED frame. 
// We first do a change of basis to camera optical frame (Z forward, X right, Y down) 
void AirsimROSWrapper::publish_camera_tf(const ImageResponse& img_response, const ros::Time& ros_time, const std::string& frame_id, const std::string& child_frame_id)
{
    geometry_msgs::TransformStamped cam_tf_body_msg;
    cam_tf_body_msg.header.stamp = ros_time;
    cam_tf_body_msg.header.frame_id = frame_id;
    cam_tf_body_msg.child_frame_id = child_frame_id + "_body";
    cam_tf_body_msg.transform.translation.x = img_response.camera_position.x();
    cam_tf_body_msg.transform.translation.y = img_response.camera_position.y();
    cam_tf_body_msg.transform.translation.z = img_response.camera_position.z();
    cam_tf_body_msg.transform.rotation.x = img_response.camera_orientation.x();
    cam_tf_body_msg.transform.rotation.y = img_response.camera_orientation.y();
    cam_tf_body_msg.transform.rotation.z = img_response.camera_orientation.z();
    cam_tf_body_msg.transform.rotation.w = img_response.camera_orientation.w();

    if (isENU_)
    {
        std::swap(cam_tf_body_msg.transform.translation.x, cam_tf_body_msg.transform.translation.y);
        std::swap(cam_tf_body_msg.transform.rotation.x, cam_tf_body_msg.transform.rotation.y);
        cam_tf_body_msg.transform.translation.z = -cam_tf_body_msg.transform.translation.z;
        cam_tf_body_msg.transform.rotation.z = -cam_tf_body_msg.transform.rotation.z;
    }

    geometry_msgs::TransformStamped cam_tf_optical_msg;
    cam_tf_optical_msg.header.stamp = ros_time;
    cam_tf_optical_msg.header.frame_id = frame_id;
    cam_tf_optical_msg.child_frame_id = child_frame_id + "_optical";
    cam_tf_optical_msg.transform.translation.x = cam_tf_body_msg.transform.translation.x;
    cam_tf_optical_msg.transform.translation.y = cam_tf_body_msg.transform.translation.y;
    cam_tf_optical_msg.transform.translation.z = cam_tf_body_msg.transform.translation.z;

    tf2::Quaternion quat_cam_body;
    tf2::Quaternion quat_cam_optical;
    tf2::convert(cam_tf_body_msg.transform.rotation, quat_cam_body);
    tf2::Matrix3x3 mat_cam_body(quat_cam_body); 
    // tf2::Matrix3x3 mat_cam_optical = matrix_cam_body_to_optical_ * mat_cam_body * matrix_cam_body_to_optical_inverse_;
    // tf2::Matrix3x3 mat_cam_optical = matrix_cam_body_to_optical_ * mat_cam_body;
    tf2::Matrix3x3 mat_cam_optical;
    mat_cam_optical.setValue(mat_cam_body.getColumn(1).getX(), mat_cam_body.getColumn(2).getX(), mat_cam_body.getColumn(0).getX(), 
                             mat_cam_body.getColumn(1).getY(), mat_cam_body.getColumn(2).getY(), mat_cam_body.getColumn(0).getY(),
                             mat_cam_body.getColumn(1).getZ(), mat_cam_body.getColumn(2).getZ(), mat_cam_body.getColumn(0).getZ()); 
    mat_cam_optical.getRotation(quat_cam_optical);
    quat_cam_optical.normalize();
    tf2::convert(quat_cam_optical, cam_tf_optical_msg.transform.rotation);

    // tf_broadcaster_.sendTransform(cam_tf_body_msg);
    // tf_broadcaster_.sendTransform(cam_tf_optical_msg);
}

void AirsimROSWrapper::convert_yaml_to_simple_mat(const YAML::Node& node, SimpleMatrix& m) const
{
    int rows, cols;
    rows = node["rows"].as<int>();
    cols = node["cols"].as<int>();
    const YAML::Node& data = node["data"];
    for (int i = 0; i < rows*cols; ++i)
    {
        m.data[i] = data[i].as<double>();
    }
}

void AirsimROSWrapper::read_params_from_yaml_and_fill_cam_info_msg(const std::string& file_name, sensor_msgs::CameraInfo& cam_info) const
{
    std::ifstream fin(file_name.c_str());
    YAML::Node doc = YAML::Load(fin);

    cam_info.width = doc[WIDTH_YML_NAME].as<int>();
    cam_info.height = doc[HEIGHT_YML_NAME].as<int>();

    SimpleMatrix K_(3, 3, &cam_info.K[0]);
    convert_yaml_to_simple_mat(doc[K_YML_NAME], K_);
    SimpleMatrix R_(3, 3, &cam_info.R[0]);
    convert_yaml_to_simple_mat(doc[R_YML_NAME], R_);
    SimpleMatrix P_(3, 4, &cam_info.P[0]);
    convert_yaml_to_simple_mat(doc[P_YML_NAME], P_);

    cam_info.distortion_model = doc[DMODEL_YML_NAME].as<std::string>();

    const YAML::Node& D_node = doc[D_YML_NAME];
    int D_rows, D_cols;
    D_rows = D_node["rows"].as<int>();
    D_cols = D_node["cols"].as<int>();
    const YAML::Node& D_data = D_node["data"];
    cam_info.D.resize(D_rows*D_cols);
    for (int i = 0; i < D_rows*D_cols; ++i)
    {
        cam_info.D[i] = D_data[i].as<float>();
    }
}
