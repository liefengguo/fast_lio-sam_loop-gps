// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>

#include <Eigen/Core>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include <livox_ros_driver/CustomMsg.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "IMU_Processing.hpp"
#include "preprocess.h"
#include "utility.h"
#include <ikd-Tree/ikd_Tree.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geoid.hpp>

#include "fast_lio_sam_loop/save_map.h"

#include "tic_toc.h"

// using namespace gtsam;

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;

// T1为雷达初始时间戳，s_plot为整个流程耗时，s_plot2特征点数量,s_plot3为kdtree增量时间，s_plot4为kdtree搜索耗时，s_plot5为kdtree删除点数量
//，s_plot6为kdtree删除耗时，s_plot7为kdtree初始大小，s_plot8为kdtree结束大小,s_plot9为平均消耗时间，s_plot10为添加点数量，s_plot11为点云预处理的总时间
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];

// 定义全局变量，用于记录时间,match_time为匹配时间，solve_time为求解时间，solve_const_H_time为求解H矩阵时间
double match_time = 0, solve_time = 0, solve_const_H_time = 0;

// kdtree_size_st为ikd-tree获得的节点数，kdtree_size_end为ikd-tree结束时的节点数，add_point_size为添加点的数量，kdtree_delete_counter为删除点的数量
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;

// runtime_pos_log运行时的log是否开启，pcd_save_en是否保存pcd文件，time_sync_en是否同步时间
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_odom_en = true;
/**************************/

float res_last[100000] = {0.0};          // 残差，点到面距离平方和
float DET_RANGE = 300.0f;                // 设置的当前雷达系中心到各个地图边缘的距离 
const float MOV_THRESHOLD = 1.5f;        // 设置的当前雷达系中心到各个地图边缘的权重
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;           // https://blog.csdn.net/weixin_43369786/article/details/129225369

string root_dir = ROOT_DIR;                                // 设置根目录
string map_file_path, lid_topic, imu_topic, gps_odom_topic;  // 设置地图文件路径以及传感器topic


double res_mean_last = 0.05, total_residual = 0.0;              // 残差平均值，残差总和
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;     // 雷达时间戳，imu时间戳

double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;  // imu预设参数

double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;   // 0.5 具体作用再看
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;  // 设置立方体长度，视野一半的角度，视野总角度，总距离，雷达结束时间，雷达初始时间
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;

vector<vector<int>>  pointSearchInd_surf; 
vector<BoxPointType> cub_needrm;         // ikd-tree中，地图需要移除的包围盒序列
vector<PointVector>  Nearest_Points; 
vector<double>       extrinT(3, 0.0);    // 外参
vector<double>       extrinR(9, 0.0);
deque<double>                     time_buffer;     // 激光雷达数据时间戳缓存队列，存储的激光雷达一帧的初始时刻
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());   // 去畸变的特征，lidar系
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());   // 畸变纠正后降采样的单帧点云，lidar系
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());  // odom系 单帧点云
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));  
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterICP;     // loop, icp matching, down sampling
pcl::VoxelGrid<PointType> downSizeFilterSurf;    // fast-lio, measure point cloud, down sampling
// pcl::VoxelGrid<PointType> downSizeFilterMap;      

KD_TREE<PointType> ikdtree;  // odom系 ikd-tree地图

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);    
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;                                  // 当前的欧拉角
V3D position_last(Zero3d);                      // 上一帧的位置
V3D Lidar_T_wrt_IMU(Zero3d);                    // T lidar to imu (imu = r * lidar + t)
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;                            // 点云和激光雷达数据
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;  
state_ikfom state_point;                          // ! 估计器的当前状态
vect3 pos_lid;                                    // world系下lidar坐标

nav_msgs::Path odom_path;                              // ! odom path
nav_msgs::Odometry odomAftMapped;                 // imu里程计
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;

// ! add-mtc-20250116, for bundle_adjustment_test, add keyframe detection
bool   scan_lidar_pub_en = false, keyframe_pub_en = false;          // 是否发布雷达坐标系下的激光点云，是否发布关键帧
bool   first_keyframe = true;                                       // 第一帧强制插入
double last_timestamp_keyframe = 0;                                 // 上一次关键帧插入的时刻
double keyframe_timestamp_th = 0.25;                                // 关键帧插入的时间间隔
bool timestamp_save_en = false;                                     // 存储激光雷达时间戳

state_ikfom last_odom_kf_state;                                     // 上一里程计坐标系下的关键帧状态量
double keyframe_trans_th = 0, keyframe_rot_th = 0;                  // 关键帧位姿增量检测
nav_msgs::Odometry lidarOdom;                                       // lidar里程计
// ! ---------------------------- add-mtc ---------------------------------

// ! add-mtc-20250331, for loop closing with odom change mode

// 点云存储服务 
string save_directory;                      // ? need init
float global_map_server_leaf_size = 0.4;    // ? need init，0.4

// frame for message and tf
string odometry_frame = "odom";             // ? need init
string map_frame = "map";                   // ? need init 

// gtsam
gtsam::NonlinearFactorGraph gtsam_graph; 
gtsam::Values initial_estimate;
gtsam::Values optimized_estimate;
gtsam::ISAM2 *isam;                         // ? need init
gtsam::Values isam_current_estimate;
Eigen::MatrixXd correct_pose_covariance;  // 从isam中拿到优化后的最新关键帧的协方差矩阵。

// 关键帧对应的点云
// 注意，如果启动显示界面，则需要是两个容器。否则显示线程可能会导致里程计线程丢帧。存储降采样点云是为了更好的控制运行内存
bool sparse_raw_point_cloud_flag = true;    // ? need init, default: true
vector<pcl::PointCloud<PointType>::Ptr> surf_cloud_keyframes;     // 存的是降采样点云
vector<pcl::PointCloud<PointType>::Ptr> laser_cloud_raw_keyframes; // 默认存降采样，可以改成所有点

// 里程计线程中的里程计位姿
float transformTobeMapped[6];   //  当前帧的位姿(odometry_frame系下)

// 关键帧位姿
pcl::PointCloud<PointType>::Ptr cloud_key_poses_3D;
pcl::PointCloud<PointTypePose>::Ptr cloud_key_poses_6D;
pcl::PointCloud<PointType>::Ptr copy_cloud_key_poses_3D;
pcl::PointCloud<PointTypePose>::Ptr copy_cloud_key_poses_6D;

// Loop closure
bool loop_closure_enable_flag;            // ? need init，true，开了关键帧检测进而判断是否使用loop
float loop_closure_frequency;             // ? need init，1.0 
float history_keyframe_search_radius;     // ? need init
float history_keyframe_search_time_diff;  // ? need init
int history_keyframe_search_num;          // ? need init
float history_keyframe_fitness_score;     // ? need init
float surrounding_keyframe_search_radius;      // ? need init，重建ikd-tree的关键帧搜索范围，50m
float surrounding_keyframe_density;            // ? need init，重建ikd-tree的关键帧密度，1.0m
float mapping_surf_leaf_size;                  // ? need init，重建ikd-tree的点云密度，0.4m
pcl::KdTreeFLANN<PointType>::Ptr kdtree_history_key_poses(new pcl::KdTreeFLANN<PointType>());

// global map visualization radius
float global_map_visualization_search_radius;  // ? need init
float global_map_visualization_pose_density;   // ? need init
float global_map_visualization_leaf_size;      // ? need init

bool aloop_Is_closed = false;
map<int, int> loop_Index_container;           // from new to old
vector<pair<int, int>> loop_Index_queue;
vector<gtsam::Pose3> loop_pose_queue;
vector<gtsam::noiseModel::Diagonal::shared_ptr> loop_noise_queue;

// fe-pose correct by isam
bool correct_fe_flag;           // 默认开启，但是如果需要进行路径规划等任务，需要关闭。
gtsam::Pose3 correct_Tmo;       // correct_fe_flag = false时使用，需要初始化
nav_msgs::Path global_path;     // path in map_frame

// Ros Publisher
ros::Publisher pubLidarOdom;
ros::Publisher pubLaserCloudFull_lidar;
ros::Publisher pubHistoryKeyFrames;
ros::Publisher pubIcpKeyFrames;
ros::Publisher pubLoopConstraintEdge;
ros::Publisher pubLaserCloudSurround;
ros::ServiceServer srvSaveMap;

std::mutex mtx;

// 关键帧检测
bool need_new_keyframe();
void keyframe_detection(const ros::Publisher & pubLidarOdom, const ros::Publisher & pubLaserCloudFull_lidar);

// 回环线程
void loop_closure_thread();
bool detect_loop_closure_distance(int *latestID, int *closestID);
bool detect_loop_closure_multi_cond(int *latestID, int *closestID);  // plus（检测静止启动、避免临近关键帧（lio-sam-6-axis），使用局部窗口里面的最老关键帧(slict)）
void perform_loop_closure();
void visualize_loop_closure();
void loop_find_near_keyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum);

// 里程计线程
void get_current_pose(state_ikfom cur_state);     // fast_lio_sam
void save_keyframes_and_factor();
void save_keyframes_and_factor_wt_update_ikf();

void add_odom_factor();
void add_odom_factor_fastlio();

void add_loop_factor();

void correct_poses();  // 回环成功的话，更新轨迹。并且if correct_fe_flag == true, 重建Ikd-tree地图
void correct_poses_wt_rebuild_ikd();

void recontruct_ikd_tree();    // ikd-tree地图重建，使用lio-sam中的局部地图搜索方案	
void extract_cloud(pcl::PointCloud<PointType>::Ptr cloud_to_extract);

// 发布全局地图
void visualize_global_map_thread();
void publish_global_map();

// 存储地图
bool save_map_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

// !  ------------------------------------- add-mtc ---------------------------------------------

// ! add-mtc-20250407, gps factor for back end optimization

bool path_map_en; 
bool use_gps;
int gps_frequence;
int gpc_factor_init_num;
bool use_gps_elevation;
float gps_cov_threshold;
float gps_distance;
bool manual_gps_init;   // ! if you sure
float manual_init_yaw;  // ! if you sure

std::vector<gtsam::GPSFactor> keyframeGPSfactor;
std::deque<nav_msgs::Odometry> gpsQueue;
pcl::PointCloud<PointType>::Ptr cloudKeyGPSPoses3D;
map<int, int> gps_index_container;

std::mutex mtxGpsInfo;
Eigen::Vector3d originLLA;
bool system_initialized = false;
bool gpsTransfromInit = false;
GeographicLib::LocalCartesian geo_converter;

ros::Subscriber subGPS;
ros::Publisher pub_map_path;         // global_path
ros::Publisher pubGPSOdometry;

void gps_handler(const nav_msgs::Odometry::ConstPtr &gpsMsg);
void update_initial_guess(); // 初始化gps，并且无视Kf检测，创建第一个gps因子
void add_gps_factor();
bool sync_gps(std::deque<nav_msgs::Odometry> &gpsBuf, nav_msgs::Odometry &aligedGps, double timestamp,
double eps_cam);
void publish_global_path();  // 发布pubGPSOdometry(natfix)+global_path

// !  ------------------------------------- add-mtc ---------------------------------------------

shared_ptr<Preprocess> p_pre(new Preprocess());   // 点云预处理器
shared_ptr<ImuProcess> p_imu(new ImuProcess());   // imu预处理器

#include "core/fast_lio_impl.hpp"
#include "core/back_end_impl.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;
    
    /***************************** fast-lio *************************/
    nh.param<bool>("publish/path_odom_en", path_odom_en, false);
    nh.param<bool>("publish/path_map_en", path_map_en, true);
    nh.param<bool>("publish/scan_publish_en", scan_pub_en, true);
    nh.param<bool>("publish/dense_publish_en", dense_pub_en, true);
    nh.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en, true);
    nh.param<bool>("publish/scan_lidarframe_pub_en", scan_lidar_pub_en, true);
    nh.param<int>("max_iteration", NUM_MAX_ITERATIONS,4);
    nh.param<string>("map_file_path", map_file_path,"");
    nh.param<string>("common/lid_topic", lid_topic,"/livox/lidar");
    nh.param<string>("common/imu_topic", imu_topic,"/livox/imu");
    nh.param<string>("common/gps_odom_topic", gps_odom_topic, "/gps_odom");
    nh.param<bool>("common/time_sync_en", time_sync_en, false);
    nh.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
    nh.param<double>("filter_size_corner", filter_size_corner_min,0.5);
    nh.param<double>("filter_size_surf", filter_size_surf_min,0.5);
    nh.param<double>("filter_size_map", filter_size_map_min,0.5);
    nh.param<double>("cube_side_length", cube_len,500);
    nh.param<float>("mapping/det_range", DET_RANGE,300.f);
    nh.param<double>("mapping/fov_degree", fov_deg,180);
    nh.param<double>("mapping/gyr_cov", gyr_cov,0.1);
    nh.param<double>("mapping/acc_cov", acc_cov,0.1);
    nh.param<double>("mapping/b_gyr_cov", b_gyr_cov,0.0001);
    nh.param<double>("mapping/b_acc_cov", b_acc_cov,0.0001);
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
    nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);
    nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    nh.param<bool>("time_save/timestamp_save_en", timestamp_save_en, false);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());


    // ***************************** frame coordinate *****************************
    nh.param<string>("frame/odometry_frame", odometry_frame, "odom");
    nh.param<string>("frame/map_frame", map_frame, "map");

    // ***************************** keyframe detection *****************************
    nh.param<bool>("publish/scan_lidarframe_pub_en", scan_lidar_pub_en, true);
	nh.param<bool>("publish/keyframe_pub_en", keyframe_pub_en, true);
	nh.param<double>("keyframe/interval_th", keyframe_timestamp_th, 0.25);
	nh.param<double>("keyframe/trans_th", keyframe_trans_th, 0.1);
	nh.param<double>("keyframe/rot_rad_th", keyframe_rot_th, 0.0872);

    // ***************************** loop closing *****************************
    nh.param<bool>("odometry_mode/correct_fe_en", correct_fe_flag, false);
    nh.param<bool>("loop/loop_en", loop_closure_enable_flag, false);
    nh.param<float>("loop/loop_frequency", loop_closure_frequency, 1.0);
    nh.param<bool>("loop/sparse_raw_point_cloud_en", sparse_raw_point_cloud_flag, true);
    nh.param<float>("loop/history_kf_search_radius", history_keyframe_search_radius, 25.0);
    nh.param<float>("loop/history_kf_search_time_diff", history_keyframe_search_time_diff, 25.0);    
    nh.param<int>("loop/history_kf_search_num", history_keyframe_search_num, 25);   
    nh.param<float>("loop/history_kf_fitness_score", history_keyframe_fitness_score, 0.4);    
    nh.param<float>("loop/surrounding_kf_search_radius", surrounding_keyframe_search_radius, 30.0);
    nh.param<float>("loop/surrounding_kf_density", surrounding_keyframe_density, 2.0);
    nh.param<float>("loop/mapping_surf_leaf_size", mapping_surf_leaf_size, 0.4);

    // ******************************** gps *********************************
    nh.param<bool>("gps/gps_en", use_gps, false);
    nh.param<int>("gps/gps_frequence", gps_frequence, 10);
    nh.param<int>("gps/gpc_factor_init_num", gpc_factor_init_num, 20);
    nh.param<bool>("gps/use_gps_elevation", use_gps_elevation, false);
    nh.param<float>("gps/gps_cov_threshold", gps_cov_threshold, 8.0);
    nh.param<float>("gps/gps_distance", gps_distance, 0.5);
    nh.param<bool>("gps/manual_gps_init", manual_gps_init, false);
    nh.param<float>("gps/manual_init_yaw", manual_init_yaw, 0.0);

    if(keyframe_pub_en && loop_closure_enable_flag)
    {
        ROS_INFO("\033[1;32m----> LIO system run with LOOP Function.\033[0m");
        if(correct_fe_flag)
            ROS_INFO("\033[1;34m----> LOOP Function with rebuild ikd-tree.\033[0m");
        else
            ROS_INFO("\033[1;34m----> LOOP Function without rebuild ikd-tree.\033[0m");
    }
    else
        ROS_INFO("\033[1;32m----> LIO system run without LOOP Function.\033[0m");
    
    if(use_gps)
        ROS_INFO("\033[1;34m----> Back end with gps factor.\033[0m");
    else    
        ROS_INFO("\033[1;34m----> Back end without gps factor.\033[0m");

    correct_Tmo = gtsam::Pose3();

    // ***************************** visualization *****************************
    nh.param<float>("visualization/global_map_visualization_search_radius", global_map_visualization_search_radius, 1000.0);
    nh.param<float>("visualization/global_map_visualization_pose_density", global_map_visualization_pose_density, 10.0);
    nh.param<float>("visualization/global_map_visualization_leaf_size", global_map_visualization_leaf_size, 0.5);

    // ******************************* service *******************************
    nh.param<string>("service/save_directory", save_directory, "/home/mtcjyb/Documents/");
    nh.param<float>("service/global_map_server_leaf_size", global_map_server_leaf_size, 0.4);
    
    // ***************************** optimization *****************************
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    isam = new gtsam::ISAM2(parameters);

    // ***************************** allocate Memory **************************
    cloud_key_poses_3D.reset(new pcl::PointCloud<PointType>());
    cloudKeyGPSPoses3D.reset(new pcl::PointCloud<PointType>());
    cloud_key_poses_6D.reset(new pcl::PointCloud<PointTypePose>());
    copy_cloud_key_poses_3D.reset(new pcl::PointCloud<PointType>());
    copy_cloud_key_poses_6D.reset(new pcl::PointCloud<PointTypePose>());

    // ***************************** init odometry pose **************************
    for (int i = 0; i < 6; ++i)
        transformTobeMapped[i] = 0;

    odom_path.header.stamp    = ros::Time::now();
    odom_path.header.frame_id = odometry_frame;

    /*** variables definition ***/
    int effect_feat_num = 0, frame_num = 0; // 雷达总帧数 
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    
    //这里没用到
    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    _featsArray.reset(new PointCloudXYZI());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterICP.setLeafSize(mapping_surf_leaf_size, mapping_surf_leaf_size, mapping_surf_leaf_size);
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    // downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

    // std::cout << "filter_size_surf_min: " << filter_size_surf_min << std::endl;

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);

    // 将函数地址传入kf对象中，用于接收特定于系统的模型及其差异
    // 通过一个函数（h_dyn_share_in）同时计算测量（z）、估计测量（h）、偏微分矩阵（h_x，h_v）和噪声协方差（R）
    // get_f: fast-lio2 eq(1) 动力学模型
    // df_dx, df_dw: fast-lio2 eq(7) 误差状态传递
    // h_share_model: 测量模型，用于计算残差, h_share_model需要进行数据关联
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

    /*** debug record ***/
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(),"w");

    ofstream fout_pre, fout_out, fout_dbg;
    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
    fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
    if (fout_pre && fout_out)
        cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
    else
        cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;
    
    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \
        nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : \
        nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
    // ros::Subscriber sub_pcl = nh.subscribe(lid_topic, 200000, standard_pcl_cbk);

    subGPS = nh.subscribe<nav_msgs::Odometry>(gps_odom_topic, 200, gps_handler);

    /*** ROS publisher initialization ***/
    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered", 100000);
    ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered_body", 100000);
    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_effected", 100000);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
            ("/Laser_map", 100000);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> 
            ("/Odometry", 100000);
    ros::Publisher pubOdomPath      = nh.advertise<nav_msgs::Path> 
            ("/odom_path", 100000);

    pubLaserCloudFull_lidar = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered_lidar", 100000);
	pubLidarOdom = nh.advertise<nav_msgs::Odometry> 
            ("/LidarOdometry", 100000);

    pubHistoryKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("loop/icp_loop_closure_history_cloud", 1);
    pubIcpKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("loop/icp_loop_closure_corrected_cloud", 1);
    pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>("/loop/loop_closure_constraints", 1);
    pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("visualization/map_global", 1);
    pubGPSOdometry = nh.advertise<sensor_msgs::NavSatFix>("gps/odometry_gps", 1);
    pub_map_path = nh.advertise<nav_msgs::Path> ("/map_path", 100000);

    srvSaveMap = nh.advertiseService("service/save_map", &save_map_service);

    std::thread loopthread(&loop_closure_thread);
    std::thread visualizeMapThread(&visualize_global_map_thread);

//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();
    while (status)
    {
        if (flg_exit) break;  // 如果有中断产生，则结束主循环
        ros::spinOnce();      // ROS消息回调处理函数，放在ROS的主循环中
        if(sync_packages(Measures))   // ROS消息回调处理函数，放在ROS的主循环中
        {
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;       // 这一步操作，可以让imu中的数据于lidar内部对齐，因此，第一帧实际上是被扔掉了。
                continue;
            }

            TicToc t_odom;
            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();

            update_initial_guess(); // ! init gps

            if(system_initialized || !use_gps)
            {
                p_imu->Process(Measures, kf, feats_undistort); // 点云去畸变到帧尾
                state_point = kf.get_x();  // 这里拿到的是预测的Tmi
                pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;  // 雷达坐标系下的机器人位置

                if (feats_undistort->empty() || (feats_undistort == NULL))
                {
                    ROS_WARN("No point, skip this scan!\n");
                    continue;
                }

                flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
                /*** Segment the map in lidar FOV ***/
                lasermap_fov_segment(); // 维护局部地图

                /*** downsample the feature points in a scan ***/
                downSizeFilterSurf.setInputCloud(feats_undistort);
                downSizeFilterSurf.filter(*feats_down_body);        // 0.5， 控制参与前端的点云数量

                t1 = omp_get_wtime();
                feats_down_size = feats_down_body->points.size();
                /*** initialize the map kdtree ***/ 
                if(ikdtree.Root_Node == nullptr)
                {
                    if(feats_down_size > 5)
                    {
                        ikdtree.set_downsample_param(filter_size_map_min);
                        feats_down_world->resize(feats_down_size);
                        for(int i = 0; i < feats_down_size; i++)
                        {
                            pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                        }
                        ikdtree.Build(feats_down_world->points);
                    }
                    continue;
                }
                int featsFromMapNum = ikdtree.validnum();
                kdtree_size_st = ikdtree.size();
                
                // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

                /*** ICP and iterated Kalman filter update ***/
                if (feats_down_size < 5)
                {
                    ROS_WARN("No point, skip this scan!\n");
                    continue;
                }
                
                normvec->resize(feats_down_size);
                feats_down_world->resize(feats_down_size);

                V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
                <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;

                if(0) // If you need to see map point, change to "if(1)"
                {
                    PointVector ().swap(ikdtree.PCL_Storage);
                    ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                    featsFromMap->clear();
                    featsFromMap->points = ikdtree.PCL_Storage;
                }

                pointSearchInd_surf.resize(feats_down_size);
                Nearest_Points.resize(feats_down_size);
                int  rematch_num = 0;
                bool nearest_search_en = true; //

                t2 = omp_get_wtime();
                
                /*** iterated state estimation ***/
                double t_update_start = omp_get_wtime();
                double solve_H_time = 0;
                kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
                state_point = kf.get_x();
                euler_cur = SO3ToEuler(state_point.rot);
                pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
                geoQuat.x = state_point.rot.coeffs()[0];
                geoQuat.y = state_point.rot.coeffs()[1];
                geoQuat.z = state_point.rot.coeffs()[2];
                geoQuat.w = state_point.rot.coeffs()[3];

                get_current_pose(state_point);                    // ! 更新transformTobeMapped

                double t_update_end = omp_get_wtime();

                /******* Publish odometry *******/
                std::cout << "frame_id: " << frame_num << std::endl;
                publish_odometry(pubOdomAftMapped);

                /*** add the feature points to map kdtree ***/
                t3 = omp_get_wtime();
                map_incremental();    // 分为需要和不需要降采样的点
                t5 = omp_get_wtime();
                
                /******* Publish points *******/
                if (path_odom_en)                    publish_odom_path(pubOdomPath);
                if (path_map_en)                     publish_global_path();
                if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull);
                if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);

                // publish_effect_world(pubLaserCloudEffect);
                // publish_map(pubLaserCloudMap);

                /******* Keyframe detection (And save imu key pose for pose_graph) *******/
                // ! ******* rebuild ikd-tree based on your need, use config to change the mode !!!! *******/
                keyframe_detection(pubLidarOdom, pubLaserCloudFull_lidar);

                /******* 激光雷达时间戳存储 *******/
                if(timestamp_save_en)
                {
                    ofstream f;
                    std::string res_dir = string(ROOT_DIR) + "result/timestamps.txt";
                    f.open(res_dir, ios::app);
                    f << fixed;
                    f << setprecision(6) << lidar_end_time << std::endl;
                    f.close();
                }

                /*** Debug variables ***/
                frame_num ++;
                if (runtime_pos_log)
                {
                    kdtree_size_end = ikdtree.size();
                    aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                    aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
                    aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                    aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
                    aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
                    aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
                    T1[time_log_counter] = Measures.lidar_beg_time;
                    s_plot[time_log_counter] = t5 - t0;
                    s_plot2[time_log_counter] = feats_undistort->points.size();
                    s_plot3[time_log_counter] = kdtree_incremental_time;
                    s_plot4[time_log_counter] = kdtree_search_time;
                    s_plot5[time_log_counter] = kdtree_delete_counter;
                    s_plot6[time_log_counter] = kdtree_delete_time;
                    s_plot7[time_log_counter] = kdtree_size_st;
                    s_plot8[time_log_counter] = kdtree_size_end;
                    s_plot9[time_log_counter] = aver_time_consu;
                    s_plot10[time_log_counter] = add_point_size;
                    time_log_counter ++;
                    printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
                    ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                    fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
                    <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
                    dump_lio_state_to_log(fp);
                }

                std::cout << "odometry cost time: " << t_odom.toc() << " ms" << std::endl << std::endl;
            }
        }

        status = ros::ok();
        rate.sleep();
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name<<endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    fout_out.close();
    fout_pre.close();

    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    loopthread.join(); //  分离线程
    visualizeMapThread.join();

    return 0;
}
