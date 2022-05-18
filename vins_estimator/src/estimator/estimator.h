/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once
 
#include <thread>
#include <mutex>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <ceres/ceres.h>
#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "parameters.h"
#include "feature_manager.h"
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../initial/solve_5pts.h"
#include "../initial/initial_sfm.h"
#include "../initial/initial_alignment.h"
#include "../initial/initial_ex_rotation.h"
#include "../factor/imu_factor.h"
#include "../factor/projection_factor.h"
#include "../factor/pose_local_parameterization.h"
#include "../factor/marginalization_factor.h"
#include "../factor/projectionTwoFrameOneCamFactor.h"
#include "../factor/projectionTwoFrameTwoCamFactor.h"
#include "../factor/projectionOneFrameTwoCamFactor.h"

#include "../factor/line_parameterization.h"
#include "../factor/line_projection_factor.h"
#include "../factor/lidarFactor.hpp" // why 2021.1.7

#include "../featureTracker/feature_tracker.h"
#include "../featureTracker/linefeature_tracker.h"
#include "../featureTracker/lidar_feature_tracker.h" // why


class Estimator
{
  public:
    Estimator();
    ~Estimator();
    void setParameter();

    // interface
    void initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r);
    void inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity);
    void inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> &featureFrame);
    void inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());    
    void processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> &image, const double header);
    
    // why 2020.11.3 加入激光雷达之后修改的input函数
    void inputSImageLidar(double t, const cv::Mat &_img, const cv::Mat &_img1, const pcl::PointCloud<pcl::PointXYZ> &_lidar_points); 

    // end

    // ----------------------- why 2019-12-07 ： 线特征相关处理函数-------------
    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> &image, 
                      const map<int, vector<pair<int, Vector4d>>> &lines, 
                      const double header,
                      const pcl::PointCloud<PointType>::Ptr  cornerPointsSharp);
    void optimizationwithLine(); 
    void onlyLineOpt();
    void LineBA();
    void LineBAincamera();
    // ---------------------------------end---------------------------------
    
    void processMeasurements();
    void changeSensorType(int use_imu, int use_stereo);


    //----------------------------------------  why：：odom部分-暂时没用----------------------
    void inputOdom(double t, const int &left_pulse, const int &right_pulse);    // 接收脉冲数据,在odom_callback使用
    void processOdom(double t, const int &left_pulse, const int &right_pulse);  // 使用脉冲计算出旋转（绕z轴）和平移（x、y方向）
    void cal_pulse(int &current, int &receive, int &delta);

    queue<pair<double, int>> left_pulse_Buf;
    queue<pair<double, int>> right_pulse_Buf; 
    double reduction_ratio_ = 2.5;          // 减速比。 以Autorlabr参考
    double encoder_resolution_ = 1600.0;    // 编码器转一圈产生的脉冲数
    double wheel_diameter_ = 0.15;          // 车轮直径
    double model_param_cw_ = 0.78;          // 顺时针旋转运动模型参数
    double model_param_acw_ = 0.78;         // 逆时针旋转运动模型参数
    double pid_rate_ = 50.0;                // PID控制频率
    double pulse_per_cycle_;
    double accumulation_x_, accumulation_y_, accumulation_th_;

    int cur_left_, cur_right_, delta_left_, delta_right_;
    // ---------------------------------------- end ----------------------------------------

    // internal
    void clearState();
    bool initialStructure();
    bool visualInitialAlign();
    bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    void slideWindow();
    void slideWindowNew();
    void slideWindowOld();
    void optimization();
    void vector2double();
    void double2vector();
    void double2vector2(); // why
    bool failureDetection();
    bool getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector, 
                                              vector<pair<double, Eigen::Vector3d>> &gyrVector);
    void getPoseInWorldFrame(Eigen::Matrix4d &T);
    void getPoseInWorldFrame(int index, Eigen::Matrix4d &T);
    void predictPtsInNextFrame();
    void outliersRejection(set<int> &removeIndex);
    double reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                     Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, 
                                     double depth, Vector3d &uvi, Vector3d &uvj);
    void updateLatestStates();
    void fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity);
    bool IMUAvailable(double t);
    void initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector);

    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };


    std::mutex mProcess;
    std::mutex mBuf;
    queue<pair<double, Eigen::Vector3d>> accBuf;
    queue<pair<double, Eigen::Vector3d>> gyrBuf;
    queue<pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 8, 1> > > > > > featureBuf;

    // why: 线特征相关
    queue<pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 4, 1>> > > > > linefeatureBuf;

    // why: 激光雷达特征 2021.1.7
    queue<pcl::PointCloud<PointType> > lidarfeatureBuf;


    // 计数计时
    double frame_cnt_ = 0;
    double sum_solver_time_ = 0.0;
    double mean_solver_time_ = 0.0;
    double sum_marg_time_ = 0.0;
    double mean_marg_time_=0.0;
    // end

    // why:: odom变量
    // end


    double prevTime, curTime;
    bool openExEstimation;

    std::thread trackThread;
    std::thread processThread;

    FeatureTracker featureTracker;
    LineFeatureTracker linefeatureTracker;      // why
    Lidar_feature_tracker lidarfeatureTracker;  // why 2021.1.7

    SolverFlag solver_flag;
    MarginalizationFlag  marginalization_flag;
    Vector3d g;

    Matrix3d ric[2];
    Vector3d tic[2];

    Vector3d        Ps[(WINDOW_SIZE + 1)];
    Vector3d        Vs[(WINDOW_SIZE + 1)];
    Matrix3d        Rs[(WINDOW_SIZE + 1)];
    Vector3d        Bas[(WINDOW_SIZE + 1)];
    Vector3d        Bgs[(WINDOW_SIZE + 1)];
    double td;

    Matrix3d back_R0, last_R, last_R0;
    Vector3d back_P0, last_P, last_P0;
    double Headers[(WINDOW_SIZE + 1)];

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    Vector3d acc_0, gyr_0;

    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

    int frame_count;
    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;
    int inputImageCnt;

    FeatureManager f_manager;
    MotionEstimator m_estimator;
    InitialEXRotation initial_ex_rotation;

    bool first_imu;
    bool is_valid, is_key;
    bool failure_occur;

    vector<Vector3d> point_cloud;
    vector<Vector3d> margin_cloud;
    vector<Vector3d> key_poses;
    double initial_timestamp;


    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double para_Feature[NUM_OF_F][SIZE_FEATURE];

    double para_LineFeature[NUM_OF_F][SIZE_LINE]; // why

    double para_Ex_Pose[2][SIZE_POSE];
    double para_Retrive_Pose[SIZE_POSE];
    double para_Td[1][1];
    double para_Tr[1][1];

    int loop_window_index;

    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;

    map<double, ImageFrame> all_image_frame;
    IntegrationBase *tmp_pre_integration;

    Eigen::Vector3d initP;
    Eigen::Matrix3d initR;

    double latest_time;
    Eigen::Vector3d latest_P, latest_V, latest_Ba, latest_Bg, latest_acc_0, latest_gyr_0;
    Eigen::Quaterniond latest_Q;

    bool initFirstPoseFlag;
    bool initThreadFlag;
};
