/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "../estimator/parameters.h"
#include "../utility/tic_toc.h"
#include "linefeature_tracker.h"


using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt);
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

// why
struct LidarPoint { // single lidar point in space
    double x,y,z,r; // x,y,z in [m], r is point reflectivity 点反射率
};
// endl

class FeatureTracker
{
public:
    FeatureTracker();
    
    map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    
    // why 2020.11.3===============================================================
    vector<pair<double, cv::Point2f>> transPtsType(vector<cv::Point2f> &pts);

    map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> trackLImage(double _cur_time, const cv::Mat &_img, 
                                                                        const pcl::PointCloud<pcl::PointXYZ>& _lidar_points, 
                                                                        const cv::Mat &_img1);

    void depthCLoudProj_kdtree(const pcl::PointCloud<pcl::PointXYZ>& curlidarpoints);
    void dataAssociatCamLidar(vector< pair<double, cv::Point2f> > &curpts);

    void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                    vector<int> &curLeftIds,
                    //    const pair<vector<double>, vector<cv::Point2f> > &curLeftPts,
                    const vector<pair<double, cv::Point2f> > &curLeftPts,
                    vector<cv::Point2f> &curRightPts,
                    const pcl::PointCloud<pcl::PointXYZ> &lidarpoints,
                    map<int, cv::Point2f> &prevLeftPtsMap);
    // end ===================== ==========================================
    
    
    void setMask();
    void readIntrinsicParameter(const vector<string> &calib_file);
    void showUndistortion(const string &name);
    // why娣诲姞
    void ExtractEdgeFeature(std::vector<cv::Point2f> &total_pts, int NeedNum);
    // 
    void rejectWithF();
    void undistortedPoints();
    vector<cv::Point2f> undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam);
    vector<cv::Point2f> ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                    map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts);
    void showTwoImage(const cv::Mat &img1, const cv::Mat &img2, 
                      vector<cv::Point2f> pts1, vector<cv::Point2f> pts2);
    void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                                   vector<int> &curLeftIds,
                                   vector<cv::Point2f> &curLeftPts, 
                                   vector<cv::Point2f> &curRightPts,
                                   map<int, cv::Point2f> &prevLeftPtsMap);
    void setPrediction(map<int, Eigen::Vector3d> &predictPts);
    double distance(cv::Point2f &pt1, cv::Point2f &pt2);
    void removeOutliers(set<int> &removePtsIds);
    cv::Mat getTrackImage();
    bool inBorder(const cv::Point2f &pt);

    int row, col;
    cv::Mat imTrack;
    cv::Mat mask;
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img;
    vector<cv::Point2f> n_pts;
    vector<cv::Point2f> predict_pts;
    vector<cv::Point2f> predict_pts_debug;
    vector<cv::Point2f> prev_pts, cur_pts, cur_right_pts;
    vector<cv::Point2f> prev_un_pts, cur_un_pts, cur_un_right_pts;
    vector<cv::Point2f> pts_velocity, right_pts_velocity;
    vector<int> ids, ids_right;
    vector<int> track_cnt;
    map<int, cv::Point2f> cur_un_pts_map, prev_un_pts_map;
    map<int, cv::Point2f> cur_un_right_pts_map, prev_un_right_pts_map;
    map<int, cv::Point2f> prevLeftPtsMap;
    vector<camodocal::CameraPtr> m_camera;
    double cur_time;
    double prev_time;
    bool stereo_cam;
    int n_id;
    bool hasPrediction;


    // why 2020.11.3 修改的原特征点数据结构
    vector<pair<double, cv::Point2f>> prev_pts_pair_, cur_pts_pair_, cur_right_pts_pair_;
    vector<pair<double, cv::Point2f>> prev_un_pts_pair_, cur_un_pts_pair_, cur_un_right_pts_pair_;
    int count_d = 0; // foe debug

    // why 2020.11.3 激光点云相关
    pcl::PointCloud<pcl::PointXYZ> prev_lidar_, cur_lidar_;
    vector<cv::Point2f> cur_lidar_pts_;
    vector<int> ids_lidar_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr depthCloud_;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr KdTree_;
    std::vector<int> pointSearchInd_;
    std::vector<float> pointSearchSqrDis_;
    int depthCloudNum_ = 0;

    vector<LidarPoint> lidar_point_;
    // end
};
