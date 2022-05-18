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

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// why
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
// end

#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"



Estimator estimator;

queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
std::mutex m_buf;

queue<sensor_msgs::PointCloud2ConstPtr> lidar_buf; // why
double MINIMUM_RANGE = 0.1;  // why


// why 2021.1.25
void save_timestamp(double &t, string filename)
{
    ofstream time("/home/why/SLAM_Fusion/PLSL-SLAM/src/plsl-slam/result/time_stamp/time_" + filename + ".txt", ios::app);
    time.setf(ios::fixed, ios::floatfield);
    time.precision(3);
    time << t << endl;
}


void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);

    // why 2021.1.25保存时间戳
    double t = img_msg->header.stamp.toSec();
    save_timestamp(t, "imgL");
    // end

    m_buf.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);

    // why 2021.1.25保存时间戳
    double t = img_msg->header.stamp.toSec();
    save_timestamp(t, "imgR");
    // end

    m_buf.unlock();
}

// why 2020.10.30===========================================
void lidar_callback(const sensor_msgs::PointCloud2ConstPtr &lidar_msg)
{
    m_buf.lock();
    lidar_buf.push(lidar_msg);

    // why 2021.1.25保存时间戳
    double t = lidar_msg->header.stamp.toSec();
    save_timestamp(t, "lidar");
    // end

    m_buf.unlock();
}

void removeClosedPointCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
                              pcl::PointCloud<pcl::PointXYZ> &cloud_out, float thres)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres
                && cloud_in.points[i].z > 0)  // why 去掉 相机后面的 点云, cloud_in.points[i].z > 0 好像没啥用
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }

    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}


pcl::PointCloud<pcl::PointXYZ> getPointFromLidar(const sensor_msgs::PointCloud2ConstPtr &lidar_msg)
{
    pcl::PointCloud<pcl::PointXYZ> lidar_points;
    lidar_points.clear();

    pcl::fromROSMsg(*lidar_msg, lidar_points);

    // std::cout << "before lidar_points size is = " << lidar_points.size() << std::endl;
    // why 去除无效的点,以及去除离的很近的点
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(lidar_points, lidar_points, indices);
    removeClosedPointCloud(lidar_points, lidar_points, MINIMUM_RANGE);

    // std::cout << "    after lidar_points size is = " << lidar_points.size() << std::endl;

    pcl::transformPointCloud(lidar_points, lidar_points, T_lidar2cam);
    return lidar_points;
}
// end =========================================== 


cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    
    return img;
}

// extract images with same timestamp from two topics
void sync_process()
{
    while(1)
    {
        if(STEREO && !LIDAR)
        {
            cv::Mat image0, image1;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                double time0 = img0_buf.front()->header.stamp.toSec();
                double time1 = img1_buf.front()->header.stamp.toSec();
                // 0.003s sync tolerance
                if(time0 < time1 - 0.003)
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(time0 > time1 + 0.003)
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                }
                else
                {
                    time = img0_buf.front()->header.stamp.toSec();
                    header = img0_buf.front()->header;
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop();
                    //printf("find img0 and img1\n");
                }
            }
            m_buf.unlock();
            if(!image0.empty())
                estimator.inputImage(time, image0, image1);
        }
        else if(STEREO && LIDAR)  // why 双目 +　Lidar --> PLSL-SLAM
        {
            cv::Mat image0, image1;
            pcl::PointCloud<pcl::PointXYZ> lidar_points;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty() && !lidar_buf.empty())
            {
                double time0 = img0_buf.front()->header.stamp.toSec();
                double time1 = img1_buf.front()->header.stamp.toSec();
                double time2 = lidar_buf.front()->header.stamp.toSec();

                // 0.003s sync tolerance
                if(time0 < time1 - 0.003 && time0 < time2 - 0.06)
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(time0 > time1 + 0.003 && time0 > time2 + 0.06)
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                    lidar_buf.pop();
                    printf("throw lidar\n");
                }
                else
                {
                    time = img0_buf.front()->header.stamp.toSec();
                    header = img0_buf.front()->header;
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop();

                    lidar_points = getPointFromLidar(lidar_buf.front()); // why
                    lidar_buf.pop();
                    // printf("find img0 and img1---------------------------------------\n");
                }
            }
            m_buf.unlock();
            if(!image0.empty()){ // why 2020-12-07 前面部分同步没啥问题 
                // std::cout << "ONLY LIDAR-STEREO------ ------------------------------ " << std::endl;
                estimator.inputSImageLidar(time, image0, image1, lidar_points); // why 2020-12-07 这函数是目前报错的原因

            }
        }
        else if(LIDAR)   // why
        {  
                cv::Mat image0;
                pcl::PointCloud<pcl::PointXYZ> lidar_points;
                std_msgs::Header header;
                double time = 0;

                m_buf.lock();
                if(!img0_buf.empty() && !lidar_buf.empty()){
                    double time0 = img0_buf.front()->header.stamp.toSec();
                    double time1 = lidar_buf.front()->header.stamp.toSec();

                    int count_throw = 0;
                    if(time0 < time1 - 0.06){
                        img0_buf.pop();
                        count_throw++;
                        printf("throw image---  %d\n", count_throw);
                    }
                    else if(time0 > time1 + 0.06){
                        lidar_buf.pop();
                        printf("throw lidar\n");
                    }
                    else{
                        time = img0_buf.front()->header.stamp.toSec();
                        header = img0_buf.front()->header;
                        image0 = getImageFromMsg(img0_buf.front());
                        img0_buf.pop();
                        lidar_points = getPointFromLidar(lidar_buf.front());
                        lidar_buf.pop();
                    }

                }
                m_buf.unlock();
                if( !image0.empty()){
                    std::cout << "ONLY LIDAR-MONO------ ------------------------------ " << std::endl;
                    // estimator.inputMImageLidar(time, image0, lidar_points); // why
                }
        }
        else
        {
            cv::Mat image;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if(!img0_buf.empty())
            {
                time = img0_buf.front()->header.stamp.toSec();
                header = img0_buf.front()->header;
                image = getImageFromMsg(img0_buf.front());
                img0_buf.pop();
            }
            m_buf.unlock();
            if(!image.empty())
                estimator.inputImage(time, image);
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    estimator.inputIMU(t, acc, gyr);

    // why 2021.1.25保存时间戳
    double t1 = imu_msg->header.stamp.toSec();
    save_timestamp(t1, "imu");
    // end


    return;
}


void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> featureFrame;
    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
    {
        int feature_id = feature_msg->channels[0].values[i];
        int camera_id = feature_msg->channels[1].values[i];
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        double p_u = feature_msg->channels[2].values[i];
        double p_v = feature_msg->channels[3].values[i];
        double velocity_x = feature_msg->channels[4].values[i];
        double velocity_y = feature_msg->channels[5].values[i];
        if(feature_msg->channels.size() > 5)
        {
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        ROS_ASSERT(z == 1);
        double depth = 0;
        Eigen::Matrix<double, 8, 1> xyz_uv_velocity_depth;
        xyz_uv_velocity_depth << x, y, z, p_u, p_v, velocity_x, velocity_y, depth;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity_depth);
    }
    double t = feature_msg->header.stamp.toSec();
    estimator.inputFeature(t, featureFrame);
    return;
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        estimator.clearState();
        estimator.setParameter();
    }
    return;
}

void imu_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use IMU!");
        estimator.changeSensorType(1, STEREO);
    }
    else
    {
        //ROS_WARN("disable IMU!");
        estimator.changeSensorType(0, STEREO);
    }
    return;
}

void cam_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use stereo!");
        estimator.changeSensorType(USE_IMU, 1);
    }
    else
    {
        //ROS_WARN("use mono camera (left)!");
        estimator.changeSensorType(USE_IMU, 0);
    }
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "plsl_slam");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if(argc != 2)
    {
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    readParameters(config_file);
    estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(n);

    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    // ros::Subscriber sub_feature = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
    ros::Subscriber sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);

    // why 2020.10.30
    ros::Subscriber sub_lidar = n.subscribe(LIDAR_TOPIC, 100, lidar_callback);


    ros::Subscriber sub_restart = n.subscribe("/vins_restart", 100, restart_callback);
    ros::Subscriber sub_imu_switch = n.subscribe("/vins_imu_switch", 100, imu_switch_callback);
    ros::Subscriber sub_cam_switch = n.subscribe("/vins_cam_switch", 100, cam_switch_callback);

    std::thread sync_thread{sync_process};
    ros::spin();

    return 0;
}
