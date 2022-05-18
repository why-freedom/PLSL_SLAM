#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>

ros::Publisher pointcloud_pub;
ros::Publisher limage_pub;
ros::Publisher rimage_pub;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

void callback(const sensor_msgs::PointCloud2ConstPtr& pointcloud,const sensor_msgs::ImageConstPtr& limage, const sensor_msgs::ImageConstPtr& rimage)
{
    pointcloud_pub.publish(pointcloud);
    limage_pub.publish(limage);
    rimage_pub.publish(rimage);

    ROS_INFO("pointcloud stamp value is: %f", pointcloud->header.stamp.toSec());
    ROS_INFO("limage stamp value is: %f", limage->header.stamp.toSec());
    ROS_INFO("rimage stamp value is: %f", rimage->header.stamp.toSec());

    ROS_INFO("sync succeed.");
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"data_sync_node");
    ros::NodeHandle nh;

    pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("sync/pointcloud",1);
    limage_pub = nh.advertise<sensor_msgs::Image>("sync/limage",1);
    rimage_pub = nh.advertise<sensor_msgs::Image>("sync/rimage",1);

    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub(nh,"/rslidar_points",1);
    message_filters::Subscriber<sensor_msgs::Image> limage_sub(nh,"/mynteye/left/image_mono",1);
    message_filters::Subscriber<sensor_msgs::Image> rimage_sub(nh,"/mynteye/right/image_mono",1);

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(60),pointcloud_sub, limage_sub, rimage_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));



    //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image>    approximate_policy;
    //message_filters::Synchronizer<approximate_policy> sync(approximate_policy(10), pointcloud_sub, limage_sub, rimage_sub);
    //sync.registerCallback(boost::bind(&callback, _1, _2, _3));

    ros::spin();

    return 0;    
}
