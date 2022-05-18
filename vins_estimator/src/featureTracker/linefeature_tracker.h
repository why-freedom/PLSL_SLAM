
#pragma once

#include <cstdio>

#include <iostream>
#include <queue>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "camodocal/camera_models/EquidistantCamera.h"

#include "../estimator/parameters.h"
#include "../utility/tic_toc.h"

#include <opencv2/opencv.hpp>

#include <opencv2/line_descriptor.hpp>
#include <opencv2/features2d.hpp>

using namespace cv::line_descriptor;
using namespace std;
using namespace cv;
using namespace camodocal;
using namespace Eigen;

struct Line
{
	Point2f StartPt;
	Point2f EndPt;
	float lineWidth;
	Point2f Vp;

	Point2f Center;
	Point2f unitDir; // [cos(theta), sin(theta)]
	float length;
	float theta;

	// para_a * x + para_b * y + c = 0
	float para_a;
	float para_b;
	float para_c;

	float image_dx;
	float image_dy;
    float line_grad_avg;

	float xMin;
	float xMax;
	float yMin;
	float yMax;
	unsigned short id;
	int colorIdx;
};

class FrameLines
{
public:
    int frame_id;
    Mat img;
    
    vector<Line> vecLine;
    vector< int > lineID;

    // opencv3 lsd+lbd
    std::vector<KeyLine> keylsd;
    Mat lbd_descr;
};
typedef shared_ptr< FrameLines > FrameLinesPtr;

class LineFeatureTracker
{
  public:
    LineFeatureTracker();

    void readIntrinsicParameter(const string &calib_file);
    void NearbyLineTracking(const vector<Line> forw_lines, const vector<Line> cur_lines, vector<pair<int, int> >& lineMatches);

    vector<Line> undistortedLineEndPoints();
    vector<Line> undistortedLineEndPointsMei(camodocal::CameraPtr cam);

    void readImage(const cv::Mat &_img);
    map<int, vector<pair<int, Eigen::Matrix<double, 4, 1>>>> trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    FrameLinesPtr curframe_, forwframe_;

    cv::Mat undist_map1_, undist_map2_ , K_;

    camodocal::CameraPtr m_camera;       // pinhole camera

    int frame_cnt;
    vector<int> ids;                     // 每个特征点的id
    vector<int> linetrack_cnt;           // 记录某个特征已经跟踪多少帧了，即被多少帧看到了
    int allfeature_cnt;                  // 用来统计整个地图中有了多少条线，它将用来赋值

    double sum_time;
    double mean_time;

};
