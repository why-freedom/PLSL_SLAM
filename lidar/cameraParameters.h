#ifndef camera_parameters_h
#define camera_parameters_h

#include <opencv/cv.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/features2d/features2d.hpp>
// #include "opencv2/contrib/contrib.hpp"
// #include "opencv2/calib3d/calib3d.hpp"

// #include <cv_bridge/cv_bridge.h>

// const int imageWidth = 744;
// const int imageHeight = 480;

// double kImage[9] = {4.177343016733e+002, 0, 3.715643918956e+002, 
//                     0, 4.177970397634e+002, 1.960688121183e+002, 
//                     0, 0, 1};

// double dImage[4] = {-3.396867101163e-001, 1.309347902588e-001, -2.346791258754e-004, 2.209387016957e-004};


// why:KITTI - 04.bag
// const int imageWidth = 1241;
// const int imageHeight = 376;
// double kImage[9] = {7.070911865234e+002, 0, 6.018873291015e+002, 
//                     0, 7.070911865234e+002, 1.831103973388e+002, 
//                     0, 0, 1};
// double dImage[4] = {0, 0, 0, 0};


// why:mynteye.bag
// const int imageWidth = 640;
// const int imageHeight = 480;

// double kImage[9] = {3.5932730102539062500e+02, 0, 3.2712988281250000000e+02, 
//                     0, 3.5972198486328125000e+02, 2.3601489257812500000e+02, 
//                     0, 0, 1};

// double dImage[4] = {-2.9687118530273438e-01, 7.6961517333984380e-02, 9.1552734375000000e-04, -2.0980834960938000e-04};

// 1280 * 720
const int imageWidth = 1280;
const int imageHeight = 720;

double kImage[9] = {713.6898193359375, 0, 654.5809936523437, 
                    0, 714.6583862304687, 352.4136047363281, 
                    0, 0, 1};

double dImage[4] = {-0.30114364624023438, 0.08311080932617188, 0.00044631958007812, -0.00022125244140625};



#endif

//   // why lidarPoint to camera
//   Eigen::Matrix4f transform;
//   std::cout << "transform is start" << std::endl; // why for debug 


//   transform  << -1.2866589987073040e-01, -9.9935332890541950e-01, -3.1609074533867410e-02, 0.048331542,
//                -1.2817374489219518e-01, 3.3549046227023371e-02, -9.9118411640702120e-01, -0.09191652,
//                 9.9160360059226360e-01, -1.2937754940996415e-02, -1.7140315422086605e-02, -0.07415591,
//                 0, 0, 0, 1;

//   // transform <<  -1.2866589987073040e-01, -1.2937754940996415e-02, 9.9160360059226360e-01, 7.8562691695536410e-02,
//   //               -9.9118411640702120e-01, 3.3549046227023371e-02, -1.2817374489219518e-01, 4.1484322701292688e-02,
//   //               -3.1609074533867410e-02, -9.9935332890541950e-01, -1.7140315422086605e-02, -9.1600419087036714e-02, 
//   //                0., 0., 0., 1. ;

//   // transform = transform.inverse().eval();
//   pcl::transformPointCloud(*syncCloud, *syncCloud, transform);
//   std::cout << "transform is done" << std::endl; // why for debug 