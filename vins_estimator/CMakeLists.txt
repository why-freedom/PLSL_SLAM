cmake_minimum_required(VERSION 2.8.3)
project(plsl)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    camera_models
    image_transport)

find_package(OpenCV 3 REQUIRED)
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})
# message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

find_package(Ceres REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package()

add_library(vins_lib
    src/estimator/parameters.cpp
    src/estimator/estimator.cpp
    src/estimator/feature_manager.cpp
    src/factor/pose_local_parameterization.cpp
    src/factor/projectionTwoFrameOneCamFactor.cpp
    src/factor/projectionTwoFrameTwoCamFactor.cpp
    src/factor/projectionOneFrameTwoCamFactor.cpp
    src/factor/marginalization_factor.cpp
    src/utility/utility.cpp
    src/utility/visualization.cpp
    src/utility/CameraPoseVisualization.cpp
    src/initial/solve_5pts.cpp
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp
    src/featureTracker/feature_tracker.cpp
    src/factor/projection_factor.cpp

    # why 线特征部分
    src/sophus/common.hpp
    src/sophus/types.hpp
    src/sophus/so3.hpp
    src/utility/line_geometry.h
    src/utility/line_geometry.cpp
    src/factor/line_parameterization.h
    src/factor/line_parameterization.cpp
    src/factor/line_projection_factor.h
    src/factor/line_projection_factor.cpp
    src/featureTracker/linefeature_tracker.cpp

    # why 激光雷达点云特征部分
    src/featureTracker/lidar_feature_tracker.cpp

    )

target_link_libraries(vins_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} ${PCL_LIBRARIES})

add_executable(plsl_slam src/rosNodeTest.cpp)
target_link_libraries(plsl_slam vins_lib) 


add_executable(scanRegistration src/scanRegistration.cpp)
target_link_libraries(scanRegistration vins_lib)


add_executable(laserMapping src/laserMapping.cpp)
target_link_libraries(laserMapping vins_lib)

# add_executable(kitti_odom_test src/KITTIOdomTest.cpp)
# target_link_libraries(kitti_odom_test vins_lib) 

# add_executable(kitti_gps_test src/KITTIGPSTest.cpp)
# target_link_libraries(kitti_gps_test vins_lib) 

