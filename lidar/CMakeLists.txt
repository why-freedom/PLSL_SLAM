cmake_minimum_required(VERSION 2.8.3)
project(lidar_estimator)

set(CMAKE_BUILD_TYPE "Release")
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
    )

find_package(OpenCV 3 REQUIRED)

# message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

# why
find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#

find_package(Ceres REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package()

add_executable(lidar_estimator registerPointCloud_node.cpp   #why
                )
target_link_libraries(lidar_estimator ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} ${PCL_LIBRARIES}) 


add_executable(processDepthmap processDepthmap.cpp)
target_link_libraries(processDepthmap ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} ${PCL_LIBRARIES}) 
