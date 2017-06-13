#pragma once

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
/*
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
*/


#include <boost/thread/recursive_mutex.hpp>
#include <boost/bind.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <string>
#include <vector>
#include <cmath>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_cv/grid_map_cv.hpp>

#include <cv_bridge/cv_bridge.h>
#include "cv.h"
#include "highgui.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv2/video/background_segm.hpp>
#include <cxcore.h>
#include <highgui.h>

#include <eigen3/Eigen/Dense>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

#include "planner_msgs/Mapbuilder.h"

#define PI 3.14159265359
#define deg2rad(deg)  ((deg) * PI / 180.0)
#define rad2deg(rad)  ((rad) * 180.0 / PI)

using namespace std;
using namespace grid_map;
using namespace cv;
