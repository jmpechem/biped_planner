#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
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
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/bind.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <string>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Int32MultiArray.h>
#include "planner_msgs/Mapbuilder.h"
using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr hcut_clouds (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr processed_clouds(new pcl::PointCloud<pcl::PointXYZ>);
ros::Publisher processed_cloud_pub;


void hcut_point_cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  hcut_clouds->clear();
  /*for(size_t i=0;i<msg->data.size(); i++)
  {
      hcut_clouds->points[i].x = msg->data
  }*/
  pcl::fromROSMsg(*msg,*hcut_clouds);

}

void map_builder_cb(const planner_msgs::Mapbuilder::ConstPtr &cmd)
{

  if(cmd->state == "noise_preprocessing")
  {
   ROS_INFO("%s",cmd->state.c_str());
   float leaf_size = cmd->val1;
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointNormal>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointNormal>);

   pcl::VoxelGrid<pcl::PointXYZ> sor;
   sor.setInputCloud (hcut_clouds);
   sor.setLeafSize (leaf_size, leaf_size, leaf_size);
   sor.filter (*cloud_filtered);


   std::vector<int> mapping;
   pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, mapping);
   pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> filter;
   filter.setInputCloud(cloud_filtered);
   filter.setSearchRadius(leaf_size*2.0);
   filter.setPolynomialFit(true);
   filter.setComputeNormals(true);
   pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
   filter.setSearchMethod(kdtree);
   filter.process(*smoothedCloud);

   processed_clouds->clear();
   processed_clouds->resize(smoothedCloud->points.size());
   for(size_t i=0; i<smoothedCloud->points.size();++i)
   {
         processed_clouds->points[i].x = smoothedCloud->points[i].x;
         processed_clouds->points[i].y = smoothedCloud->points[i].y;
         processed_clouds->points[i].z = smoothedCloud->points[i].z;
   }

  sensor_msgs::PointCloud2::Ptr output_cloud(new sensor_msgs::PointCloud2);
  pcl::toROSMsg (*processed_clouds, *output_cloud);
  output_cloud->header.frame_id = "base_link";
  processed_cloud_pub.publish(output_cloud);
  ROS_INFO("Noise processing success");
  }

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "noise_preprocessing");
  ros::NodeHandle nh;

  ros::Subscriber map_builder_cmd_sub = nh.subscribe<planner_msgs::Mapbuilder>("map_builder_cmd",100,map_builder_cb);
  ros::Subscriber hcut_point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("height_cut_point_cloud",100,hcut_point_cloud_cb);
  processed_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("noise_preprocessed_point_cloud",100);
  ros::Rate rate(10.0);


  while(nh.ok())
    {

      ros::spinOnce();
    }
}


