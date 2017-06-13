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
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>


using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr plane_clouds (new pcl::PointCloud<pcl::PointXYZ>);
ros::Publisher segmented_cloud_pub;


void seg_cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::fromROSMsg(*msg,*plane_clouds);
  /*sensor_msgs::PointCloud2::Ptr output_cloud(new sensor_msgs::PointCloud2);
  pcl::toROSMsg (*plane_clouds, *output_cloud);
  output_cloud->header.frame_id = "base_link";
  segmented_cloud_pub.publish(output_cloud);*/

/*


*/
}
void plane_segmentation(int number_of_plane)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointXYZ temp;
  for(size_t i=0;i<plane_clouds->size();i++)
  {
      temp.x = plane_clouds->points[i].x;
      temp.y = plane_clouds->points[i].y;
      temp.z = plane_clouds->points[i].z;
      cloud.points.push_back(temp);
  }
  cloud.width = plane_clouds->size();
  cloud.height = 1;
  cloud.is_dense = true;
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.02);

    pcl::PointCloud<pcl::PointXYZI> TotalCloud;
    sensor_msgs::PointCloud2 output;
    for(int i = 0; i < number_of_plane; i++)
    {
        if(cloud.size() < 100)
               break;

        seg.setInputCloud (cloud.makeShared());

        pcl::ModelCoefficients coefficients;
        // pcl::PointIndices inliers;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        seg.segment (*inliers, coefficients);

        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        // Extract the inliers
        ROS_INFO("extract the inliers");
        pcl::PointCloud<pcl::PointXYZ> in_cloud;
        extract.setInputCloud (cloud.makeShared());
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (in_cloud);
        extract.setNegative (true);
        extract.filter (cloud);

// in_cloud -> segmented points by using this we can detect supportable area and edge
        pcl::PointCloud<pcl::PointXYZ>::iterator index = in_cloud.begin();
        for(; index != in_cloud.end(); index++)
        {
           pcl::PointXYZ pt = *index;
           pcl::PointXYZI pt2;
           pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
           pt2.intensity = (float)(i + 1);

           TotalCloud.push_back(pt2);
        }

        ROS_INFO("%d. remained point cloud = %d", i, (int)cloud.size());
    }

      // Convert To ROS data type
    pcl::PCLPointCloud2 cloud_p;
    pcl::toPCLPointCloud2(TotalCloud, cloud_p);
    pcl_conversions::fromPCL(cloud_p, output);
    output.header.frame_id = "base_link";
    segmented_cloud_pub.publish(output);

    ROS_INFO("published it.");


}

void map_builder_cb(const planner_msgs::Mapbuilder::ConstPtr &cmd)
{

  if(cmd->state == "step_detect")
  {
   ROS_INFO("%s",cmd->state.c_str());
   plane_segmentation(cmd->id);

  }
  else if(cmd->state == "save_stl")
  {
      ROS_INFO("%s",cmd->state.c_str());
      pcl::io::savePCDFile("/home/jimin/catkin_ws/src/plane_seg/result.pcd", *plane_clouds,true);
  }

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "plane_seg");
  ros::NodeHandle nh;

  ros::Subscriber map_builder_cmd_sub = nh.subscribe<planner_msgs::Mapbuilder>("map_builder_cmd",100,map_builder_cb);
  ros::Subscriber hcut_point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("plane_seg",1,seg_cloud_cb);
  segmented_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("plane_seg_result",1);
  ros::Rate rate(10.0);


  while(nh.ok())
    {

      ros::spinOnce();
    }
}


