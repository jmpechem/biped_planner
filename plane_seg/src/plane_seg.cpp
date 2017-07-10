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
#include <pcl/features/normal_3d.h>
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
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_hull.h>
#include "planner_msgs/Mapbuilder.h"

#define PI 3.14159265359
#define deg2rad(deg)  ((deg) * PI / 180.0)
#define rad2deg(rad)  ((rad) * 180.0 / PI)
using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr plane_clouds (new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr normal_clouds (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZI>::Ptr dist_seg_clouds (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZ>::Ptr convex_clouds (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZI>::Ptr final_seg_clouds (new pcl::PointCloud<pcl::PointXYZI>);
ros::Publisher segmented_cloud_pub;
ros::Publisher segment_plane_pub;
ros::Publisher segment_dist_pub;
int dist_seg_num = 0;
int conv_seg_num = 0;

void seg_cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  plane_clouds->clear();
  pcl::fromROSMsg(*msg,*plane_clouds);  
}
void seg_normal_cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  normal_clouds->clear();
  pcl::fromROSMsg(*msg,*normal_clouds);
}
// using box size mask which can support area and foot area
bool box_check_area(pcl::PointCloud<pcl::PointXYZ> segmented_cloud,float grid_size,int masksize)
{
  if(masksize%2 == 0)
  {
      cout << "mask size should be odd number!" << endl;
      masksize = masksize+1;
  }
  int half_masksize = masksize/2;

  pcl::PointXYZ center_pos;
  pcl::PointXY check_pos;
  pcl::PointCloud<pcl::PointXY> mask_pos;

  //cout << "segment size : " << segmented_cloud.size() << " , half_masksize : " << half_masksize<< endl;

  if(segmented_cloud.size() > masksize*masksize)
  {
      int sub_seg_num = 0;
      for(size_t i=0;i<segmented_cloud.size();i++)
      {
          mask_pos.clear();
          center_pos = segmented_cloud.at(i);
          // make mask from center pos
          for(int j=-half_masksize;j<half_masksize+1;j++)
          {
              check_pos.x = center_pos.x + grid_size*j;
              for(int k=-half_masksize;k<half_masksize+1;k++)
              {
                  check_pos.y = center_pos.y + grid_size*k;
                  mask_pos.push_back(check_pos);
              }
          }
      //
      //
      // this part need to modified ... checking part!!!
      //

          int occupied = 0;
          for(int m=0;m<mask_pos.size();m++)
          {

              for(size_t l=0;l<segmented_cloud.size();l++)
              {
                  if(std::abs(segmented_cloud.at(l).x - mask_pos.at(m).x) < 0.001 && std::abs(segmented_cloud.at(l).y - mask_pos.at(m).y) < 0.001)
                  {
                    occupied++;
                    break;
                  }
              }

          }
          if(occupied == masksize*masksize)
          {
              sub_seg_num++;
          }
          if(sub_seg_num > 4)
          {
              break;
          }

      }
      //cout << "foothold area : " << sub_seg_num << endl;
      if(sub_seg_num > 4)
      {
          return true;
      }
  }
    return false;
}

void distance_clustering(pcl::PointCloud<pcl::PointXYZ> preseg_cloud,float grid_size)
{

  float dist_threshold = grid_size*sqrt(2);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (preseg_cloud.makeShared());
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (dist_threshold);
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (preseg_cloud.makeShared());
  ec.extract (cluster_indices);
  //cout << "ec seg num : " << cluster_indices.size() << endl;

  double r = sqrt(0.1*0.1 + 0.09*0.09);
  double foot_area = r*r*PI;
  double body_area = 0.5*0.5*PI;

  pcl::PointCloud<pcl::PointXYZ> ec_seg_cloud;
  pcl::PointXYZ max_pt, min_pt;

  pcl::ConvexHull<pcl::PointXYZ> hull;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
      ec_seg_cloud.clear();
      convex_hull->clear();
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      {
        pcl::PointXYZ pt = preseg_cloud.points[*pit];
        pcl::PointXYZI pt2;
        pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
        pt2.intensity = (float)(dist_seg_num + 1);
        ec_seg_cloud.push_back(pt);
        dist_seg_clouds->push_back(pt2);
      }

      if(box_check_area(ec_seg_cloud,grid_size,5))
      {
          for(int t=0; t<ec_seg_cloud.size();t++)
          {
            pcl::PointXYZ pt3 = ec_seg_cloud.points[t];
            pcl::PointXYZI pt4;
            pt4.x = pt3.x, pt4.y = pt3.y, pt4.z = pt3.z;
            pt4.intensity = (float)(conv_seg_num+1);
            final_seg_clouds->push_back(pt4);
          }
          conv_seg_num++;
      }  
      dist_seg_num++;

  }
}

void plane_segmentation(int number_of_plane,float dist_threshold,float grid_size,float slope_threshold)
{
  dist_seg_clouds->clear();
  final_seg_clouds->clear();
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
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (dist_threshold);
    const Eigen::Vector3f z(0,0,1);
    seg.setAxis(z);
    double threshold_angle = deg2rad(slope_threshold);
    seg.setEpsAngle(threshold_angle);


    pcl::PointCloud<pcl::PointXYZI> TotalCloud;
    sensor_msgs::PointCloud2 output;
    int seg_plane_num = 0;
    for(int i = 0; i < number_of_plane; i++)
    {
        if(cloud.size() < 100)
               break;

        seg.setInputCloud (cloud.makeShared());
        //seg.setInputNormals (normal.makeShared());
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

        pcl::PointCloud<pcl::PointXYZ> preseg_cloud;
// in_cloud -> segmented points by using this we can detect supportable area and edge
        pcl::PointCloud<pcl::PointXYZ>::iterator index = in_cloud.begin();
        for(; index != in_cloud.end(); index++)
        {
           pcl::PointXYZ pt = *index;
           pcl::PointXYZI pt2;
           pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
           pt2.intensity = (float)(i + 1);
           preseg_cloud.push_back(pt);
           TotalCloud.push_back(pt2);
        }
        distance_clustering(preseg_cloud,grid_size);

        seg_plane_num++;
        ROS_INFO("%d. remained point cloud = %d", i, (int)cloud.size());
    }
    cout << "supportable plane : " << conv_seg_num  << " plane : " << seg_plane_num << endl;
    int result_plane = conv_seg_num;
    dist_seg_num = 0;
    conv_seg_num = 0;

    pcl::PCLPointCloud2 cloud_p;
    //pcl::toPCLPointCloud2(*dist_seg_clouds, cloud_p);
    pcl::toPCLPointCloud2(*final_seg_clouds, cloud_p);
    pcl_conversions::fromPCL(cloud_p, output);
    output.header.frame_id = "base_link";
    segmented_cloud_pub.publish(output);
      // Convert To ROS data type
    /*
    pcl::PCLPointCloud2 cloud_p;
    pcl::toPCLPointCloud2(TotalCloud, cloud_p);
    pcl_conversions::fromPCL(cloud_p, output);
    output.header.frame_id = "base_link";
    segmented_cloud_pub.publish(output);


    pcl::toPCLPointCloud2(*final_seg_clouds, cloud_p);
    pcl_conversions::fromPCL(cloud_p, output);
    output.header.frame_id = "base_link";
    segment_dist_pub.publish(output);*/


    ROS_INFO("published it.");

    planner_msgs::Mapbuilder plane_info;
    plane_info.state = "seg_plane";
    plane_info.id = result_plane;
    //plane_info.id = seg_plane_num;
    segment_plane_pub.publish(plane_info);
}



void map_builder_cb(const planner_msgs::Mapbuilder::ConstPtr &cmd)
{

  if(cmd->state == "step_detect")
  {
   ROS_INFO("%s",cmd->state.c_str());
   plane_segmentation(cmd->id,cmd->val1,cmd->val2,cmd->val3);

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
  ros::Subscriber normal_point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/plane_seg_normal",1,seg_normal_cloud_cb);

  segmented_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("plane_seg_result",1);  
  segment_plane_pub = nh.advertise<planner_msgs::Mapbuilder>("segment_info_recv",1);
  segment_dist_pub = nh.advertise<sensor_msgs::PointCloud2>("/distance_seg_result",1);
  ros::Rate rate(10.0);


  while(nh.ok())
    {

      ros::spinOnce();
    }
}


