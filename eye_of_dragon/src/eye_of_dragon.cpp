#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
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
#include <std_msgs/Empty.h>
#include <pcl/filters/passthrough.h>

#include "planner_msgs/Mapbuilder.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr recv_clouds (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr send_clouds (new pcl::PointCloud<pcl::PointXYZ>);

class EyeOfDragon
{
public:
    EyeOfDragon()
    {
        shoot_once = false;
        run_assembly = false;
        assembly_pt_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/assemble_cloud",10);
        reset_pt_ = nh_.subscribe<planner_msgs::Mapbuilder>("/reset_pt",10,&EyeOfDragon::visual_sensor_cmd,this);
        pt_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/self_filtered_cloud",10,&EyeOfDragon::robot_point_cloud_cb,this);
    }
    void visual_sensor_cmd(const planner_msgs::Mapbuilder::ConstPtr &cmd)
    {
        if(cmd->state == "reset_assemble") // reset data
        {
            recv_clouds->clear();
            send_clouds->clear();
        }
        else if(cmd->state == "disable_assemble") // stop receving data
        {
            run_assembly = false;
            shoot_once = true;
        }
        else if(cmd->state == "run_assemble") // restart receving data
        {
            run_assembly = true;
        }
    }
    void robot_point_cloud_cb(const sensor_msgs::PointCloud2::ConstPtr &cloud)
    {
        if(run_assembly == true)
        {
            recv_clouds->clear();
            pcl::fromROSMsg(*cloud,*recv_clouds);
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_clouds (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(recv_clouds);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(-1.9,0.1);
            pass.filter(*filtered_clouds);
            for(size_t i=0;i<filtered_clouds->size();i++)
            {
                send_clouds->push_back(filtered_clouds->at(i));
            }

            sensor_msgs::PointCloud2::Ptr output_cloud(new sensor_msgs::PointCloud2);            
            pcl::toROSMsg (*send_clouds, *output_cloud);
            output_cloud->header.frame_id = "base_link";
            assembly_pt_pub_.publish(output_cloud);
        }
        if(shoot_once == true)
        {
            sensor_msgs::PointCloud2::Ptr output_cloud(new sensor_msgs::PointCloud2);
            pcl::toROSMsg (*send_clouds, *output_cloud);
            output_cloud->header.frame_id = "base_link";
            assembly_pt_pub_.publish(output_cloud);
            shoot_once = false;
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher  assembly_pt_pub_;
    ros::Subscriber reset_pt_;
    ros::Subscriber pt_sub_;
    bool            run_assembly;
    bool            shoot_once;
};


int main(int argc, char **argv)
{
   ros::init(argc,argv,"eye_of_dragon");
   EyeOfDragon hanzo;   
   ros::spin();
}
