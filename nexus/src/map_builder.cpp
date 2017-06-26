#include "nexus/map_builder.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr clouds (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr hcut_clouds (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr processed_clouds (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr grid_points(new pcl::PointCloud<pcl::PointXYZ>);

map_builder::map_builder()
{ 
  clouds->clear();
  hcut_clouds->clear();
  processed_clouds->clear();
  grid_points->clear();
  map_builder_cmd_sub = m_nh.subscribe<planner_msgs::Mapbuilder>("map_builder_cmd",100,&map_builder::map_builder_cmd,this);
  online_cloud_sub = m_nh.subscribe<sensor_msgs::PointCloud2>("sync_scan_cloud_filtered",100,&map_builder::load_from_online,this);
  processed_cloud_sub = m_nh.subscribe<sensor_msgs::PointCloud2>("noise_preprocessed_point_cloud",100,&map_builder::processed_cloud_cb,this);
  raw_cloud_pub = m_nh.advertise<sensor_msgs::PointCloud2>("raw_point_cloud",100);
  hcut_cloud_pub = m_nh.advertise<sensor_msgs::PointCloud2>("height_cut_point_cloud",100);
  grid_map_pub = m_nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  isOnline_recv = false;
  grid_map_plane_seg_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/plane_seg",100);
}

void map_builder::processed_cloud_cb(const sensor_msgs::PointCloud2::ConstPtr &input)
{
  processed_clouds->clear();
  pcl::fromROSMsg(*input,*processed_clouds);
}

void map_builder::map_builder_cmd(const planner_msgs::Mapbuilder::ConstPtr &cmd)
{

  if(cmd->state == "load_pcd")
  {
    ROS_INFO("%s",cmd->state.c_str());
    clouds->clear();
    load_from_pcd();

  }
  else if(cmd->state == "load_onlinedata")
  {
    ROS_INFO("%s",cmd->state.c_str());
    isOnline_recv = true;
  }
  else if(cmd->state == "height_cut")
  {
      ROS_INFO("%s",cmd->state.c_str());
      hcut_clouds->clear();
      height_cut(cmd->val1);
  }
  else if(cmd->state == "grid_map")
  {      
      ROS_INFO("%s",cmd->state.c_str());
      grid_points->clear();
      grid_map_converting(cmd->val1);
  }
}

void map_builder::load_from_pcd()
{

  //if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/jimin/catkin_ws/src/jm_global/out_pcd_data_set/out_indo_ds.pcd", *clouds) == -1) //* load the file
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/jimin/catkin_ws/src/jm_global/pcd_data_set/pt4.pcd", *clouds) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read pcd file\n");
    return ;
  }
  else
  {
    ROS_INFO("PCD read success!");
    sensor_msgs::PointCloud2::Ptr output_cloud(new sensor_msgs::PointCloud2);
    pcl::toROSMsg (*clouds, *output_cloud);
    output_cloud->header.frame_id = "base_link";
    raw_cloud_pub.publish(output_cloud);
  }
}

void map_builder::load_from_online(const sensor_msgs::PointCloud2::ConstPtr &input)
{
    if(isOnline_recv)
    {
        clouds->clear();
        pcl::fromROSMsg(*input,*clouds);
        isOnline_recv = false;
    }
}

void map_builder::height_cut(float robot_height)
{
  if(clouds->size() != 0)
  {
      pcl::PointCloud<pcl::PointXYZ>::Ptr assembly (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointXYZ temp;
      for(int i=0;i<clouds->size();i++)
      {
          temp.x = clouds->points[i].x;
          temp.y = clouds->points[i].y;
          temp.z = clouds->points[i].z;
          assembly->push_back(temp);
      }

      pcl::PointXYZ filter_min;
      pcl::PointXYZ filter_max;
      pcl::getMinMax3D(*assembly,filter_min,filter_max);
      pcl::PointCloud<pcl::PointXYZ> final_assembly;
      double z_thres = robot_height;
      cout << "assm size : " << assembly->size() << endl;
      for(int i=0;i<assembly->size();i++)
      {
              if ( abs(filter_min.z-assembly->points[i].z) <= z_thres)
              {
                      temp.x = assembly->points[i].x;
                      temp.y = assembly->points[i].y;
                      temp.z = assembly->points[i].z;
                      final_assembly.push_back(temp);
              }
      }
      pcl::PCLPointCloud2 cloud_p;
      pcl::toPCLPointCloud2(final_assembly, cloud_p);
      sensor_msgs::PointCloud2 output;
      pcl_conversions::fromPCL(cloud_p, output);
      output.header.frame_id = "base_link";
      hcut_cloud_pub.publish(output);

      //hcut_clouds->points[i].x

      ROS_INFO("Height cut success!");
  }
  else
  {
      ROS_INFO("Fail:Empty clouds!");
  }
}

void map_builder::grid_map_converting(float grid_resolution)
{
  if(processed_clouds->size() != 0)
  {
      boost::recursive_mutex::scoped_lock scopedLockmap(MapMutex_);
      GridMap grid_map_({"elevation"});
      float xysize = grid_resolution * 100;
      ROS_INFO("Gridmap : %f %f",xysize,xysize/2);
      grid_map_.setGeometry(Length(xysize,xysize),grid_resolution,Position(xysize/2,0));
      grid_map_.setFrameId("base_link");

      for(unsigned int i=0; i<processed_clouds->size();++i)
      {
        auto& point = processed_clouds->points[i];
        Index index;
        Position position(point.x,point.y);
        if(!grid_map_.getIndex(position, index)) continue;
        auto& elevation = grid_map_.at("elevation",index);
        if(!grid_map_.isValid(index)){ elevation = point.z;continue;}
      }
      // publish gridmap to pointcloud2 form

      for (GridMapIterator it(grid_map_); !it.isPastEnd(); ++it) {
        Position3 pos;
        grid_map_.getPosition3("elevation",*it,pos);
        pcl::PointXYZ pt(pos.x(),pos.y(),pos.z());
        grid_points->points.push_back(pt);
        }
        grid_points->width = grid_points->points.size();
        grid_points->height = 1;
        grid_points->is_dense = true;
        ros::Time time = ros::Time::now();
        grid_map_.setTimestamp(time.toNSec());
        grid_map_msgs::GridMap message;
        GridMapRosConverter::toMessage(grid_map_, message);
        grid_map_pub.publish(message);
        ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
        scopedLockmap.unlock();

        pcl::PointCloud<pcl::PointXYZ> seg_cloud;
        seg_cloud.clear();



        for(GridMapIterator iter(grid_map_);!iter.isPastEnd();++iter)
        {
                Position pos;
                grid_map_.getPosition(*iter,pos);
                Index idx;
                grid_map_.getIndex(pos,idx);
                Position3 pos3;
                grid_map_.getPosition3("elevation",idx,pos3);
                pcl::PointXYZ   pt;
                pt.x = pos3(0);
                pt.y = pos3(1);
                pt.z = pos3(2);
                if(pt.x != 0) // I don't know why x=0 position grid generated....
                seg_cloud.push_back(pt);
        }
        pcl::PCLPointCloud2 cloud_p;
        pcl::toPCLPointCloud2(seg_cloud, cloud_p);
        sensor_msgs::PointCloud2 output;
        pcl_conversions::fromPCL(cloud_p, output);
        output.header.frame_id = "base_link";
        grid_map_plane_seg_pub.publish(output);
  }
  else
  {
    ROS_INFO("Fail:preprocessed cloud is empty!");
  }

}
