#include "def_include.h"

class map_builder{
  public:
  map_builder();

  void map_builder_cmd(const planner_msgs::Mapbuilder::ConstPtr &cmd);
  void load_from_pcd();
  void load_from_online(const sensor_msgs::PointCloud2::ConstPtr &input);
  void height_cut(float robot_height);
  void processed_cloud_cb(const sensor_msgs::PointCloud2::ConstPtr &input);
  void grid_map_converting(float grid_resolution);


  private:
  ros::NodeHandle m_nh;
  ros::Subscriber map_builder_cmd_sub;
  ros::Subscriber online_cloud_sub;
  ros::Subscriber processed_cloud_sub;

  ros::Publisher raw_cloud_pub;
  ros::Publisher hcut_cloud_pub;
  ros::Publisher grid_map_pub;

  boost::recursive_mutex    MapMutex_;
};
