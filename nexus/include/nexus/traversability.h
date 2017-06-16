#include "def_include.h"

class traversability{
  public:
  traversability();
  void traversability_cmd(const planner_msgs::Mapbuilder::ConstPtr &cmd);
  void grid_map_cb(const grid_map_msgs::GridMap& map_input);
  void segmented_grid_map_cb(const sensor_msgs::PointCloud2::ConstPtr &clouds);
  void normal_extraction(float radius);
  void obstacle_finder(float slope, float rel_height);
  void obstacle_potential_field(float pf_radius,float grid_resolution);

  float plane_dist_cal(float center_x, float center_y, float target_x, float target_y);

  private:
  ros::NodeHandle t_nh;
  ros::Subscriber grid_map_sub;
  ros::Subscriber traversability_cmd_sub;
  ros::Subscriber segmented_grid_data_sub;
  ros::Publisher grid_map_data_pub;
  //ros::Publisher grid_map_plane_seg_pub;

  bool isfirstcb;
  float search_radius;
  float big_radius;
  float reference_height;

  boost::recursive_mutex    TraversabilityMutex_;
};
