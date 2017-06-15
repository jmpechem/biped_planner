#include "def_include.h"
#include "nexus/Dstar.h"
#include "nexus/tinysplinecpp.h"

class root_path_finder{
  public:
  root_path_finder();
  void root_path_finder_cmd(const planner_msgs::Mapbuilder::ConstPtr &cmd);
  void init_pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &i_pose);
  void goal_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &g_pose);
  void grid_map_cb(const grid_map_msgs::GridMap& map_input);
  void root_path_plan();  
  bool convert_pose2grid();
  float pose2grid_map_dist(float pose_x,float pose_y,float map_x,float map_y);
  void spline_root_path(int deg, int node_num, float resolution);

  private:
  ros::NodeHandle r_nh;
  ros::Subscriber init_pose_sub;
  ros::Subscriber goal_pose_sub;
  ros::Subscriber root_grid_map_sub;
  ros::Subscriber plan_cmd_sub;
  ros::Publisher  plan_pose_pub;
  ros::Publisher  spline_plan_x_pub;
  ros::Publisher  spline_plan_y_pub;
  ros::Publisher  plan_line_pub;
  ros::Publisher  spline_plan_line_pub;


  bool goal_setted;
  bool init_setted;
  bool init_pose_found;
  bool goal_pose_found;
  Index init_pose_index;
  Index goal_pose_index;
  std::vector<geometry_msgs::PoseStamped> plan_result;
  std::vector<geometry_msgs::PoseStamped> spline_plan_result;
  nav_msgs::Path path_plan_line_draw;
  nav_msgs::Path path_plan_spline_draw;
  ros::Publisher pub_root;
  ros::Publisher pub_splined_root;

  vector<Index> root_index;

  geometry_msgs::PoseWithCovarianceStamped init_pose_stamped;
  geometry_msgs::PoseStamped goal_pose_stamped;
  geometry_msgs::PoseStamped spline_pose;


};
