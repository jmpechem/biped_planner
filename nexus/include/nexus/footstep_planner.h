#include "def_include.h"


class footstep_planner{
  public:
  footstep_planner();
  void curvature_cal(int start_node, int end_node);
  void ref_path_x_cb(const std_msgs::Float32MultiArray::ConstPtr &array);
  void ref_path_y_cb(const std_msgs::Float32MultiArray::ConstPtr &array);
  void init_yaw_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &i_pose);
  void goal_yaw_cb(const geometry_msgs::PoseStamped::ConstPtr &g_pose);
  void footstep_planner_cmd(const planner_msgs::Mapbuilder::ConstPtr &cmd);
  void creat_footstep(double foot_distance, double onestep);

  double root_path_slope_cal(int start_node, int end_node);
  void foot_draw(double x,double y, tf::Quaternion q_input ,int num,bool isleft);
  void save_offline_footstep();
  int find_one_step_max_node(int start_node,double max_length);
  void init_foot_pose(float init_x,float init_y,double d,tf::Quaternion q_init);
  void final_foot_pose(float goal_x,float goal_y,double final_yaw,bool final_left_start,double d,tf::Quaternion q_final);

  private:
  ros::NodeHandle f_nh;
  ros::Publisher  foot_box_pub;
  ros::Publisher  foot_step_plan_pub;


  ros::Subscriber ref_path_x_sub;
  ros::Subscriber ref_path_y_sub;
  ros::Subscriber init_yaw_sub;
  ros::Subscriber goal_yaw_sub;
  ros::Subscriber footstep_cmd_sub;
  float init_xy[2];
  float goal_xy[2];
  tf::Quaternion qi;
  tf::Quaternion qg;

  std::vector<double> path_x;
  std::vector<double> path_y;
  std::vector<double> node_curvatures;

  visualization_msgs::MarkerArray foot_markers;

  int cnt_of_foot;
  planner_msgs::foot_info_array foot_step_lists;

};
