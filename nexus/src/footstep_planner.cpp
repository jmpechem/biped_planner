#include "nexus/footstep_planner.h"

GridMap footstep_grid_map_data;

footstep_planner::footstep_planner()
{ 
 foot_box_pub = f_nh.advertise<visualization_msgs::MarkerArray>("foot_step_marker",1);
 ref_path_x_sub = f_nh.subscribe<std_msgs::Float32MultiArray>("spline_path_x",1,&footstep_planner::ref_path_x_cb,this);
 ref_path_y_sub = f_nh.subscribe<std_msgs::Float32MultiArray>("spline_path_y",1,&footstep_planner::ref_path_y_cb,this);

 init_yaw_sub = f_nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",10,&footstep_planner::init_yaw_cb,this);
 goal_yaw_sub = f_nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal",10,&footstep_planner::goal_yaw_cb,this);
 footstep_cmd_sub = f_nh.subscribe<planner_msgs::Mapbuilder>("/map_builder_cmd",100,&footstep_planner::footstep_planner_cmd,this);
 path_x.clear();
 path_y.clear();
 foot_markers.markers.clear();
}
void footstep_planner::init_yaw_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &i_pose)
{
  qi.setX(i_pose->pose.pose.orientation.x);
  qi.setY(i_pose->pose.pose.orientation.y);
  qi.setZ(i_pose->pose.pose.orientation.z);
  qi.setW(i_pose->pose.pose.orientation.w);
}
void footstep_planner::goal_yaw_cb(const geometry_msgs::PoseStamped::ConstPtr &g_pose)
{
  qg.setX(g_pose->pose.orientation.x);
  qg.setY(g_pose->pose.orientation.y);
  qg.setZ(g_pose->pose.orientation.z);
  qg.setW(g_pose->pose.orientation.w);
}
void footstep_planner::ref_path_x_cb(const std_msgs::Float32MultiArray::ConstPtr &array)
{
  int i = 0;
  path_x.clear();
  for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
  {
     path_x.push_back(*it);
  }

  return;
}

void footstep_planner::ref_path_y_cb(const std_msgs::Float32MultiArray::ConstPtr &array)
{
  int i = 0;
  path_y.clear();
  for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
  {
     path_y.push_back(*it);
  }
  return;
}

void footstep_planner::foot_draw(float x,float y, tf::Quaternion q_input,int num,bool isleft)
{
  uint32_t shape = visualization_msgs::Marker::CUBE;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "footstep_list";
  marker.id = num;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = q_input.getX();
  marker.pose.orientation.y = q_input.getY();
  marker.pose.orientation.z = q_input.getZ();
  marker.pose.orientation.w = q_input.getW();
  marker.scale.x = 0.28;
  marker.scale.y = 0.1;
  marker.scale.z = 0.03;
  if(isleft)
  {
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.5;
  }
  else
  {
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.5;
  }
  marker.lifetime = ros::Duration();

  foot_markers.markers.push_back(marker);
  //foot_box_pub.publish(marker);

}
void footstep_planner::create_footpose()
{
  float d= 0.1;

}
int footstep_planner::find_one_step_max_node(int start_node,float max_length)
{
  int max_node;
  double dist = 0.0f;
  for(int i = start_node; i<path_x.size();i++)
  {
   dist = sqrt( (path_x.at(i) - path_x.at(start_node)) * (path_x.at(i) - path_x.at(start_node)) + (path_y.at(i) - path_y.at(start_node)) * (path_y.at(i) - path_y.at(start_node)) );
   if( abs(dist - max_length) <= 0.005)
   {
       max_node = i;
       return max_node;
   }
   else
   {
       return 0;
   }
  }

}

void footstep_planner::creat_footstep()
{
  //foot_markers.markers.clear();
  if(path_x.size() == 0 || path_y.size() == 0){ }
  else
  {
  float d = 0.1; // root to foot center distance
  float step_max_length = 0.1; // one step foot maximum length

  tf::Matrix3x3 mi(qi);
  double ri, pi, yi;
  mi.getRPY(ri, pi, yi);

  double xi_shift = -sin(yi)*d;
  double yi_shift = cos(yi)*d;

  foot_draw(path_x.at(0)+xi_shift,path_y.at(0)+yi_shift,qi,0,true);
  foot_draw(path_x.at(0)-xi_shift,path_y.at(0)-yi_shift,qi,1,false);


  int cur_max_index;
  cur_max_index = find_one_step_max_node(0,step_max_length);



  tf::Matrix3x3 mg(qg);
  double rg, pg, yg;
  mg.getRPY(rg, pg, yg);
  int x_end = path_x.size()-1;
  int y_end = path_y.size()-1;
  double xg_shift = -sin(yg)*d;
  double yg_shift = cos(yg)*d;

  foot_draw(path_x.at(x_end)+xg_shift,path_y.at(y_end)+yg_shift,qg,2,true);
  foot_draw(path_x.at(x_end)-xg_shift,path_y.at(y_end)-yg_shift,qg,3,false);


  foot_box_pub.publish(foot_markers);

  }
}

void footstep_planner::curvature_cal()
{
  /*
  std::vector< cv::Point2f > vecCurvature( vecContourPoints.size() );

  cv::Point2f posOld, posOlder;
  cv::Point2f f1stDerivative; f2ndDerivative;
  for (size_t i = 0; i < vecContourPoints.size(); i++ )
  {
      const cv::Point2f& pos = vecContourPoints[i];

      if ( i == 0 ){ posOld = posOlder = pos; }

      f1stDerivative.x =   pos.x -        posOld.x;
      f1stDerivative.y =   pos.y -        posOld.y;
      f2ndDerivative.x = - pos.x + 2.0f * posOld.x - posOlder.x;
      f2ndDerivative.y = - pos.y + 2.0f * posOld.y - posOlder.y;

      float curvature2D = 0.0f;
      if ( std::abs(f2ndDerivative.x) > 10e-4 && std::abs(f2ndDerivative.y) > 10e-4 )
      {
          curvature2D = sqrt( std::abs(pow( f2ndDerivative.y*f1stDerivative.x - f2ndDerivative.x*f1stDerivative.y, 2.0f ) / pow( f2ndDerivative.x + f2ndDerivative.y, 3.0 ) ) );
      }

      vecCurvature[i] = curvature2D;

      posOlder = posOld;
      posOld = pos;
  }*/
}
void footstep_planner::footstep_planner_cmd(const planner_msgs::Mapbuilder::ConstPtr &cmd)
{
  if(cmd->state == "footstep_plan")
  {
    ROS_INFO("%s",cmd->state.c_str());
    creat_footstep();

  }
  else if(cmd->state == "t1")
  {
    ROS_INFO("%s",cmd->state.c_str());

  }
  else if(cmd->state == "t2")
  {
    ROS_INFO("%s",cmd->state.c_str());

  }
}
