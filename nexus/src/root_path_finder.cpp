#include "nexus/root_path_finder.h"

GridMap root_grid_map_data;

root_path_finder::root_path_finder()
{ 
  goal_setted = false;
  init_setted = false;
  init_pose_found = false;
  goal_pose_found = false;
  init_pose_sub = r_nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",10,&root_path_finder::init_pose_cb,this);
  goal_pose_sub = r_nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal",10,&root_path_finder::goal_pose_cb,this);
  root_grid_map_sub = r_nh.subscribe("grid_map",100,&root_path_finder::grid_map_cb,this);
  plan_pose_pub = r_nh.advertise<planner_msgs::Mapbuilder>("plan_pose_recv",10);
  plan_cmd_sub = r_nh.subscribe<planner_msgs::Mapbuilder>("/map_builder_cmd",100,&root_path_finder::root_path_finder_cmd,this);

  spline_plan_x_pub = r_nh.advertise<std_msgs::Float32MultiArray>("spline_path_x",10,this);
  spline_plan_y_pub = r_nh.advertise<std_msgs::Float32MultiArray>("spline_path_y",10,this);;

  pub_root = r_nh.advertise<nav_msgs::Path>("path_root",1,true);
  pub_splined_root = r_nh.advertise<nav_msgs::Path>("splined_path_root",1,true);
}
void root_path_finder::init_pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &i_pose)
{
  init_pose_stamped = *i_pose;
  planner_msgs::Mapbuilder msg;
  msg.id = 0;
  msg.state = "init_pose";
  msg.val1 = init_pose_stamped.pose.pose.position.x;
  msg.val2 = init_pose_stamped.pose.pose.position.y;
  msg.val3 = init_pose_stamped.pose.pose.orientation.z;
  plan_pose_pub.publish(msg);
  init_setted = true;
}
void root_path_finder::goal_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &g_pose)
{
  goal_pose_stamped = *g_pose;
  planner_msgs::Mapbuilder msg;
  msg.id = 1;
  msg.state = "goal_pose";
  msg.val1 = goal_pose_stamped.pose.position.x;
  msg.val2 = goal_pose_stamped.pose.position.y;
  msg.val3 = goal_pose_stamped.pose.orientation.z;
  plan_pose_pub.publish(msg);
  goal_setted = true;
}
void root_path_finder::grid_map_cb(const grid_map_msgs::GridMap& map_input)
{
  root_grid_map_data.clearAll();
  grid_map::GridMapRosConverter::fromMessage(map_input,root_grid_map_data);
}
float root_path_finder::pose2grid_map_dist(float pose_x,float pose_y,float map_x,float map_y)
{
  return sqrt( (pose_x-map_x)*(pose_x-map_x) + (pose_y-map_y)*(pose_y-map_y) );
}

bool root_path_finder::convert_pose2grid()
{
  for(GridMapIterator iter(root_grid_map_data);!iter.isPastEnd();++iter)
  {
      Position pos;
      root_grid_map_data.getPosition(*iter,pos);
      float init_dist = pose2grid_map_dist(pos(0),pos(1),init_pose_stamped.pose.pose.position.x,init_pose_stamped.pose.pose.position.y);
      float goal_dist = pose2grid_map_dist(pos(0),pos(1),goal_pose_stamped.pose.position.x,goal_pose_stamped.pose.position.y);
      if(init_dist < 0.05)
      {
          root_grid_map_data.getIndex(pos,init_pose_index);
          init_pose_found = true;
      }
      if(goal_dist < 0.05)
      {
          root_grid_map_data.getIndex(pos,goal_pose_index);
          goal_pose_found = true;
      }
  }
  if(init_pose_found == true && goal_pose_found == true){ return true;}
  else
  {
      ROS_INFO("Pose Failed == init_pose : %d / goal_pose : %d",init_pose_found,goal_pose_found);
      return false;
  }
}

void root_path_finder::root_path_plan()
{
  if(init_setted == true && goal_setted == true)
  {
      Dstar *dstar = new Dstar();
      dstar->init(0,0,1,1);
      list<state> root_path;
      for(GridMapIterator iter(root_grid_map_data);!iter.isPastEnd();++iter)
      {
          Position pos;
          root_grid_map_data.getPosition(*iter,pos);
          Index idx;
          root_grid_map_data.getIndex(pos,idx);
          if(root_grid_map_data.isValid(*iter,"sub_elevation"))
          {
              dstar->updateCell(idx(0),idx(1),1);
          }
          else
          {
              dstar->updateCell(idx(0),idx(1),-1);
          }
      }
      if(convert_pose2grid() == true)
      {
          geometry_msgs::PoseStamped plan_root_pose;
          dstar->updateStart(init_pose_index(0),init_pose_index(1));
          dstar->updateGoal(goal_pose_index(0),goal_pose_index(1));
          dstar->replan();
          root_path = dstar->getPath();
          plan_result.clear();
          plan_root_pose.header.frame_id = "base_link";
          plan_root_pose.pose.position.z = 0.1;
          plan_root_pose.pose.orientation.x = 0.0;
          plan_root_pose.pose.orientation.y = 0.0;
          plan_root_pose.pose.orientation.z = 0.1;
          plan_root_pose.pose.orientation.w = 0.0;

          list<state>::iterator itor;
          itor=root_path.begin();

          root_index.clear();
          int spline_control_point = 0;
          for(itor=root_path.begin(); itor != root_path.end(); itor++)
            {
              Index idx;
              state temp = *itor;
              idx(0) = temp.x;
              idx(1) = temp.y;
              root_index.push_back(idx);
              Position pos;
              root_grid_map_data.getPosition(idx,pos);              
              plan_root_pose.pose.position.x = pos(0);
              plan_root_pose.pose.position.y = pos(1);
              plan_result.push_back(plan_root_pose);
              spline_control_point++;
            }
          path_plan_line_draw.poses.clear();
          path_plan_line_draw.poses.resize(plan_result.size());
          for(size_t i=0; i<plan_result.size();i++){
          path_plan_line_draw.poses.push_back(plan_result.at(i));
          }
          path_plan_line_draw.header.frame_id = "base_link";
          path_plan_line_draw.header.stamp = ros::Time::now();
          pub_root.publish(path_plan_line_draw);



      }
      delete dstar;
  }
  else
  {
    ROS_INFO("Fail: pose is not setted");
  }
}
void root_path_finder::spline_root_path(int deg, int node_num, float resolution)
{

  Index spline_plan_idx;
  Position spline_grid_pos;
  int spline_deg = deg;
  int spline_dimension = 2;

  int spline_control_point = root_index.size();

  int divide_control_point = spline_control_point/node_num;

  ts::BSpline spline(spline_deg,spline_dimension,divide_control_point,TS_CLAMPED);
  std::vector<float> ctrlp = spline.ctrlp();
  spline_plan_result.clear();
  for(int i=0;i<divide_control_point;i++){

      if(i==(divide_control_point-1))
        {
          spline_plan_idx = root_index.back();

        }
      else
        {
          spline_plan_idx = root_index.at(node_num*i);
        }

      //root_grid_map_data.at("sub_elevation",spline_plan_idx) = 255;
      root_grid_map_data.getPosition(spline_plan_idx,spline_grid_pos);

      ctrlp[2*i] = (float)spline_grid_pos(0);
      ctrlp[1+2*i] = (float)spline_grid_pos(1);

    }

  std_msgs::Float32MultiArray x_path;
  std_msgs::Float32MultiArray y_path;
  x_path.data.clear();
  y_path.data.clear();

  spline.setCtrlp(ctrlp);
  std::vector<float> spline_result;
  float spline_resolution = 0.0f;
  spline_pose.header.frame_id="base_link";
  float spline_res  = 1/resolution;
    for(int i=0;i<resolution;i++)
    {
      spline_result = spline.evaluate(spline_resolution).result();
      spline_resolution = i*spline_res;
      spline_pose.pose.position.z = 0.0;
      spline_pose.pose.orientation.w = 1.0;
      spline_pose.pose.orientation.x = 0.0;
      spline_pose.pose.orientation.y = 0.0;
      spline_pose.pose.orientation.z = 0.1;
      spline_pose.pose.position.x = (double)spline_result[0];
      spline_pose.pose.position.y = (double)spline_result[1];
      x_path.data.push_back(spline_result[0]);
      y_path.data.push_back(spline_result[1]);
      spline_plan_result.push_back(spline_pose);
      spline_result.clear();
    }
     spline_result.clear();
     ctrlp.clear();
      path_plan_spline_draw.poses.clear();
      path_plan_spline_draw.poses.resize(spline_plan_result.size());

    for(int i=0;i<spline_plan_result.size();i++)
    {
          path_plan_spline_draw.poses.push_back(spline_plan_result.at(i));
    }
    path_plan_spline_draw.header.frame_id = "base_link";
    path_plan_spline_draw.header.stamp = ros::Time::now();
    pub_splined_root.publish(path_plan_spline_draw);

    spline_plan_x_pub.publish(x_path);
    spline_plan_y_pub.publish(y_path);
}

void root_path_finder::root_path_finder_cmd(const planner_msgs::Mapbuilder::ConstPtr &cmd)
{
  if(cmd->state == "plan_root")
  {
    ROS_INFO("%s",cmd->state.c_str());
    root_path_plan();

  }
  else if(cmd->state == "spline_root")
  {
    ROS_INFO("%s",cmd->state.c_str());
    spline_root_path((int)cmd->val1,(int)cmd->val2,cmd->val3);
  }

}
