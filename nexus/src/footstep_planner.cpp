#include "nexus/footstep_planner.h"
#include <fstream>

GridMap footstep_grid_map_data;

typedef struct foot_list{
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;
  int   foot_kind;
}*pfoot_list;

vector<foot_list> plan_foot_lists;

footstep_planner::footstep_planner()
{ 
 foot_box_pub = f_nh.advertise<visualization_msgs::MarkerArray>("foot_step_marker",1);
 foot_step_plan_pub = f_nh.advertise<planner_msgs::foot_info_array>("foot_step_for_walking",1);
 ref_path_x_sub = f_nh.subscribe<std_msgs::Float32MultiArray>("spline_path_x",1,&footstep_planner::ref_path_x_cb,this);
 ref_path_y_sub = f_nh.subscribe<std_msgs::Float32MultiArray>("spline_path_y",1,&footstep_planner::ref_path_y_cb,this);

 init_yaw_sub = f_nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",10,&footstep_planner::init_yaw_cb,this);
 goal_yaw_sub = f_nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal",10,&footstep_planner::goal_yaw_cb,this);
 footstep_cmd_sub = f_nh.subscribe<planner_msgs::Mapbuilder>("/map_builder_cmd",100,&footstep_planner::footstep_planner_cmd,this);

 path_x.clear();
 path_y.clear();
 foot_markers.markers.clear();
 cnt_of_foot = 0;
 plan_foot_lists.clear();
 qi.setX(0.0f);
 qi.setY(0.0f);
 qi.setZ(0.0f);
 qi.setW(1.0f);
 init_xy[0] = 0.0f;
 init_xy[1] = 0.0f;
 goal_xy[0] = 0.0f;
 goal_xy[1] = 0.0f;
 foot_step_lists.foot_steps.clear();
}
void footstep_planner::init_yaw_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &i_pose)
{
  init_xy[0] = i_pose->pose.pose.position.x;
  init_xy[1] = i_pose->pose.pose.position.y;
  qi.setX(i_pose->pose.pose.orientation.x);
  qi.setY(i_pose->pose.pose.orientation.y);
  qi.setZ(i_pose->pose.pose.orientation.z);
  qi.setW(i_pose->pose.pose.orientation.w);
}
void footstep_planner::goal_yaw_cb(const geometry_msgs::PoseStamped::ConstPtr &g_pose)
{
  goal_xy[0] = g_pose->pose.position.x;
  goal_xy[1] = g_pose->pose.position.y;
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

void footstep_planner::foot_draw(double x,double y, tf::Quaternion q_input,int num,bool isleft)
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
  marker.scale.x = 0.3;
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

  tf::Matrix3x3 m(q_input);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  cout << "foot" << x << ", "<< y << ", "<< yaw << ", "<< isleft << endl;


  foot_list foot;
  foot.x = x;
  foot.y = y;
  foot.z = 0.0f;
  foot.roll = 0.0f;
  foot.pitch = 0.0f;
  foot.yaw = yaw;
  if(isleft)
  {foot.foot_kind = 0;}
  else
  {foot.foot_kind = 1;}

  plan_foot_lists.push_back(foot);

  planner_msgs::foot_info foot_step_tmp;
  foot_step_tmp.pos_x = x;
  foot_step_tmp.pos_y = y;
  foot_step_tmp.pos_z = 0.0f;
  foot_step_tmp.ori_r = 0.0f;
  foot_step_tmp.ori_p = 0.0f;
  foot_step_tmp.ori_y = yaw;
  if(isleft)
  {foot_step_tmp.foot_kind = 0;}
  else
  {foot_step_tmp.foot_kind = 1;}

  foot_step_lists.foot_steps.push_back(foot_step_tmp);
}

void footstep_planner::init_foot_pose()
{

}

void footstep_planner::final_foot_pose()
{

}

void footstep_planner::create_footpose()
{
  float d= 0.1;

}
int footstep_planner::find_one_step_max_node(int start_node,double max_length)
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
   else if( i == (path_x.size()-1))
   {
        return i;
   }
  }

  return start_node+5;
}

void footstep_planner::creat_footstep(double foot_distance, double onestep)
{
  plan_foot_lists.clear();
  foot_step_lists.foot_steps.clear();
  foot_markers.markers.clear();
  cnt_of_foot = 0;
  if(path_x.size() == 0 || path_y.size() == 0){ }
  else
  {
  double d = foot_distance/2.0; // root to foot center distance
  double step_max_length = onestep; // one step foot maximum length
  float d_delta = 0.01; // dist scale
  float alpha_delta = deg2rad(10.0); // deg scale
  float alpha_limit = deg2rad(20.0); // deg scale
  tf::Matrix3x3 mi(qi);
  double ri, pi, yi;
  mi.getRPY(ri, pi, yi);

  double xi_shift = -sin(yi)*d;
  double yi_shift = cos(yi)*d;


  foot_draw(init_xy[0],init_xy[1]-d,qi,cnt_of_foot,false);
  cnt_of_foot++;
  foot_draw(init_xy[0],init_xy[1]+d,qi,cnt_of_foot,true);
  cnt_of_foot++;

  foot_draw(path_x.at(0)-xi_shift,path_y.at(0)-yi_shift,qi,cnt_of_foot,false);
  cnt_of_foot++;
  foot_draw(path_x.at(0)+xi_shift,path_y.at(0)+yi_shift,qi,cnt_of_foot,true);
  cnt_of_foot++;

  int start_node_index = 0;
  int cur_max_index = 0;

  int goal_start_node_index = 0;
  int goal_max_node_index = 0;
  // loop based foot pose generation need


  bool find_end_node = false;
  double yaw_cur, yaw_new, yaw_diff;
  double yaw_cal_final;
  for(size_t i=0; i<100;i++)
  {
    cur_max_index = find_one_step_max_node(start_node_index,step_max_length);
    curvature_cal(start_node_index,cur_max_index);

    double yaw_cal = root_path_slope_cal(start_node_index,cur_max_index);
    yaw_cal = yaw_cal + 1.57079632679;
    if(rad2deg(yaw_cal) >= 180 && rad2deg(yaw_cal) <= 270)
    {
        yaw_cal = yaw_cal - 3.14159265359;
    }
        if(i==0) { yaw_cur = yi; }
        yaw_new = yaw_cal;
        yaw_diff = yaw_new - yaw_cur;

    if(start_node_index == (path_x.size()-1) && start_node_index == (path_y.size()-1))
    {
        cout << "end of loop!" << endl;
        goal_start_node_index = start_node_index;
        goal_max_node_index = cur_max_index;
        yaw_cal_final = yaw_cal;
        break;     
    }
    else
    {
        if(abs(yaw_diff) < alpha_limit)
        {
            tf::Quaternion qc = tf::createQuaternionFromYaw(yaw_cal); // atan2 problem?
            tf::Matrix3x3 mc(qc);
            double rc,pc,yc;
            mc.getRPY(rc,pc,yc);
            double xc_shift = -sin(yc)*d;
            double yc_shift = cos(yc)*d;
            if( (cnt_of_foot%2) == 0 )
            {
             foot_draw(path_x.at(cur_max_index)-xc_shift,path_y.at(cur_max_index)-yc_shift,qc,cnt_of_foot,false);
            }
            else if( (cnt_of_foot%2) == 1)
            {
              foot_draw(path_x.at(cur_max_index)+xc_shift,path_y.at(cur_max_index)+yc_shift,qc,cnt_of_foot,true);
            }
            cnt_of_foot++;
            yaw_cur = yaw_new;
            start_node_index = cur_max_index;
        }
        else // rotation at current position
        {
            double input_yaw = 0.0f;
            if(yaw_diff>0)
            {
                input_yaw = yaw_cur + alpha_delta;
            }
            else
            {
                input_yaw = yaw_cur - alpha_delta;
            }
            tf::Quaternion qc = tf::createQuaternionFromYaw(input_yaw); // atan2 problem?
            tf::Matrix3x3 mc(qc);
            double rc,pc,yc;
            mc.getRPY(rc,pc,yc);
            double xc_shift = -sin(yc)*d;
            double yc_shift = cos(yc)*d;
            foot_draw(path_x.at(start_node_index)+xc_shift,path_y.at(start_node_index)+yc_shift,qc,cnt_of_foot,true);
            cnt_of_foot++;
            foot_draw(path_x.at(start_node_index)-xc_shift,path_y.at(start_node_index)-yc_shift,qc,cnt_of_foot,false);
            cnt_of_foot++;
            yaw_cur = input_yaw;
        }
     }
  }
  // end of foot position and orientation
  for(size_t i=0; i<100;i++)
  {
        double yaw_cal = root_path_slope_cal(goal_start_node_index,goal_max_node_index);
        yaw_cal = yaw_cal + 1.57079632679;
        if(rad2deg(yaw_cal) >= 180 && rad2deg(yaw_cal) <= 270)
        {
            yaw_cal = yaw_cal - 3.14159265359;
        }
        if(i==0) { yaw_cur = yi; }
        yaw_new = yaw_cal;
        yaw_diff = yaw_new - yaw_cur;
  }

  tf::Matrix3x3 mg(qg);
  double rg, pg, yg;
  mg.getRPY(rg, pg, yg);
  int x_end = path_x.size()-1;
  int y_end = path_y.size()-1;
  double xg_shift = -sin(yg)*d;
  double yg_shift = cos(yg)*d;
  if( (cnt_of_foot%2) == 1)
  {
      //foot_draw(path_x.at(x_end)+xg_shift,path_y.at(y_end)+yg_shift,qg,cnt_of_foot,true);
      foot_draw(goal_xy[0]+xg_shift,goal_xy[1]+yg_shift,qg,cnt_of_foot,true);
      cnt_of_foot++;
      //foot_draw(path_x.at(x_end)-xg_shift,path_y.at(y_end)-yg_shift,qg,cnt_of_foot,false);
      foot_draw(goal_xy[0]-xg_shift,goal_xy[1]-yg_shift,qg,cnt_of_foot,false);
      cnt_of_foot++;
  }
  else if( (cnt_of_foot%2) == 0)
  {
      //foot_draw(path_x.at(x_end)-xg_shift,path_y.at(y_end)-yg_shift,qg,cnt_of_foot,false);
      foot_draw(goal_xy[0]-xg_shift,goal_xy[1]-yg_shift,qg,cnt_of_foot,false);
      cnt_of_foot++;
      //foot_draw(path_x.at(x_end)+xg_shift,path_y.at(y_end)+yg_shift,qg,cnt_of_foot,true);
      foot_draw(goal_xy[0]+xg_shift,goal_xy[1]+yg_shift,qg,cnt_of_foot,true);
      cnt_of_foot++;
  }


  foot_box_pub.publish(foot_markers);
  foot_step_plan_pub.publish(foot_step_lists);
  }
}
double footstep_planner::root_path_slope_cal(int start_node, int end_node)
{
  double delta_x = path_x.at(end_node) - path_x.at(start_node);
  double delta_y = path_y.at(end_node) - path_y.at(start_node);
  double dAngle = -atan2(delta_x , delta_y);
  //dAngle *= (180.0/M_PI); // 0~360 notation
  //if( dAngle < 0.0 ) dAngle += 360.0;
  return dAngle;
}

void footstep_planner::curvature_cal(int start_node, int end_node)
{
  int CurvaturePointSize = end_node - start_node + 1;
  std::vector<double> vecCurvature(CurvaturePointSize);

  Eigen::Vector2d f1stDerivative;
  Eigen::Vector2d f2ndDerivative;

  Eigen::Vector2d pos(path_x.at(end_node),path_y.at(end_node));
  Eigen::Vector2d posOld(path_x.at(start_node/end_node),path_y.at(start_node/end_node));
  Eigen::Vector2d posOlder(path_x.at(start_node),path_y.at(start_node));
  f1stDerivative[0] =   pos[0]      - posOld[0];
  f1stDerivative[1] =   pos[1]      - posOld[1];
  f2ndDerivative[0] = - pos[0] + 2.0f * posOld[0] - posOlder[0];
  f2ndDerivative[1] = - pos[1] + 2.0f * posOld[1] - posOlder[1];
  double curvature2D = 0.0f;
  if(std::abs(f2ndDerivative[0]) > 10e-4 && std::abs(f2ndDerivative[1]) > 10e-4)
  {
    curvature2D = sqrt( std::abs(pow( f2ndDerivative[1]*f1stDerivative[0] - f2ndDerivative[0]*f1stDerivative[1], 2.0f ) / pow( f2ndDerivative[0] + f2ndDerivative[1], 3.0 ) ) );
  }
  vecCurvature.push_back(curvature2D);
/*  for(size_t i=0; i<CurvaturePointSize;i++)
  {

    pos[0] = path_x.at(i);
    pos[1] = path_y.at(i);
     cout << pos << endl;
    if(i==0) {posOld = posOlder = pos;}
    f1stDerivative[0] =   pos[0]      - posOld[0];
    f1stDerivative[1] =   pos[1]      - posOld[1];
    f2ndDerivative[0] = - pos[0] + 2.0f * posOld[0] - posOlder[0];
    f2ndDerivative[1] = - pos[1] + 2.0f * posOld[1] - posOlder[1];



    double curvature2D = 0.0f;
    if(std::abs(f2ndDerivative[0]) > 10e-4 && std::abs(f2ndDerivative[1]) > 10e-4)
    {
      curvature2D = sqrt( std::abs(pow( f2ndDerivative[1]*f1stDerivative[0] - f2ndDerivative[0]*f1stDerivative[1], 2.0f ) / pow( f2ndDerivative[0] + f2ndDerivative[1], 3.0 ) ) );
    }

    vecCurvature.push_back(curvature2D);
    posOlder = posOld;
    posOld = pos;
  }
*/
  node_curvatures.clear();
  for(size_t i=0; i<vecCurvature.size();i++)
  {
      node_curvatures.push_back(vecCurvature.at(i));
      if(vecCurvature.at(i) != 0)
      {
        //  cout << "(" << start_node << ","<< end_node << ")" << " curvt:" << vecCurvature.at(i) << endl;
      }
  }  
}

void footstep_planner::save_offline_footstep()
{
  ofstream fout;

  fout.open("/home/jimin/catkin_ws/src/nexus/data/footstepInfo.txt");

  for(size_t i=0; i<plan_foot_lists.size();i++)
  {
      foot_list foot = plan_foot_lists.at(i);
      fout<<foot.x<<"\t"<<foot.y<<"\t"<<foot.z<<"\t"<<foot.roll<<"\t"<<foot.pitch<<"\t"<<foot.yaw<<"\t"<<foot.foot_kind<<endl;
  }

  if(fout.is_open()==true) // 파일 닫기
  {
     fout.close();
  }
}

void footstep_planner::footstep_planner_cmd(const planner_msgs::Mapbuilder::ConstPtr &cmd)
{
  if(cmd->state == "footstep_plan")
  {
    ROS_INFO("%s",cmd->state.c_str());
    creat_footstep((double)cmd->val1,(double)cmd->val2);

  }
  else if(cmd->state == "save_footstep")
  {
    ROS_INFO("%s",cmd->state.c_str());
    save_offline_footstep();

  }
  else if(cmd->state == "t2")
  {
    ROS_INFO("%s",cmd->state.c_str());

  }
}
