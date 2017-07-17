#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "planner_msgs/Mapbuilder.h"
#include "thormang_ctrl_msgs/JointState.h"

ros::Publisher rviz_joint_pub;

const std::string JointName[40] = {"WaistPitch","WaistYaw",
                              "R_ShoulderPitch","R_ShoulderRoll","R_ShoulderYaw","R_ElbowRoll","R_WristYaw","R_WristRoll","R_HandYaw",
                              "L_ShoulderPitch","L_ShoulderRoll","L_ShoulderYaw","L_ElbowRoll","L_WristYaw","L_WristRoll","L_HandYaw",
                              "R_HipYaw","R_HipRoll","R_HipPitch","R_KneePitch","R_AnklePitch","R_AnkleRoll",
                              "L_HipYaw","L_HipRoll","L_HipPitch","L_KneePitch","L_AnklePitch","L_AnkleRoll"};


void real_joint_cb(const thormang_ctrl_msgs::JointState::ConstPtr &joint_value)
{
  sensor_msgs::JointState view_model;
  view_model.header.stamp = ros::Time::now();
  view_model.name.resize(joint_value->id.size());
  view_model.position.resize(joint_value->id.size());
  for(int i=0; i< joint_value->id.size();i++)
  {
      view_model.name[i] = JointName[i];
      view_model.position[i] = joint_value->angle[i];
  }
  rviz_joint_pub.publish(view_model);
}

int main(int argc, char **argv)
{
   ros::init(argc,argv,"high_noon");
   ros::NodeHandle nh;
   rviz_joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states",1);
   ros::Subscriber real_joint_sub = nh.subscribe<thormang_ctrl_msgs::JointState>("joint_state",1,real_joint_cb);


   ros::Rate r(30);
   float angle = -1.575f;
   while(ros::ok())
   {
      /*
      view_model.header.stamp = ros::Time::now();
      view_model.name.resize(28);
      view_model.position.resize(28);
      view_model.name[0] = "L_HipYaw";
      view_model.name[1] = "L_HipRoll";
      view_model.name[2] = "L_HipPitch";
      view_model.name[3] = "L_KneePitch";
      view_model.name[4] = "L_AnklePitch";
      view_model.name[5] = "L_AnckleRoll";
      view_model.name[6] = "R_HipYaw";
      view_model.name[7] = "R_HipRoll";
      view_model.name[8] = "R_HipPitch";
      view_model.name[9] = "R_KneePitch";
      view_model.name[10] = "R_AncklePitch";
      view_model.name[11] = "R_AnckleRoll";
      view_model.name[12] = "WaistPitch";
      view_model.name[13] = "WaistYaw";

      view_model.name[14] = "L_ShoulderPitch";
      view_model.name[15] = "L_ShoulderRoll";
      view_model.name[16] = "L_ShoulderYaw";
      view_model.name[17] = "L_ElbowRoll";
      view_model.name[18] = "L_WristYaw";
      view_model.name[19] = "L_WristRoll";
      view_model.name[20] = "L_HandYaw";

      view_model.name[21] = "R_ShoulderPitch";
      view_model.name[22] = "R_ShoulderRoll";
      view_model.name[23] = "R_ShoulderYaw";
      view_model.name[24] = "R_ElbowRoll";
      view_model.name[25] = "R_WristYaw";
      view_model.name[26] = "R_WristRoll";
      view_model.name[27] = "R_HandYaw";

      angle += 0.01;

      for(int i=0;i<28;i++)
      {

        view_model.position[i] = 0.0f;
      }

      rviz_joint_pub.publish(view_model);
      */

      ros::spinOnce();
      r.sleep();
   }

}
