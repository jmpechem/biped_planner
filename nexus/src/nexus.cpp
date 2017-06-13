#include "nexus/def_include.h"
#include "nexus/map_builder.h"
#include "nexus/traversability.h"
#include "nexus/root_path_finder.h"
#include "nexus/footstep_planner.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nexus_system");
  ros::NodeHandle nh;

  map_builder obj_map_builder;
  traversability obj_traversability;
  root_path_finder obj_rpf;
  footstep_planner obj_fsp;
  ros::Rate rate(10.0);


  while(nh.ok())
    {
    //  g_map.Update();
      ros::spinOnce();
    }
}
