cmake_minimum_required(VERSION 2.8.3)
project(planner_msgs)


find_package(catkin REQUIRED
   message_generation
   std_msgs
)

add_message_files(
   FILES
   Mapbuilder.msg   
   foot_info.msg
   foot_info_array.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs  
)
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

