#!/bin/bash

args=""
for option in "$@"; do
  if [ "$option" == "--sem" ]; then
    args="$args /elevation_mapping/elevation_map_raw:=/recorded/elevation_mapping/elevation_map_raw \
    /elevation_mapping/semantic_map:=/recorded/elevation_mapping/semantic_map"
  elif [ "$option" == "--wvn" ]; then
    args="$args /wild_visual_navigation_node/front/camera_info:=/recorded/wild_visual_navigation_node/front/camera_info \
    /wild_visual_navigation_node/front/confidence:=/recorded/wild_visual_navigation_node/front/confidence \
    /wild_visual_navigation_node/front/feat:=/recorded/wild_visual_navigation_node/front/feat \
    /wild_visual_navigation_node/front/image_input:=/recorded/wild_visual_navigation_node/front/image_input \
    /wild_visual_navigation_node/front/traversability:=/recorded/wild_visual_navigation_node/front/traversability \
    /wild_visual_navigation_node/graph_footprints:=/recorded/wild_visual_navigation_node/graph_footprints \
    /wild_visual_navigation_node/graph_footprints_array:=/recorded/wild_visual_navigation_node/graph_footprints_array \
    /wild_visual_navigation_node/mission_graph:=/recorded/wild_visual_navigation_node/mission_graph \
    /wild_visual_navigation_node/rear/camera_info:=/recorded/wild_visual_navigation_node/rear/camera_info \
    /wild_visual_navigation_node/rear/confidence:=/recorded/wild_visual_navigation_node/rear/confidence \
    /wild_visual_navigation_node/rear/image_input:=/recorded/wild_visual_navigation_node/rear/image_input \
    /wild_visual_navigation_node/rear/traversability:=/recorded/wild_visual_navigation_node/rear/traversability \
    /wild_visual_navigation_node/supervision_graph:=/recorded/wild_visual_navigation_node/supervision_graph \
    /wild_visual_navigation_visu_front_trav/traversability:=/recorded/wild_visual_navigation_visu_front_trav/traversability \
    /wild_visual_navigation_visu_rear_trav/traversability:=/recorded/wild_visual_navigation_visu_rear_trav/traversability"
  elif [ "$option" == "--flp" ]; then
    args="$args /field_local_planner/action_server/status:=/recorded/field_local_planner/action_server/status \
    /field_local_planner/current_base:=/recorded/field_local_planner/current_base \
    /field_local_planner/current_goal:=/recorded/field_local_planner/current_goal \
    /field_local_planner/parameter_descriptions:=/recorded/field_local_planner/parameter_descriptions \
    /field_local_planner/parameter_updates:=/recorded/field_local_planner/parameter_updates \
    /field_local_planner/path:=/recorded/field_local_planner/path \
    /field_local_planner/real_carrot:=/recorded/field_local_planner/real_carrot \
    /field_local_planner/rmp/control_points:=/recorded/field_local_planner/rmp/control_points \
    /field_local_planner/rmp/parameter_descriptions:=/recorded/field_local_planner/rmp/parameter_descriptions \
    /field_local_planner/rmp/parameter_updates:=/recorded/field_local_planner/rmp/parameter_updates \
    /field_local_planner/status:=/recorded/field_local_planner/status \
    /elevation_mapping/elevation_map_wifi:=/recorded/elevation_mapping/elevation_map_wifi"
  elif [ "$option" == "--tf" ]; then
    args="$args /tf:=/recorded/tf" 
    # /tf_static:=/recorded/tf_static"
    echo "rosrun anymal_rsl_launch replay.py c /media/Data/Datasets/2023_Oxford_Testing/2023_01_27_Oxford_Park/mission_data/2023-01-27-11-00-22/2023-01-27-11-00-22_anymal-coyote-lpc_mission.yaml"
  elif [ "$option" == "--tm"]; then
    args="$args "
    #/husky_velocity_controller/odom:=
    
    """
    /gps:=                                  2256 msgs    : sensor_msgs/NavSatFix                  
             /husky_velocity_controller/cmd_vel   19567 msgs    : geometry_msgs/Twist                    
             /husky_velocity_controller/odom       4246 msgs    : nav_msgs/Odometry                      
             /joy_teleop/cmd_vel                  19568 msgs    : geometry_msgs/Twist                    
             /joy_teleop/cmd_vel_raw              19559 msgs    : geometry_msgs/Twist                    
             /microstrain/imu/data                45120 msgs    : sensor_msgs/Imu                        
             /microstrain/nav/filtered_imu/data   45119 msgs    : sensor_msgs/Imu                        
             /microstrain/nav/heading             45120 msgs    : microstrain_inertial_msgs/FilterHeading
             /microstrain/nav/odom                45119 msgs    : nav_msgs/Odometry                      
             /oak_front/color/image_raw            6772 msgs    : sensor_msgs/Image                      
             /oak_front/depth/image_raw            6772 msgs    : sensor_msgs/Image                      
             /oak_left/color/image_raw             6772 msgs    : sensor_msgs/Image                      
             /oak_left/depth/image_raw             6772 msgs    : sensor_msgs/Image                      
             /oak_right/color/image_raw            6772 msgs    : sensor_msgs/Image                      
             /oak_right/depth/image_raw            6771 msgs    : sensor_msgs/Image                      
             /status_pc                            1806 msgs    : mrl_husky_pc/HuskyPCStatus             
             /tf  
      """"  


  else
    args="$args $option"
  fi
done

echo args
rosparam set use_sim_time true
rosbag play --clock $args
