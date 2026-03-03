# Author: Addison Sears-Collins 
# Date: August 30, 2021
# Modified: Megan Neville, March 2, 2026

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

  # =============================
  # 🔥 ASK USER FOR DIRECTION
  # =============================
  while True:
      direction_input = input("Enter direction (left/right): ").strip().lower()
      if direction_input in ['left', 'right']:
          break
      print("Please enter 'left' or 'right'.")

  print(f"\nLaunching {direction_input.upper()} configuration...\n")


  pkg_share = FindPackageShare(package='eced3901').find('eced3901')
  default_launch_dir = os.path.join(pkg_share, 'launch')
  default_model_path = os.path.join(pkg_share, 'models/eced3901bot.urdf')
  robot_name_in_urdf = 'eced3901bot'
  default_rviz_config_path = os.path.join(pkg_share, 'rviz/nav2.rviz')

  nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup') 
  nav2_launch_dir = os.path.join(nav2_dir, 'launch') 

  nav2_params_path = os.path.join(pkg_share, 'params', 'nav2_params.yaml')

  nav2_bt_path = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')
  behavior_tree_xml_path = os.path.join(
      nav2_bt_path,
      'behavior_trees',
      'navigate_w_replanning_and_recovery.xml'
  )

  if direction_input == 'left':
      static_map_path = os.path.join(pkg_share, 'maps', 'maze_map_left.yaml')
      wp_executable = 'leftcoast.launch.py'
  else:
      static_map_path = os.path.join(pkg_share, 'maps', 'maze_map_right.yaml')
      wp_executable = 'rightcoast.launch.py'

  autostart = LaunchConfiguration('autostart')
  map_yaml_file = LaunchConfiguration('map')
  model = LaunchConfiguration('model')
  namespace = LaunchConfiguration('namespace')
  params_file = LaunchConfiguration('params_file')
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  slam = LaunchConfiguration('slam')
  use_namespace = LaunchConfiguration('use_namespace')
  use_rviz = LaunchConfiguration('use_rviz')
  use_sim_time = LaunchConfiguration('use_sim_time')

  declare_namespace_cmd = DeclareLaunchArgument(
      name='namespace',
      default_value='',
      description='Top-level namespace')

  declare_use_namespace_cmd = DeclareLaunchArgument(
      name='use_namespace',
      default_value='False')

  declare_autostart_cmd = DeclareLaunchArgument(
      name='autostart',
      default_value='true')

  declare_map_yaml_cmd = DeclareLaunchArgument(
      name='map',
      default_value=static_map_path)

  declare_model_path_cmd = DeclareLaunchArgument(
      name='model',
      default_value=default_model_path)

  declare_params_file_cmd = DeclareLaunchArgument(
      name='params_file',
      default_value=nav2_params_path)

  declare_rviz_config_file_cmd = DeclareLaunchArgument(
      name='rviz_config_file',
      default_value=default_rviz_config_path)

  declare_slam_cmd = DeclareLaunchArgument(
      name='slam',
      default_value='False')

  declare_use_rviz_cmd = DeclareLaunchArgument(
      name='use_rviz',
      default_value='True')

  declare_use_sim_time_cmd = DeclareLaunchArgument(
      name='use_sim_time',
      default_value='True')

  # Launch RViz
  start_rviz_cmd = Node(
      condition=IfCondition(use_rviz),
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      output='screen',
      arguments=['-d', rviz_config_file]
  )

  # Launch Nav2
  start_ros2_navigation_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(nav2_launch_dir, 'bringup_launch.py')),
      launch_arguments={
          'namespace': namespace,
          'use_namespace': use_namespace,
          'slam': slam,
          'map': map_yaml_file,
          'use_sim_time': use_sim_time,
          'params_file': params_file,
          'autostart': autostart
      }.items()
  )

  # Launch Waypoint Follower
  start_wpfollow = Node(
      package='eced3901',
      executable=wp_executable,
      name='wp_follower',
      output='screen'
  )

  ld = LaunchDescription()

  ld.add_action(declare_namespace_cmd)
  ld.add_action(declare_use_namespace_cmd)
  ld.add_action(declare_autostart_cmd)
  ld.add_action(declare_map_yaml_cmd)
  ld.add_action(declare_model_path_cmd)
  ld.add_action(declare_params_file_cmd)
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_slam_cmd)
  ld.add_action(declare_use_rviz_cmd)
  ld.add_action(declare_use_sim_time_cmd)

  ld.add_action(start_rviz_cmd)
  ld.add_action(start_ros2_navigation_cmd)
  ld.add_action(start_wpfollow)

  return ld