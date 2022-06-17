import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  # Set environment variables
  SetEnvironmentVariable(name='LC_NUMERIC', value='en_US.UTF-8')

  # Set the path to this package.
  pkg_share = FindPackageShare(package='bobik_robot').find('bobik_robot')

  # Set the path to the RViz configuration settings
  default_rviz_config_path = os.path.join(pkg_share, 'rviz/rviz_basic_settings.rviz')

  # Set the path to the URDF file
  default_urdf_model_path = os.path.join(pkg_share, 'urdf/bobik_robot.urdf.xacro')

  map_yaml_file = os.path.join(get_package_share_directory('bobik_robot'), 'maps', 'place.yaml')

  ########### YOU DO NOT NEED TO CHANGE ANYTHING BELOW THIS LINE ##############  
  # Launch configuration variables specific to simulation
  gui = LaunchConfiguration('gui')
  urdf_model = LaunchConfiguration('urdf_model')
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  slam = LaunchConfiguration('slam')
  use_rviz = LaunchConfiguration('use_rviz')
  use_sim_time = LaunchConfiguration('use_sim_time')

  # Declare the launch arguments  
  declare_urdf_model_path_cmd = DeclareLaunchArgument(
    name='urdf_model', 
    default_value=default_urdf_model_path, 
    description='Absolute path to robot urdf file')

  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')

  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')

  declare_slam_cmd = DeclareLaunchArgument(
    name='slam',
    default_value='False',
    description='Whether to run SLAM')

  declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Whether to start RVIZ')

  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='False',
    description='Use simulation (Gazebo) clock if true')


  # Specify the actions

  # Bobik bridge to mediate data between robot and main over ZeroMQ
  start_bobik_bridge_node = Node(
    package='bobik_bridge',
    executable='bobik_bridge',
    name='bobik_bridge')

  # Bobik high level skills
  start_bobik_robot_node = Node(
    package='bobik_robot',
    executable='bobik_robot',
    name='bobik_robot')

  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'use_sim_time': use_sim_time, 
    'robot_description': Command(['xacro ', urdf_model])}],
    arguments=[default_urdf_model_path])

  # start Nav2 localization
  start_localization_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
  )

  # Launch RViz
  start_rviz_cmd = Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])

  start_ros2_navigation_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(FindPackageShare(package='nav2_bringup').find('nav2_bringup'), 'launch/bringup_launch.py')),
    launch_arguments = {
      'slam': 'True',
      'map': map_yaml_file,
      'use_sim_time': use_sim_time,
      'params_file': os.path.join(pkg_share, 'params', 'nav2_params.yaml'),
      'default_bt_xml_filename': os.path.join(FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator'), 'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
      'autostart': 'True',
    }.items())

  start_rosbridge_server_cmd = IncludeLaunchDescription(
    AnyLaunchDescriptionSource(os.path.join(FindPackageShare(package='bobik_robot').find('bobik_robot'), 'launch/rosbridge_websocket_launch.xml')),
    launch_arguments = {
      'ssl': 'true',
      'certfile': '/home/honza/projects/bobik/bobik_web/cert/server.crt',
      'keyfile': '/home/honza/projects/bobik/bobik_web/cert/server.key'
    }.items())

  # Create the launch description and populate
  ld = LaunchDescription()
   # Declare the launch options
  ld.add_action(declare_urdf_model_path_cmd)
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)  
  ld.add_action(declare_use_rviz_cmd) 
  ld.add_action(declare_use_sim_time_cmd)

  # Add any actions
  ld.add_action(start_bobik_bridge_node)
  ld.add_action(start_bobik_robot_node)
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(start_localization_node)
  ld.add_action(start_ros2_navigation_cmd)
  ld.add_action(start_rosbridge_server_cmd)
  ld.add_action(start_rviz_cmd)

  return ld
