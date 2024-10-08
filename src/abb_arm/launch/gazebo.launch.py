#!/usr/bin/python3

# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import yaml
from moveit_configs_utils import MoveItConfigsBuilder

# LOAD FILE:
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None
# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None

# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():
    
    # *********************** Gazebo *********************** # 
    
    # DECLARE Gazebo WORLD file:
    abb1200_ros2_gazebo = os.path.join(
        get_package_share_directory('abb_arm'),
        'worlds',
        'my_empty_world.world')
    # DECLARE Gazebo LAUNCH file:
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                launch_arguments={'world': abb1200_ros2_gazebo}.items(),
             )

    # ***** ROBOT DESCRIPTION ***** #
    # ABB-IRB120 Description file package:
    abb1200_description_path = os.path.join(
        get_package_share_directory('abb_arm'))
    # ABB-IRB120 ROBOT urdf file path:
    xacro_file = os.path.join(abb1200_description_path,
                              'description','urdf','abb_1200.urdf.xacro')
    # Generate ROBOT_DESCRIPTION for ABB-IRB120:
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}

    # ROBOT STATE PUBLISHER NODE:
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_state_publisher",
        output='both',
        parameters=[robot_description,
                    {"use_sim_time": True}]
    )
    # Static TF:
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_0"],
        parameters=[{"use_sim_time": True}]
    )

    # SPAWN ROBOT TO GAZEBO:
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'abb_1200'],
                        output='both',
                        parameters=[{"use_sim_time": True}])

    # ***** CONTROLLERS ***** #
    # Joint STATE BROADCASTER:
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": True}]
    )
    # Joint TRAJECTORY Controller:
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": True}]
    )


    # *********************** MoveIt!2 *********************** #   
    launch_arguments = {
        "robot_ip": "xxx.yyy.zzz.www",
        "use_fake_hardware": "true",
        "gripper": "link_6",
        "dof": "6",
    }
    moveit_config = (
        MoveItConfigsBuilder(
            "abb_arm", package_name="abb_1200_moveit_config"
        )
        .robot_description(mappings=launch_arguments)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl"]
        )
        .to_moveit_configs()
    )
    
    # Command-line argument: RVIZ file?
    rviz_arg = DeclareLaunchArgument(
        "rviz_file", default_value="False", description="Load RVIZ file."
    )

    # *** PLANNING CONTEXT *** #
    # Robot description, URDF:
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("abb_arm"),'description','urdf','abb_1200.urdf.xacro',
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}
    # Robot description, SRDF:
    robot_description_semantic_config = load_file(
        "abb_1200_moveit_config", "config/abb_1200.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # Kinematics.yaml file:
    kinematics_yaml = load_yaml(
        "abb_1200_moveit_config", "config/kinematics.yaml"
    )
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # MoveIt!2 Controllers:
    moveit_simple_controllers_yaml = load_yaml(
        "abb_1200_moveit_config", "config/moveit_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 5.0,
        "trajectory_execution.allowed_goal_duration_margin": 5.0,
        "trajectory_execution.allowed_start_tolerance": 5.0,
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description":True, 
        "publish_robot_description_semantic":True,
    }

    # START NODE -> MOVE GROUP:
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(),
                    moveit_controllers,
                    {"use_sim_time": True}],
    )

    # RVIZ:
    load_RVIZfile = LaunchConfiguration("rviz_file")
    rviz_base = os.path.join(get_package_share_directory("abb_1200_moveit_config"), "config")
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True}
        ],
    )

    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        gazebo, 
        spawn_entity,
        node_robot_state_publisher,
        static_tf,
        RegisterEventHandler(
            OnProcessExit(
                target_action = spawn_entity,
                on_exit = [
                    joint_state_broadcaster_spawner,  
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action = joint_state_broadcaster_spawner,
                on_exit = [
                    joint_trajectory_controller_spawner,
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action = joint_trajectory_controller_spawner,
                on_exit = [
                    rviz_arg,
                    rviz_node,
                    run_move_group_node,
                ]
            )
        ),
    ])
        