#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
import sys

import launch
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    EnvironmentVariable,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = 'mrs_octomap_planner'

    this_pkg_path = get_package_share_directory(pkg_name)
    namespace = 'octomap_planner'

    # #{ uav_name

    uav_name = LaunchConfiguration('uav_name')

    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=os.getenv('UAV_NAME', 'uav1'),
        description='The uav name used for namespacing.',
    ))

    # #} end of uav_name

    # #{ standalone

    standalone = LaunchConfiguration('standalone')

    declare_standalone = DeclareLaunchArgument(
        'standalone',
        default_value='true',
        description='Whether to start a as a standalone or load into an existing container.'
    )

    ld.add_action(declare_standalone)

    # #} end of standalone

    # #{ container_name

    container_name = LaunchConfiguration('container_name')

    declare_container_name = DeclareLaunchArgument(
        'container_name',
        default_value='',
        description='Name of an existing container to load into (if standalone is false)'
    )

    ld.add_action(declare_container_name)

    # #} end of container_name

    # #{ custom_config

    custom_config = LaunchConfiguration('custom_config')

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    ld.add_action(DeclareLaunchArgument(
        'custom_config',
        default_value='',
        description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
    ))

    # behaviour:
    #     custom_config == "" => custom_config: ""
    #     custom_config == "/<path>" => custom_config: "/<path>"
    #     custom_config == "<path>" => custom_config: "$(pwd)/<path>"
    custom_config = IfElseSubstitution(
        condition=PythonExpression(['"', custom_config, '" != "" and ', 'not "', custom_config, '".startswith("/")']),
        if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), custom_config]),
        else_value=custom_config
    )

    # #} end of custom_config

    # #{ use_sim_time

    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value=os.getenv('USE_SIM_TIME', 'false'),
        description='Should the node subscribe to sim time?',
    ))

    # #} end of use_sim_time

    # #{ log_level

    ld.add_action(DeclareLaunchArgument(name='log_level', default_value='info'))

    ld.add_action(DeclareLaunchArgument(name='topic_namespace', default_value=''))

    # #} end of log_level

    # #{ default node

    default_node = ComposableNode(
        package=pkg_name,
        plugin=pkg_name+'::OctomapPlanner',
        namespace=uav_name,
        name=namespace,

        parameters=[
            {"uav_name": uav_name},
            {'use_sim_time': use_sim_time},
            {'config': this_pkg_path + '/config/octomap_planner.yaml'},
            {'custom_config': custom_config},
        ],

        remappings=[
            # topics in
            ('~/tracker_cmd_in', "control_manager/tracker_cmd"),
            ('~/octomap_in', "octomap_server/octomap_local_binary"),
            ('~/control_manager_diag_in', "control_manager/diagnostics"),
            ('~/constraints_in', "control_manager/current_constraints"),
            # topics out
            ('~/diagnostics_out', "~/diagnostics"),
            ('~/virtual_obstacles_out', "~/vis_virtual_obstacles"),
            # services in
            ('~/goto_in', "~/goto"),
            ('~/stop_in', "~/stop"),
            ('~/reference_in', "~/reference"),
            ('~/planner_type_in', "~/set_planner"),
            ('~/set_safety_distance_in', "~/distance"),
            ('~/set_max_altitude_in', "~/altitude"),
            ('~/add_virtual_obstacle_in', "~/add_virtual_obstacle"),
            ('~/remove_virtual_obstacles_in', "~/remove_virtual_obstacles"),
            # services out
            ('~/trajectory_generation_out', "trajectory_generation/get_path"),
            ('~/trajectory_reference_out', "control_manager/trajectory_reference"),
            ('~/hover_out', "control_manager/hover"),
        ],
    )

    load_into_existing = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[default_node],
        condition=UnlessCondition(standalone)
    )

    ld.add_action(load_into_existing)

    # #} end of default node

    # #{ standalone container

    standalone_container = ComposableNodeContainer(
        namespace=uav_name,
        name=namespace+'_container',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        # prefix=['debug_roslaunch ' + os.ttyname(sys.stdout.fileno())],
        composable_node_descriptions=[default_node],
        condition=IfCondition(standalone)
    )

    ld.add_action(standalone_container)

    # #} end of standalone container

    return ld
