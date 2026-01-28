#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
)

from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = 'mrs_octomap_planner'

    this_pkg_path = get_package_share_directory(pkg_name)
    namespace = 'minimal_octomap_planner'

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
        plugin=pkg_name+'::MinimalOctomapPlanner',
        namespace=uav_name,
        name=namespace,

        parameters=[
            {'use_sim_time': use_sim_time},
            {'config': this_pkg_path + '/config/minimal_planner.yaml'},
        ],

        remappings=[
            # topics in
            ('~/octomap_in', "octomap_server/octomap_local_full"),
            # services in
            ('~/get_path_in', "~/get_path"),
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
