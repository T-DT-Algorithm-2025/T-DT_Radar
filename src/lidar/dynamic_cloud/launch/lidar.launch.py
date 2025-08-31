import os
import sys
import yaml
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import TimerAction, Shutdown
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch

def dump_params(param_file_path, node_name):
    with open(param_file_path, 'r') as file:
        return [yaml.safe_load(file)[node_name]['ros__parameters']]

def generate_launch_description():
    
    def get_localization_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='localization_node',
            extra_arguments=[{'use_intra_process_comms': True},
                             {'use_multi_threaded_executor': True}],
        )
        
    def get_dynamic_cloud_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='dynamic_cloud_node',
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        
    def get_cluster_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='cluster_node',
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        
    def get_kalman_filter_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='kalman_filter_node',
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    def get_container(*nodes):
        print(os.getcwd())
        return ComposableNodeContainer(
            name='lidar_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=list(nodes),
            output='both',
            emulate_tty=True,
            on_exit=Shutdown(),
        )
        
    localization_node = get_localization_node('localization', 'tdt_radar::Localization')
    dynamic_cloud_node = get_dynamic_cloud_node('dynamic_cloud', 'tdt_radar::DynamicCloud')
    cluster_node = get_cluster_node('cluster', 'tdt_radar::Cluster')
    kalman_filter_node = get_kalman_filter_node('kalman_filter', 'tdt_radar::KalmanFilter')

    lidar_detector = get_container(
                                    localization_node,
                                    dynamic_cloud_node,
                                    cluster_node,
                                    kalman_filter_node
                                   )
    
    return LaunchDescription([
            lidar_detector
            ])