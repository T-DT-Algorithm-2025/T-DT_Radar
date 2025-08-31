import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory

sys.path.append(os.path.join(get_package_share_directory('tdt_vision'), 'launch'))

from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import TimerAction, Shutdown
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    def get_camera_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='vision_camera_node',
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        
    def get_foxglove_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='foxglove_bridge_node',
            parameters=[ {'send_buffer_limit': 1000000000}],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
  
    def get_radar_detect_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='radar_detect_node',
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        
    def get_radar_resolve_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='radar_resolve_node',
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        
    def get_debug_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='debug_node',
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        
    def get_record_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='record_node',
            parameters=[ ] ,
            extra_arguments=[{'use_intra_process_comms': True}]
        )        

    def get_camera_detector_container(camera_node,radar_detect_node,radar_resolve_node,foxglove_node,debug_node,record_node):
        return ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                camera_node,
                radar_detect_node,
                radar_resolve_node,
                foxglove_node,
                debug_node,
                record_node
            ],
            output='both',
            emulate_tty=True,
            on_exit=Shutdown(),
        )

    # 相机包不包含在内，需自行准备
    hik_camera_node = get_camera_node('tdt_vision', 'tdt_vision::NodeCamera')
    radar_detect_node = get_radar_detect_node('tdt_vision', 'tdt_radar::Detect')
    radar_resolve_node = get_radar_resolve_node('tdt_vision', 'tdt_radar::Resolve')
    foxglove_node = get_foxglove_node('foxglove_bridge', 'foxglove_bridge::FoxgloveBridge')
    tdt_debug_node = get_debug_node('tdt_vision', 'tdt_vision::NodeDebug')
    record_node = get_record_node('databag_tool', 'BagRecorderNode')

    cam_detector = get_camera_detector_container(hik_camera_node,radar_detect_node,radar_resolve_node,foxglove_node,tdt_debug_node,record_node)
    plugin_map_launch_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('tdt_vision'), 'launch', 'map_server_launch.py')]),
             )
    return LaunchDescription([
            cam_detector,
            plugin_map_launch_cmd,
        ])