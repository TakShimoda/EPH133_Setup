  
# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2022 Intel Corporation. All Rights Reserved.

"""Launch realsense2_camera node."""
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

cfg_36h11 = {
    "image_transport": "raw",
    "family": "36h11",
    "size": 0.0435,
    "max_hamming": 0,
    "z_up": True
}

configurable_parameters = [{'name': 'camera_name',                  'default': 'B05', 'description': 'camera unique name'},
                           {'name': 'serial_no',                    'default': "''", 'description': 'choose device by serial number', 'param_type': 'string'},
                           {'name': 'usb_port_id',                  'default': "''", 'description': 'choose device by usb port id'},
                           {'name': 'device_type',                  'default': "D435i", 'description': 'choose device by type'},
                           {'name': 'config_file',                  'default': "''", 'description': 'yaml config file'},
                           {'name': 'unite_imu_method',             'default': '0', 'description': '[0-None, 1-copy, 2-linear_interpolation]'},
                           {'name': 'json_file_path',               'default': "''", 'description': 'allows advanced configuration'},
                           {'name': 'log_level',                    'default': 'info', 'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
                           {'name': 'output',                       'default': 'screen', 'description': 'pipe node output [screen|log]'},
                           {'name': 'depth_module.profile',         'default': '640,480,30', 'description': 'depth module profile'},                           
                           {'name': 'enable_depth',                 'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'rgb_camera.profile',           'default': '640,480,30', 'description': 'color image width'},
                           {'name': 'enable_color',                 'default': 'true', 'description': 'enable color stream'},
                           {'name': 'enable_infra1',                'default': 'true', 'description': 'enable infra1 stream'},
                           {'name': 'enable_infra2',                'default': 'true', 'description': 'enable infra2 stream'},
                           {'name': 'infra_rgb',                    'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'tracking_module.profile',      'default': '0,0,0', 'description': 'fisheye width'},
                           {'name': 'enable_fisheye1',              'default': 'true', 'description': 'enable fisheye1 stream'},
                           {'name': 'enable_fisheye2',              'default': 'true', 'description': 'enable fisheye2 stream'},
                           {'name': 'enable_confidence',            'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'gyro_fps',                     'default': '0', 'description': "''"},                           
                           {'name': 'accel_fps',                    'default': '0', 'description': "''"},                           
                           {'name': 'enable_gyro',                  'default': 'false', 'description': "''"},                           
                           {'name': 'enable_accel',                 'default': 'false', 'description': "''"},                           
                           {'name': 'enable_pose',                  'default': 'true', 'description': "''"},                           
                           {'name': 'pose_fps',                     'default': '200', 'description': "''"},                           
                           {'name': 'pointcloud.enable',            'default': 'false', 'description': ''}, 
                           {'name': 'pointcloud.stream_filter',     'default': '2', 'description': 'texture stream for pointcloud'},
                           {'name': 'pointcloud.stream_index_filter','default': '0', 'description': 'texture stream index for pointcloud'},
                           {'name': 'enable_sync',                  'default': 'false', 'description': "''"},                           
                           {'name': 'align_depth.enable',           'default': 'false', 'description': "''"},                           
                           {'name': 'colorizer.enable',             'default': 'false', 'description': "''"},
                           {'name': 'clip_distance',                'default': '-2.', 'description': "''"},                           
                           {'name': 'linear_accel_cov',             'default': '0.01', 'description': "''"},                           
                           {'name': 'initial_reset',                'default': 'true', 'description': "''"},                           
                           {'name': 'allow_no_texture_points',      'default': 'false', 'description': "''"},                           
                           {'name': 'ordered_pc',                   'default': 'false', 'description': ''},
                           {'name': 'calib_odom_file',              'default': "''", 'description': "''"},
                           {'name': 'topic_odom_in',                'default': "''", 'description': 'topic for T265 wheel odometry'},
                           {'name': 'tf_publish_rate',              'default': '0.0', 'description': 'Rate of publishing static_tf'},
                           {'name': 'diagnostics_period',           'default': '0.0', 'description': 'Rate of publishing diagnostics. 0=Disabled'},
                           {'name': 'decimation_filter.enable',     'default': 'true', 'description': 'Rate of publishing static_tf'},
                           {'name': 'rosbag_filename',              'default': "''", 'description': 'A realsense bagfile to run from as a device'},
                           {'name': 'depth_module.exposure.1',     'default': '7500', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'depth_module.gain.1',         'default': '16', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'depth_module.exposure.2',     'default': '1', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'depth_module.gain.2',         'default': '16', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'wait_for_device_timeout',      'default': '-1.', 'description': 'Timeout for waiting for device to connect (Seconds)'},
                           {'name': 'reconnect_timeout',            'default': '6.', 'description': 'Timeout(seconds) between consequtive reconnection attempts'},
                          ]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def generate_launch_description():

    #Apriltag
    composable_node = ComposableNode(
        node_name='apriltag',
        package='apriltag_ros', node_plugin='AprilTagNode',
        remappings=[("/apriltag/image", "/B05/D435i/color/image_raw"), ("/apriltag/camera_info", "/B05/D435i/color/camera_info")],
        parameters=[cfg_36h11])
    container = ComposableNodeContainer(
        node_name='tag_container',
        node_namespace='apriltag',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[composable_node],
        output='screen'
    )

    log_level = 'info'
    if (os.getenv('ROS_DISTRO') == "dashing") or (os.getenv('ROS_DISTRO') == "eloquent"):
        return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
            # Realsense
            launch_ros.actions.Node(
                condition=IfCondition(PythonExpression([LaunchConfiguration('config_file'), " == ''"])),
                package='realsense2_camera',
                node_namespace=LaunchConfiguration("camera_name"),
                node_name=LaunchConfiguration("camera_name"),
                node_executable='realsense2_camera_node',
                prefix=['stdbuf -o L'],
                parameters=[set_configurable_parameters(configurable_parameters)
                            ],
                output='screen',
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                ),
            launch_ros.actions.Node(
                condition=IfCondition(PythonExpression([LaunchConfiguration('config_file'), " != ''"])),
                package='realsense2_camera',
                node_namespace=LaunchConfiguration("camera_name"),
                node_name=LaunchConfiguration("camera_name"),
                node_executable='realsense2_camera_node',
                prefix=['stdbuf -o L'],
                parameters=[set_configurable_parameters(configurable_parameters)
                            , PythonExpression([LaunchConfiguration("config_file")])
                            ],
                output='screen',
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                ), container
            ])
    else:
        return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        # Realsense
        launch_ros.actions.Node(
            condition=IfCondition(PythonExpression([LaunchConfiguration('config_file'), " == ''"])),
            package='realsense2_camera',
            namespace='B05/D435i',  # Set the desired namespace here
            name=LaunchConfiguration("camera_name"),
            executable='realsense2_camera_node',
            parameters=[set_configurable_parameters(configurable_parameters)],
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            emulate_tty=True,
        ),
        launch_ros.actions.Node(
            condition=IfCondition(PythonExpression([LaunchConfiguration('config_file'), " != ''"])),
            package='realsense2_camera',
            namespace='B05/D435i',  # Set the desired namespace here
            name=LaunchConfiguration("camera_name"),
            executable='realsense2_camera_node',
            parameters=[set_configurable_parameters(configurable_parameters), PythonExpression([LaunchConfiguration("config_file")])],
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            emulate_tty=True,
        ), container
    ])
