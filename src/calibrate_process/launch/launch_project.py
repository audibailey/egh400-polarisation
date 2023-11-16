import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pol_cam_test_params = os.path.join(
        get_package_share_directory('lucid_vision_driver'),
        'param',
        'test.param.yaml'
    )
    pol_cam_full_params = os.path.join(
        get_package_share_directory('lucid_vision_driver'),
        'config',
        'test_full_res.yaml'
    )

    return LaunchDescription([
        Node(
            package='lucid_vision_driver',
            name='arena_camera_node',
            executable='arena_camera_node_exe',
            parameters=[
                pol_cam_test_params,
                {'camera_info_url': 'file://' + pol_cam_full_params},
                {'pixel_format': 'Mono8'},
                {'horizontal_binning': 1},
                {'vertical_binning': 1},
                {'serial_no': 233600049},
                {'use_default_device_settings': False},
                {'exposure_auto': True},
                {'exposure_target': 187},
                {'gain_auto': True},
                {'gain_target': 1},
                {'gamma_target': 1.0},
                {'fps': 10}
            ]
        ),
        Node(
            namespace='vrpn_mocap',
            package='vrpn_mocap',
            name='vrpn_client_node',
            executable='client_node',
            parameters=[
                {'server': '131.181.135.150'},
                {'port': 3883},
                {'update_freq': 33.0},
                {'frame_id': 'map'},
                {'refresh_tracker_frequency': 1.0}
            ]
        ),
        Node(
            namespace="ds4_driver",
            package="ds4_driver",
            executable="ds4_driver_node.py",
            parameters=[
                {'use_standard_msgs': True}
            ]
        )
    ])
