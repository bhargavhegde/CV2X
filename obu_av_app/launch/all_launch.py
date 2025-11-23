from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Include Novatel OEM7 launch file
    novatel_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('novatel_oem7_driver'),
                'launch',
                'oem7_net.launch.py'
            )
        ),
        launch_arguments={
            'oem7_ip_addr': '192.168.100.201',
            'oem7_port': '3005'
        }.items()
    )

    # Include ds_dbw_can dbw.launch.xml
    dbw_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ds_dbw_can'),
                'launch',
                'dbw.launch.xml'
            )
        )
    )

    # Mako camera node
    mako_camera = Node(
        package='vimbax_camera',
        executable='vimbax_camera_node',
        name='mako_camera',
        output='screen',
        parameters=[{'use_ros_time': True}]
    )

    # Paths to the on-vehicle, camera, and detection node scripts (same folder)
    on_vehicle_path = os.path.join('/ws/cv2x/obu_av_app', 'on_vehicle_node.py')
    camera_node_path = os.path.join('/ws/cv2x/obu_av_app', 'camera_node.py')
    detection_node_path = os.path.join('/ws/cv2x/obu_av_app', 'detection_node.py')  

    # on_vehicle node execution
    on_vehicle_node = ExecuteProcess(
        cmd=['python3', on_vehicle_path],
        output='screen'
    )

    # Camera node execution with a 2-second delay using TimerAction
    camera_node = TimerAction(
        period=2.0,  # Delay of 2 seconds
        actions=[
            ExecuteProcess(
                cmd=['python3', camera_node_path],
                output='screen',
            )
        ]
    )

    # Detection node execution
    detection_node = TimerAction(
        period=1.0,  
        actions=[
            ExecuteProcess(
                cmd=['python3', detection_node_path],
                output='screen'
            )
        ]
    )

    
    return LaunchDescription([
        novatel_launch,
        dbw_launch, 
        mako_camera,
        on_vehicle_node,
        camera_node,
        detection_node  
    ])
