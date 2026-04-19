from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    yahboomcar_package_path = get_package_share_directory('yahboomcar_bringup')
    
    yahboomcar_brinup_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(yahboomcar_package_path, 'launch'),
         '/yahboomcar_bringup_X3_launch.py'])
    )
    lidar_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sllidar_ros2'), 
                'launch', 
                'sllidar_launch.py'
            )
        )
    )
    
    return LaunchDescription([
        yahboomcar_brinup_launch,
        lidar_bringup_launch,
        Node(
            package='object_detection',
            executable='lidar_obj_detection',
            name='lidar_obj_detection',
            output='screen',
        ),
    ])
