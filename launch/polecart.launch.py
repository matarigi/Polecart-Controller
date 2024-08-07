from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('gazebo_polecart_ros'), '/launch/polecart.launch.py']
            ),
        ),
        Node(
            package='polecart_controller',
            namespace='polecart_controller',
            executable='polecart_controller'
        )
    ])