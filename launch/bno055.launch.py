def generate_launch_description():
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    param_file = PathJoinSubstitution([
        FindPackageShare('bno055_cpp'),
        'config',
        'bno055_params.yaml'
    ])
    return LaunchDescription([
        Node(
            package='bno055_cpp',
            executable='bno055',
            name='bno055',
            output='screen',
            parameters=[param_file],
        )
    ])
