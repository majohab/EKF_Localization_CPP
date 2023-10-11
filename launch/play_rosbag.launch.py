from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

package_name = "ekf_localization"
node_name = 'localization_cpp'

def generate_launch_description():
    return LaunchDescription([
        Node(
            package=package_name,
            executable='entry',
            name=node_name,
            remappings=[
            ('/navigation/slam/state_estimation', '/navigation/localization_cpp/state_estimation'),
            ('/navigation/slam/conearraystamped', '/navigation/localization_cpp/conearraystamped'),
            ],
            parameters=[{"default": get_package_share_directory(package_name) + "/default.yaml"}],
            arguments=['--ros-args', '--log-level', node_name+":=debug"]
        )
    ])