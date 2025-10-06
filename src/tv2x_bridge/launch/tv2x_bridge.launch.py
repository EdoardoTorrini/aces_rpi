from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import os

def generate_launch_description():

    # os.system('~/mmr-drive/useful_scripts/connect_can.sh')

    config = os.path.join(
        get_package_share_directory("tv2x_bridge"),
        "config",
        "config.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package='tv2x_bridge',
                executable='tv2x_bridge',
                name='tv2x_bridge',
		        output='screen',
                parameters=[config]
            )
        ]
    )
