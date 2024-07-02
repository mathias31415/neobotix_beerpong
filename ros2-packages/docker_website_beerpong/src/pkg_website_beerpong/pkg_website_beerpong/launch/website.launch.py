from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pkg_website_beerpong',  # Ersetze 'mein_paket' durch den Namen deines ROS 2 Pakets
            executable='app',  # Der Name des ausführbaren Skripts ohne die .py Erweiterung
            output='screen'
        ),
    ])
