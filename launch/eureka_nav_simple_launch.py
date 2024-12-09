import os, yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_dir = get_package_share_directory('eureka_nav_simple')

    nav_params_path = os.path.join(pkg_dir, 'config', 'nav.yaml')

    with open(nav_params_path, 'r') as file:
        nav_params = yaml.safe_load(file)

    navcam_nodes = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([os.path.join(
                            get_package_share_directory('eureka_camera_2'), 'launch', 'eureka_navcam_simple_launch.py')
                        ])
    )

    nav_node = Node(
        package='eureka_nav_simple',
        executable='nav_simple_cv',
        name='nav_simple_cv',
        output='screen',
        parameters=[nav_params_path, nav_params]
    )

    ld = LaunchDescription()
    ld.add_action(navcam_nodes)
    ld.add_action(nav_node)