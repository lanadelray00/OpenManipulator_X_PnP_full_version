import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import yaml
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():

    # kinematics yaml
    kinematics_yaml_path = os.path.join(
        get_package_share_directory('open_manipulator_moveit_config'),
        'config',
        'kinematics.yaml',
    )
    with open(kinematics_yaml_path, 'r') as file:
        kinematics_yaml = yaml.safe_load(file)

    robot_interface_node = Node(
        package='openmanipulator_task_executor',
        executable='robot_interface',
        output='screen',
        parameters=[
            {'robot_description_kinematics': kinematics_yaml}
            ]
    )

    pick_and_place_node = Node(
        package='openmanipulator_task_executor',
        executable='open_manipulator_x_pickandplace.py',
        output='screen'
    )

    # robot_interface가 뜬 다음 pick_and_place 실행
    start_pick_and_place_after_robot = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_interface_node,
            on_start=[pick_and_place_node]
        )
    )

    return LaunchDescription([
        robot_interface_node,
        start_pick_and_place_after_robot
    ])
