from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():

    robot_interface_node = Node(
        package='openmanipulator_task_executor',
        executable='robot_interface',
        output='screen'
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
