import os
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
import launch.events
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    pkg_dir = os.path.split(os.path.realpath(__file__))[0] + '/..'
    current_config = os.path.join(pkg_dir, 'config', 'current.yaml')
    default_config = os.path.join(pkg_dir, 'config', 'default.yaml')

    config_path = current_config if os.path.exists(current_config) else default_config

    livox_driver = LifecycleNode(
        package='livox_ros2_driver',
        executable='livox_ros2_driver_node',
        name='livox_lidar_publisher',
        namespace='',
        output='screen',
        parameters=[config_path],
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(livox_driver),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=livox_driver,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(livox_driver),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    return LaunchDescription([
        activate_handler,
        livox_driver,
        configure_event,
    ])
