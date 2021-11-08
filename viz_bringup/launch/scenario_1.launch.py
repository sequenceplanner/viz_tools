import os
import sys
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node

def generate_launch_description():
    scenario = "scenario_1"
    tf_scene_dir = FindPackageShare("tf_scene").find("tf_scene")
    viz_bringup_dir = FindPackageShare("viz_bringup").find("viz_bringup")

    parameters = {
        "scenario_path": os.path.join(tf_scene_dir, "scenarios", scenario), 
        "meshes_path": "/home/endre/Desktop/ia_ros_meshes"
    }

    rviz_config_file = os.path.join(viz_bringup_dir, "config", f"{scenario}.rviz")
    
    tf_lookup_node = Node(
        package="tf_lookup",
        executable="tf_lookup",
        namespace="",
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True
    )

    tf_broadcast_node = Node(
        package="tf_broadcast",
        executable="tf_broadcast",
        namespace="",
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True
    )

    tf_sms_node = Node(
        package="tf_sms",
        executable="tf_sms",
        namespace="",
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True
    )

    viz_static_node = Node(
        package="viz_static",
        executable="viz_static",
        namespace="",
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True
    )

    viz_interactive_node = Node(
        package="viz_interactive",
        executable="viz_interactive",
        namespace="",
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace="",
        output="screen",
        arguments=["-d", rviz_config_file],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True
    )

    nodes_to_start = [
        tf_lookup_node,
        tf_broadcast_node,
        tf_sms_node,
        viz_static_node,
        viz_interactive_node,
        rviz_node
    ]

    return LaunchDescription(nodes_to_start)

if __name__ == '__main__':
    ls = LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())
