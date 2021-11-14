import os
import rclpy
import json
import time

from rclpy.node import Node

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from tf_tools_msgs.srv import ManipulateBroadcast
from std_msgs.msg import ColorRGBA


class StaticVisualization(Node):
    def __init__(self):
        self.node_name = "static_visualization"
        super().__init__(self.node_name)

        self.client = self.create_client(ManipulateBroadcast, "manipulate_broadcast")

        self.parameter_keys = ["scenario_path", "meshes_path"]

        self.declared_parameters = [
            self.declare_parameter(x, "default_value") for x in self.parameter_keys
        ]

        self.parameters = {
            x: self.get_parameter(x).get_parameter_value().string_value
            for x in self.parameter_keys
        }


        while rclpy.ok():
            try:
                frames_list = os.listdir(self.parameters["scenario_path"])
            except Exception as e:
                self.get_logger().error(f"Failed to list directory '{self.parameters['scenario_path']}'")
                time.sleep(1)
            else:
                self.get_logger().info(f"Listed directory '{self.parameters['scenario_path']}'")
                break

        self.included_things_paths = []
        if len(frames_list) == 0:
            self.get_logger().error(
                f"listed directory '{self.parameters['scenario_path']}' is empty"
            )
        else:
            for thing in frames_list:
                while rclpy.ok():
                    try:
                        with open(os.path.join(self.parameters['scenario_path'], thing)) as jsonfile:
                            thing_parameters = json.load(jsonfile)
                    except Exception as e:
                        self.get_logger().error(f"failed to open '{thing}'.")
                        time.sleep(1)
                    else:
                        self.get_logger().info(f"opened '{thing}'.")
                        break
                if thing_parameters["show"]:
                    self.included_things_paths.append(os.path.join(self.parameters['scenario_path'], thing))

        self.static_items_jsons = [
            json.load(open(x)) for x in self.included_things_paths
        ]

        # self.static_items = []
        # for item in self.static_items_jsons:
        #     if "viausliation" in item
        self.static_items = [
            x for x in self.static_items_jsons if "visualization" in x if x["visualization"]["show_mesh"]
        ]

        self.get_logger().info(
            f"added {str(len(self.static_items))} non-interactive items"
        )

        self.marker_array_publisher = self.create_publisher(
            MarkerArray, "static_markers", 10
        )

        self.individual_markers = [x for x in self.static_items]

        self.color = ColorRGBA()

        self.marker_timer_period = 0.01
        self.marker_timer = self.create_timer(
            self.marker_timer_period, self.publish_markers
        )

        # wait for the broadcaster to add the frames
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "tf broadcast service not available, waiting again..."
            )

        # wait a bit for the tf to put these frames in the buffer
        time.sleep(2)

    def publish_markers(self):
        msg = MarkerArray()
        markers = []
        id = 0
        for x in self.individual_markers:
            id = id + 1
            indiv_marker = Marker()
            indiv_marker.header.frame_id = x["child_frame"]
            indiv_marker.header.stamp = Time()
            indiv_marker.ns = ""
            indiv_marker.id = id
            indiv_marker.type = 10
            indiv_marker.action = 0
            indiv_marker.pose.position.x = 0.0
            indiv_marker.pose.position.y = 0.0
            indiv_marker.pose.position.z = 0.0
            indiv_marker.pose.orientation.x = 0.0
            indiv_marker.pose.orientation.y = 0.0
            indiv_marker.pose.orientation.z = 0.0
            indiv_marker.pose.orientation.w = 1.0
            indiv_marker.scale.x = x["visualization"]["scale_x"]
            indiv_marker.scale.y = x["visualization"]["scale_y"]
            indiv_marker.scale.z = x["visualization"]["scale_z"]
            indiv_marker.color.a = x["visualization"]["color_a"]
            indiv_marker.color.r = x["visualization"]["color_r"]
            indiv_marker.color.g = x["visualization"]["color_g"]
            indiv_marker.color.b = x["visualization"]["color_b"]
            indiv_marker.mesh_resource = "file://" + os.path.join(
                self.parameters["meshes_path"],
                x["visualization"]["mesh"],
            )
            markers.append(indiv_marker)
            msg.markers = markers
        self.marker_array_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    viz = StaticVisualization()
    rclpy.spin(viz)
    viz.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
