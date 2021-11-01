import os
import rclpy
import json
import time
import random
import datetime

from rclpy.node import Node
from viz_tools_msgs.srv import VisualizeArray

from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from tf_tools_msgs.srv import ManipulateBroadcast
from std_msgs.msg import ColorRGBA

class ActiveVisualization(Node):
    def __init__(self):
        self.node_name = "active_visualization"
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

        self.static_items = [
            x for x in self.static_items_jsons if x["visualization"]["show_mesh"]
        ]

        self.get_logger().info(
            f"added {str(len(self.static_items))} non-interactive items"
        )

        self.service = self.create_service(
            VisualizeArray, "active_visualization_service", self.viz_callback
        )

        self.marker_array_publisher = self.create_publisher(
            MarkerArray, "active_markers", 10
        )

        self.individual_markers = []
        self.marker_ids = []

        self.color = ColorRGBA()

        self.marker_array_msg = MarkerArray()

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

    def viz_callback(self, request, response):
        markers = []
        requested_markers = request.array
        for i in requested_markers:
            if i.command == "remove":
                for marker in self.individual_markers:
                    if marker["frame"] == i.frame:
                        self.marker_ids.remove(marker["id"])
                        self.individual_markers.remove(marker)
                        current_markers = self.marker_array_msg.markers
                        for mar in current_markers:
                            if mar.header.frame_id == i.frame:
                                current_markers.remove(mar)
                                self.marker_array_msg.markers = current_markers
                                self.get_logger().info(
                                    f"Removed active marker {i.name} in frame {i.frame}"
                                )
            elif i.command == "add":
                if any(i.frame == marker["frame"] for marker in self.individual_markers):
                    pass
                else:
                    with open(
                        os.path.join(
                            self.parameters["scenario_path"], i.name + ".json"
                        )
                    ) as jsonfile:
                        thing_parameters = json.load(jsonfile)
                    id = (
                        random.choice(list(set(range(0, 100)) - set(self.marker_ids)))
                        if not self.marker_ids
                        else random.choice(range(0, 100))
                    )
                    self.marker_ids.append(id)
                    self.individual_markers.append(
                        {"name": i.name, "frame": i.frame, "id": id}
                    )
                    indiv_marker = Marker()
                    indiv_marker.header.frame_id = i.frame
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
                    indiv_marker.scale.x = thing_parameters["visualization"]["scale_x"]
                    indiv_marker.scale.y = thing_parameters["visualization"]["scale_y"]
                    indiv_marker.scale.z = thing_parameters["visualization"]["scale_z"]
                    indiv_marker.color.a = thing_parameters["visualization"]["color_a"]
                    indiv_marker.color.r = thing_parameters["visualization"]["color_r"]
                    indiv_marker.color.g = thing_parameters["visualization"]["color_g"]
                    indiv_marker.color.b = thing_parameters["visualization"]["color_b"]
                    indiv_marker.mesh_resource = "file://" + os.path.join(
                        self.parameters["meshes_path"],
                        thing_parameters["visualization"]["mesh"],
                    )
                    markers.append(indiv_marker)
                    self.marker_array_msg.markers = markers
                    self.get_logger().info(
                        f"Added active marker {i.name} in frame {i.frame}"
                    )
            else:
                self.get_logger().info(
                    f"Unknown command {i.command}"
                )
        response.result = True
        return response

    def publish_markers(self):
        self.marker_array_publisher.publish(self.marker_array_msg)


def main(args=None):
    rclpy.init(args=args)

    viz = ActiveVisualization()
    rclpy.spin(viz)
    viz.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
