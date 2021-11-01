import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from viz_tools_msgs.msg import Visualize
from viz_tools_msgs.srv import VisualizeArray


class ActiveVizClientNode(Node):
    def __init__(self):
        super().__init__("viz_active_client")

        self.transform = TransformStamped()

        self.client = self.create_client(VisualizeArray, "active_visualization_service")
        self.request = VisualizeArray.Request()
        self.response = VisualizeArray.Response()

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Active viz service not available, waiting again...")

        time.sleep(5)
        self.send_add_request()
        time.sleep(5)
        self.send_remove_request()

    def send_add_request(self):
        c1 = Visualize()
        c1.command = "add"
        c1.name = "cube"
        c1.frame = "world"
        c2 = Visualize()
        c2.command = "add"
        c2.name = "cube"
        c2.frame = "frame_4"
        self.request.array = [c1, c2]
        self.future = self.client.call_async(self.request)
        self.get_logger().info(f"request sent: {self.request}")
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future.done():
                try:
                    response = self.future.result()
                except Exception as e:
                    self.get_logger().error(f"service call failed with: {(e,)}")
                else:
                    self.response = response
                    self.get_logger().info(f"lookup result: {self.response}")
                finally:
                    self.get_logger().info(f"service call completed")
                break

    def send_remove_request(self):
        c1 = Visualize()
        c1.command = "remove"
        c1.name = "cube"
        c1.frame = "world"
        self.request.array = [c1]
        self.future = self.client.call_async(self.request)
        self.get_logger().info(f"request sent: {self.request}")
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future.done():
                try:
                    response = self.future.result()
                except Exception as e:
                    self.get_logger().error(f"service call failed with: {(e,)}")
                else:
                    self.response = response
                    self.get_logger().info(f"lookup result: {self.response}")
                finally:
                    self.get_logger().info(f"service call completed")
                break


def main(args=None):
    rclpy.init(args=args)

    node = ActiveVizClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
