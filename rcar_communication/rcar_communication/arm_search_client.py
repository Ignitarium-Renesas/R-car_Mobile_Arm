import rclpy
import time
from rclpy.node import Node
import sys

from mecharm_interfaces.srv import SearchObject


class ArmSearchObjectClient(Node):
    def __init__(self):
        super().__init__('arm_search_object_client')

    def send_request(self, params):
        self.arm_search_object_client = self.create_client(SearchObject, 'arm_search_object')

        # Wait until the service is available
        while not self.arm_search_object_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the set_pose service...')

        self.get_logger().info('set_pose service is available.')

        request = SearchObject.Request()
        request.search_direction = str(params[0])
        request.search_offset = float(params[1])

        self.get_logger().info(f"Input Pose: {request.search_direction}")
        # Call the service
        future = self.arm_search_object_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            # if response.pose.has_detection:
            # pose = response.pose.pose
            self.get_logger().info(f"Response: {response}")
            # else:
            #     self.get_logger().info("No object detected.")
        else:
            self.get_logger().error("Service call failed.")
        time.sleep(1)  # Delay of 1 seconds


def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 3:  # search direction + script name
        print("Usage: ros2 run <your_package> arm_set_pose_client x y z rx ry rz")
        sys.exit(1)

    client_node = ArmSearchObjectClient()
    try:
        client_node.send_request(sys.argv[1:])
    except KeyboardInterrupt:
        pass
    finally:
        client_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
