import rclpy
import time
from rclpy.node import Node
from mecharm_interfaces.srv import CapturePose


class PoseClient(Node):
    def __init__(self):
        super().__init__('pose_est_client')

    def send_request(self):
        self.capture_pose_client = self.create_client(CapturePose, 'capture_pose')

        # Wait until the service is available
        while not self.capture_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the capture_pose service...')

        self.get_logger().info('capture_pose service is available.')

        for i in range(1):  # Call service 10 times
            self.get_logger().info(f"Calling capture_pose service - Attempt {i + 1}")

            request = CapturePose.Request()

            self.get_logger().info(f"request init")

            # Call the service
            self.get_logger().info(f"Call service")
            future = self.capture_pose_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            self.get_logger().info(f"Spinning is done")

            if future.result() is not None:
                response = future.result()
                if response.pose.has_detection:
                    pose = response.pose.pose
                    self.get_logger().info(f"Detected Pose: x={pose.x}, y={pose.y}, z={pose.z}")
                else:
                    self.get_logger().info("No object detected.")
            else:
                self.get_logger().error("Service call failed.")

            time.sleep(1)  # Delay of 1 seconds

def main(args=None):
    rclpy.init(args=args)
    client_node = PoseClient()

    try:
        client_node.send_request()
    except KeyboardInterrupt:
        pass
    finally:
        client_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
