import rclpy
import time
import sys
from rclpy.node import Node
from mecharm_interfaces.srv import ArmLinearControl


class ArmLinearControlClient(Node):
    def __init__(self):
        super().__init__('arm_linear_control_client')

    def send_request(self, params):
        self.arm_linear_control_client = self.create_client(ArmLinearControl, 'arm_linear_control')

        # Wait until the service is available
        while not self.arm_linear_control_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the arm_linear_control service...')

        self.get_logger().info('arm_linear_control service is available.')


        request = ArmLinearControl.Request()
        request.pick_point_x = float(params[0]) 
        # Call the service
        future = self.arm_linear_control_client.call_async(request)
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
    if len(sys.argv) != 2:  # search direction + script name
        print("Usage: ros2 run <your_package> arm_set_pose_client x y z rx ry rz")
        sys.exit(1)

    client_node = ArmLinearControlClient()
    try:
        client_node.send_request(sys.argv[1:])
    except KeyboardInterrupt:
        pass
    finally:
        client_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
