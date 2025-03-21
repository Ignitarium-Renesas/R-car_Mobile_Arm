import rclpy
import time
from rclpy.node import Node
import sys

from mecharm_interfaces.srv import ArmMoveToPick


class ArmSetPickPoseClient(Node):
    def __init__(self):
        super().__init__('arm_set_pick_pose_client')

    def send_request(self, base_pose):
        self.arm_set_pick_pose_client = self.create_client(ArmMoveToPick, 'set_pick_pose')

        # Wait until the service is available
        while not self.arm_set_pick_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the set_pose service...')

        self.get_logger().info('set_pose service is available.')


        request = ArmMoveToPick.Request()
        request.base_pose.x = float(base_pose[0])
        request.base_pose.y = float(base_pose[1])
        request.base_pose.z = float(base_pose[2])
        request.base_pose.rx = float(base_pose[3])
        request.base_pose.ry = float(base_pose[4])
        request.base_pose.rz = float(base_pose[5])

        self.get_logger().info(f"Input Pose: {request.base_pose}")
        # Call the service
        future = self.arm_set_pick_pose_client.call_async(request)
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
    # if len(sys.argv) != :  # 6 numbers + script name
    #     print("Usage: ros2 run <your_package> arm_set_pose_client x y z rx ry rz")
    #     sys.exit(1)

    client_node = ArmSetPickPoseClient()
    try:
        client_node.send_request(sys.argv[1:])
    except KeyboardInterrupt:
        pass
    finally:
        client_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
