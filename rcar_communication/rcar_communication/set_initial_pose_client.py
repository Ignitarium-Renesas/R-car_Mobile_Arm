import rclpy
from rclpy.node import Node
from mecharm_interfaces.srv import SetInitialPose

class SetInitialPoseClient(Node):
    def __init__(self):
        super().__init__('set_initial_pose_client')
        self.client = self.create_client(SetInitialPose, 'set_initial_pose')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.get_logger().info('Service available. Ready to send request.')

    def send_request(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        request = SetInitialPose.Request()
        request.x = x
        request.y = y
        request.z = z
        request.roll = roll
        request.pitch = pitch
        request.yaw = yaw

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    client = SetInitialPoseClient()
    response = client.send_request(1.0, 2.0, 0.0, 0.0, 0.0, 1.57)
    print(f"Response: {response.message}")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
