import rclpy
from rclpy.node import Node
from mecharm_interfaces.srv import GoToGoal

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self.client = self.create_client(GoToGoal, 'send_navigation_goal')

    def send_goal(self, x, y, yaw):
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for navigation service...')

        request = GoToGoal.Request()
        request.x = x
        request.y = y
        request.yaw = yaw

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        self.get_logger().info(f'Service Response: {response.message}')

def main():
    rclpy.init()
    client = NavigationClient()
    client.send_goal(1.0, 0.0, 0.0)  # Example: move to (2,3) with a 90-degree orientation
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
