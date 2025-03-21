import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from mecharm_interfaces.srv import GoToGoal # Custom service definition

class NavigationService(Node):
    def __init__(self):
        super().__init__('navigation_service')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.srv = self.create_service(GoToGoal, 'send_navigation_goal', self.send_goal_callback)

    def send_goal_callback(self, request, response):
        x, y, yaw = request.x, request.y, request.yaw  # Extract target coordinates and orientation
        self.get_logger().info(f'Received navigation request to: x={x}, y={y}, yaw={yaw}')
        
        target_pose = self.create_pose(x, y, 0.0, yaw)
        
        if self.send_goal(target_pose):
            response.success = True
            response.message = "Navigation goal reached successfully."
        else:
            response.success = False
            response.message = "Failed to reach navigation goal."
        
        return response

    def send_goal(self, target_pose):
        self.get_logger().info(f'Sending goal: x={target_pose.pose.position.x}, y={target_pose.pose.position.y}, yaw={target_pose.pose.orientation.z}')
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected!")
            return False

        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result and result.status == 4:
            self.get_logger().info("Goal reached successfully!")
            return True
        else:
            self.get_logger().error("Failed to reach goal!")
            return False

    def create_pose(self, x, y, z, yaw):
        from tf_transformations import quaternion_from_euler
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        
        return pose

def main(args=None):
    rclpy.init(args=args)
    navigation_service = NavigationService()
    rclpy.spin(navigation_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
