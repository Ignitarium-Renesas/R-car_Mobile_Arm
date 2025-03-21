import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from mecharm_interfaces.srv import SetInitialPose
import tf_transformations as tf

class SetInitialPoseService(Node):
    def __init__(self):
        super().__init__('set_initial_pose_service')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.srv = self.create_service(SetInitialPose, 'set_initial_pose', self.set_initial_pose_callback)
        self.get_logger().info("Initial pose service ready.")
        self.first_run=True

    def set_initial_pose_callback(self, request, response):
        if self.first_run:
            self.get_logger().info(f"status : {request.pose_set}")
            self.get_logger().info("Setting initial pose...")

            # Use provided values or default to 0
            x = request.x if request.x is not None else 0.0
            y = request.y if request.y is not None else 0.0
            z = request.z if request.z is not None else 0.0
            roll = request.roll if request.roll is not None else 0.0
            pitch = request.pitch if request.pitch is not None else 0.0
            yaw = request.yaw if request.yaw is not None else 0.0

            # Create pose message
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.pose.position.x = x
            pose_msg.pose.pose.position.y = y
            pose_msg.pose.pose.position.z = z

            # Convert roll, pitch, yaw to quaternion
            q = tf.quaternion_from_euler(roll, pitch, yaw)
            pose_msg.pose.pose.orientation.x = q[0]
            pose_msg.pose.pose.orientation.y = q[1]
            pose_msg.pose.pose.orientation.z = q[2]
            pose_msg.pose.pose.orientation.w = q[3]

            # Publish pose
            for i in range(20):
                self.publisher_.publish(pose_msg)
            self.get_logger().info("Initial pose set successfully.")

            response.success = True
            response.message = "Initial pose set successfully."
            self.first_run=False
        else:
            self.get_logger().info(f"status : {request.pose_set}")
            # Do not destroy the publisher, just skip publishing
            self.get_logger().info("Skipping initial pose setting.")
            response.success = True
            response.message = "Initial pose was not set."

        return response


def main(args=None):
    rclpy.init(args=args)
    node = SetInitialPoseService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
