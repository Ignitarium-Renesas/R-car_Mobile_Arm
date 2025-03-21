import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from mecharm_interfaces.msg import (MecharmAngles,
                                    MecharmCoords,
                                    MecharmSetAngles,
                                    MecharmSetCoords,
                                    MecharmGripperStatus,
                                    MecharmPumpStatus,
                                    MecharmErrorStatus
                                    )
import cv2
import numpy as np

class Point3DClicker(Node):
    def __init__(self):
        super().__init__('pose_estimation')
        
        # ROS 2 subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_rect_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)
        self.camera_pose_pub = self.create_publisher(msg_type=MecharmCoords,
                                                     topic="camera_pose",
                                                     qos_profile=5
                                                     )

        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.intrinsics = None
        self.radius = 3  # Radius in pixels for averaging
        self.pose = MecharmCoords()
        self.pose.x = 0.0
        self.pose.y = 0.0
        self.pose.z = 0.0
        self.pose.rx = 0.01
        self.pose.ry = -45.0
        self.pose.rz = 90.0

    def camera_info_callback(self, msg):
        # Retrieve camera intrinsic parameters
        self.intrinsics = {
            'fx': msg.k[0],
            'fy': msg.k[4],
            'ppx': msg.k[2],
            'ppy': msg.k[5]
        }

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.camera_pose_pub.publish(self.pose)
        # Display the RGB image
        if self.color_image is not None:
            cv2.imshow("RGB Image", self.color_image)
            cv2.setMouseCallback("RGB Image", self.mouse_callback)
            cv2.waitKey(1)

    def depth_callback(self, msg):
        # Convert ROS Image message to OpenCV depth image
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.depth_image is not None and self.intrinsics:
            # Draw a circle around the clicked point
            display_image = self.color_image.copy()
            cv2.circle(display_image, (x, y), self.radius, (0, 255, 0), 2)
            cv2.imshow("RGB Image", display_image)

            # Get the average 3D position within the circle of the specified radius
            points_3d = []

            # Define a circular mask to get pixels within the radius around the clicked point
            for dx in range(-self.radius, self.radius + 1):
                for dy in range(-self.radius, self.radius + 1):
                    dist = np.sqrt(dx**2 + dy**2)
                    if dist <= self.radius:
                        px, py = x + dx, y + dy
                        if 0 <= px < self.depth_image.shape[1] and 0 <= py < self.depth_image.shape[0]:
                            depth_value = self.depth_image[py, px] * 0.001  # Convert mm to meters
                            if depth_value > 0:  # Check for valid depth
                                # Compute 3D coordinates for the pixel within the radius
                                X = (px - self.intrinsics['ppx']) * depth_value / self.intrinsics['fx']
                                Y = (py - self.intrinsics['ppy']) * depth_value / self.intrinsics['fy']
                                Z = depth_value
                                points_3d.append((X, Y, Z))
            
            # Calculate the average of 3D points within the radius
            if points_3d:
                avg_x = np.mean([p[0] for p in points_3d])
                avg_y = np.mean([p[1] for p in points_3d])
                avg_z = np.mean([p[2] for p in points_3d])
                print("Clicked 2D coordinates:", (x, y))
                print("Average 3D Position with respect to camera:", (avg_x, avg_y, avg_z))
                
                self.pose.x = avg_x
                self.pose.y = avg_y
                self.pose.z = avg_z
                
                
            else:
                print("No valid depth data in the selected area.")
        

def main(args=None):
    rclpy.init(args=args)
    node = Point3DClicker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
