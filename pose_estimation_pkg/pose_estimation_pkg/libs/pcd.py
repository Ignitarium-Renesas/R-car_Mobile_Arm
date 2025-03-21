from pose_estimation_pkg.libs.utils.crop_utils import get_pcd_crop
from pose_estimation_pkg.libs.utils.display_utils import get_random_colors


import open3d as o3d
import json
import numpy as np
import cv2
import pyrealsense2 as rs
import copy


class PointCloud:
    def __init__(self,camera_intrinsic,depth_scale, clipping_distance_in_meters, display):
        self.depth_scale = depth_scale
        self.clipping_distance_in_meters = clipping_distance_in_meters
        self.display = display
        # self.intrinsic = [386.62982177734375, 386.1138916015625, 323.0791931152344, 243.10650634765625] # 52cm camera
        self.intrinsic = camera_intrinsic
        self.flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
        # self.depth_scale = 1000 #default value in open3den3d
        if self.display:
            self.vis_pcd = o3d.geometry.PointCloud()

            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window()
            self.frame_count = 0


    def get_point_cloud(self,color_frame, depth_frame):

        h,w,_ = color_frame.shape
        #convert to open3d geometry
        image_3d = o3d.geometry.Image(color_frame)
        depth_3d = o3d.geometry.Image(depth_frame)


        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                image_3d, depth_3d,
                depth_scale=1.0 / self.depth_scale,
                depth_trunc=self.clipping_distance_in_meters,
                convert_rgb_to_intensity=False,
            )

        pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
                w, h, self.intrinsic[0], self.intrinsic[1], self.intrinsic[2], self.intrinsic[3]
            )
        target_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image, pinhole_camera_intrinsic
            )
        # target_pcd.transform(self.flip_transform)
          
        return target_pcd


    def crop_pcd(self,roi,image_data,depth_data,dof_6d=False,show_flag=False,debug=False):
        if dof_6d:
            pcd = self.get_pcd_crop(
                        roi,
                        image_data,
                        depth_data,
                        self.intrinsic,
                        show_flag=show_flag,
                        debug=debug,
                    )
        else:
            pcd = self.get_pcd_box_crop(rgb_img=image_data, 
                                    depth_image=depth_data,
                                    bbox=roi)
        return pcd
    

    # def get_pcd_crop(self,contour, rgb_img, depth_img, intrinsic_mat, save_path=None, show_flag=False,debug=False):
    #     img_height, img_width, _ = rgb_img.shape
    #     mask = np.zeros_like(rgb_img)
    #     img_mask = cv2.fillPoly(mask, [np.array(contour)], [255, 255, 255])

    #     img_mask = mask[:, :, 0]
    #     depth_mask = img_mask / 255
    #     depth_mask = depth_mask.astype("uint16")
    #     img_crop = cv2.bitwise_and(rgb_img, rgb_img, mask = img_mask)
    #     depth_crop = np.multiply(depth_img, depth_mask)
        
    #     if show_flag:
    #         self.visualize_depth_image(depth_img.copy())
    #     if show_flag:
    #         self.visualize_depth_image(depth_crop.copy())
    #         cv2.imshow("color image", img_crop)
    #         cv2.waitKey(0)
    #     image_3d = o3d.geometry.Image(np.ascontiguousarray(img_crop))
    #     depth_3d = o3d.geometry.Image(np.ascontiguousarray(depth_crop))
    #     rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(image_3d, depth_3d,
    #                                                                     # depth_scale=1.0 / self.depth_scale,
    #                                                                     # depth_trunc=self.clipping_distance_in_meters,
    #                                                                     # convert_rgb_to_intensity=False,)
    #     )
    #     pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
    #             img_width, img_height, self.intrinsic[0], self.intrinsic[1], self.intrinsic[2], self.intrinsic[3]
    #         )
    #     target_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,
    #                                 pinhole_camera_intrinsic)
    #     if show_flag:
    #         o3d.visualization.draw_geometries([target_pcd])
    #     if save_path is not None:
    #         o3d.io.write_point_cloud(save_path, target_pcd)

    #     return target_pcd


    def get_pcd_crop(self,contour, rgb_img, depth_img ,intrinsic_mat=None, save_path=None, show_flag=False,debug=False):
        o3d.utility.random.seed(42)
        image = rgb_img.copy()
        img_height, img_width, _ = rgb_img.shape
        mask = np.zeros_like(rgb_img)
        # breakpoint()
        img_mask = cv2.fillPoly(mask, [np.array(contour)], [255, 255, 255])
        img_mask = mask[:, :, 0]
        
        depth_mask = img_mask / 255
        depth_mask = depth_mask.astype("uint16")
        img_crop = cv2.bitwise_and(rgb_img, rgb_img, mask = img_mask)
        depth_crop = np.multiply(depth_img, depth_mask)
        if show_flag:
            self.visualize_depth_image(depth_img.copy())
        if show_flag:
            self.visualize_depth_image(depth_crop.copy())
            cv2.imshow("color image", img_crop)
            cv2.waitKey(0)
        image_3d = o3d.geometry.Image(np.ascontiguousarray(img_crop))
        depth_3d = o3d.geometry.Image(np.ascontiguousarray(depth_crop))

        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(image_3d, depth_3d,
                                                                        # depth_scale=1.0 / self.depth_scale,
                                                                        # depth_trunc=self.clipping_distance_in_meters,
                                                                        # convert_rgb_to_intensity=False,)
        )
        pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
                img_width, img_height, self.intrinsic[0], self.intrinsic[1], self.intrinsic[2], self.intrinsic[3]
            )
        target_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,
                                    pinhole_camera_intrinsic)
        if show_flag:
            o3d.visualization.draw_geometries([target_pcd])
        if save_path is not None:
            o3d.io.write_point_cloud(save_path, target_pcd)


        cv2.imwrite("z_ref_image.jpg", image)
        # print("[INFO] write image as JPEG :", save_paths[0])
        o3d.io.write_point_cloud("z_ref_pcd.ply", target_pcd)
        # print("[INFO] Pointcloud saved as PLY:", save_paths[1])

        return target_pcd
    

    def get_pcd_box_crop(self,rgb_img, depth_image, bbox):
        img_height, img_width, _ = rgb_img.shape
        x1, y1, x2,y2 = bbox
        img_canvas = np.zeros_like(rgb_img)
        rgb_roi = rgb_img[y1:y2, x1:x2]
        img_canvas[y1:y2, x1:x2] = rgb_roi

        depth_canvas = np.zeros_like(depth_image)
        depth_roi = depth_image[y1:y2, x1:x2]
        depth_canvas[y1:y2, x1:x2] = depth_roi


        img_canvas = o3d.geometry.Image(np.ascontiguousarray(img_canvas))
        depth_canvas = o3d.geometry.Image(np.ascontiguousarray(depth_canvas))

        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(img_canvas, depth_canvas,
                                                                        depth_scale=1.0 / self.depth_scale,
                                                                        depth_trunc=self.clipping_distance_in_meters,
                                                                        convert_rgb_to_intensity=False,)
        # )
        pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
                img_width, img_height, self.intrinsic[0], self.intrinsic[1], self.intrinsic[2], self.intrinsic[3]
            )
        target_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,
                                    pinhole_camera_intrinsic)
        
        return target_pcd



    def visualize_depth_image(self,depth_image):

        # Normalize the depth image to the range 0-255 for visualization
        normalized_depth = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        normalized_depth = np.uint8(normalized_depth)  # Convert to 8-bit image

        # Apply a colormap (e.g., COLORMAP_JET)
        colored_depth = cv2.applyColorMap(normalized_depth, cv2.COLORMAP_JET)
        # Display the depth image
        cv2.imshow('Normalized Depth Image', normalized_depth)
        cv2.imshow('Colored Depth Image', colored_depth)

        # Wait and close windows
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    

    def mesh2pcd(self, mesh_path):
        # o3d.utility.random.seed(42)
        np.random.seed(42)
        mesh_pcd = o3d.io.read_triangle_mesh(mesh_path)
        mesh_pcd = mesh_pcd.sample_points_poisson_disk(5000)
        mesh_pcd.translate(-mesh_pcd.get_center())
        mesh_pcd.scale(1.0/self.depth_scale, center=mesh_pcd.get_center())

        return mesh_pcd
    
    
    def filter_point_cloud(self,pcd,display):
        pcd_copy = copy.deepcopy(pcd)
        points = np.asarray(pcd_copy.points)
        # Take only the points less than 
        thresholded_points = points[points[:, 2] <= 0.90 ]
        thresholded_points = thresholded_points[thresholded_points[:, 2] >= 0.05 ]
                
        median = np.median(thresholded_points, axis=0)
        print("Centroid point",median)


        thresh_pcd = o3d.geometry.PointCloud()
        thresh_pcd.points = o3d.utility.Vector3dVector(thresholded_points)
        colors = get_random_colors()
        thresh_pcd.colors = o3d.utility.Vector3dVector(colors)

        thresh_pcd_copy = copy.deepcopy(thresh_pcd)
        center_point = self.draw_sphere_at_pcd_center(point_cloud=thresh_pcd_copy, display=display)
        return thresh_pcd, center_point
    
    
    def draw_sphere_at_pcd_center(self,point_cloud, sphere_radius=0.05,display=False):
        """
        Draw a sphere at the center of the given point cloud.

        Args:
            point_cloud (open3d.geometry.PointCloud): The input point cloud.
            sphere_radius (float): The radius of the sphere.
        """
        # Compute the centroid of the point cloud
        # points = np.asarray(point_cloud.points)
        centroid = point_cloud.get_center()
        o3d.io.write_point_cloud("current_pcd.pcd", point_cloud)
        center_point = self.mean_points_inside_sphere(point_cloud=point_cloud, sphere_center=centroid, sphere_radius=sphere_radius) 

        print("Patch Center:", center_point)

        if not isinstance(center_point, np.ndarray):
            return None
        
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
        sphere.translate(center_point)  # Move the sphere to the centroid
        #convert mesh to point cloud
        sphere = sphere.sample_points_poisson_disk(5000)
        sphere.paint_uniform_color([0, 1, 0])  # Paint the sphere red
        point_cloud.paint_uniform_color([1,0, 0])
        result_pcd = point_cloud + sphere
        o3d.io.write_point_cloud("current_pcd.pcd", result_pcd)
        # Visualize the point cloud and sphere
        if display:
            # Create a sphere centered at the centroid
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
            sphere.translate(center_point)  # Move the sphere to the centroid

            #convert mesh to point cloud
            sphere = sphere.sample_points_poisson_disk(5000)
            sphere.paint_uniform_color([0, 1, 0])  # Paint the sphere red
            point_cloud.paint_uniform_color([1,0, 0])
            result_pcd = point_cloud + sphere
            self.vis_pcd.points = result_pcd.points
            self.vis_pcd.colors = result_pcd.colors
            if self.frame_count == 0:
                self.vis.add_geometry(self.vis_pcd)
            self.vis.update_geometry(self.vis_pcd)
            self.vis.poll_events()
            self.vis.update_renderer()
            self.frame_count += 1
        return center_point
    

    def mean_points_inside_sphere(self, point_cloud, sphere_center, sphere_radius):
        """
        Calculate the mean of points inside a sphere.

        Args:
            point_cloud (open3d.geometry.PointCloud): The input point cloud.
            sphere_center (numpy.ndarray): The center of the sphere (1x3 array).
            sphere_radius (float): The radius of the sphere.

        Returns:
            numpy.ndarray: The mean of points inside the sphere, or None if no points are found.
        """
        points = np.asarray(point_cloud.points)

        # Compute distances of all points from the sphere center
        distances = np.linalg.norm(points - sphere_center, axis=1)

        # Filter points inside the sphere
        inside_sphere = points[distances <= sphere_radius]

        if inside_sphere.shape[0] == 0:
            #print("No points inside the sphere!")
            return None

        # Calculate the mean of points inside the sphere
        mean_point = inside_sphere.mean(axis=0)

        return mean_point


    def translate_pcd(self, ref_pcd, object_pcd,debug=False):
        """
        Finds the height difference between point clouds, and translates/moves the reference pcd based on the difference 
        """
        xyz_diff = np.median(
                        np.asarray(object_pcd.points),axis=0
                    )
        ref_pcd.translate(xyz_diff)
        #print(xyz_diff)
        object_pcd.paint_uniform_color([1, 0, 0])
        if debug:
            o3d.visualization.draw_geometries(
                                        [ref_pcd, object_pcd],
                                        window_name="Individual Height Refinement",
                                    )
            
        return ref_pcd,xyz_diff

        

        