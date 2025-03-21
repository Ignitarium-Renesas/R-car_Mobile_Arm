from pose_estimation_pkg.libs.contour import Segment
from pose_estimation_pkg.libs.utils.utils import get_slope, compute_angle, apply_rotation,get_rotation_matrix_from_transformation, combine_icp_rotations, rotation_matrix_to_euler

from pose_estimation_pkg.libs.utils.icp_utils import align_centers,prepare_dataset, execute_global_registration, draw_registration_result, icp_registration

import open3d as o3d
import cv2
import time
import numpy as np
import copy

class IcpRegistration:
    def __init__(self, pointcloud, package_path, display=False):
        self.voxel_size = 0.005
        self.display = display
        self.package_path = package_path 
        self.pointcloud = pointcloud 
        self.model = Segment(package_path=package_path)
        self.source_pcd = self.get_source_pcd()
        self.source_image = self.read_source_image()

    def get_source_pcd(self):
        o3d.utility.random.seed(42)
        source_path = f"{self.package_path}/libs/datasets/white_connector/mesh/ref_pcd.ply"
        source_pcd = o3d.io.read_point_cloud(source_path)
        return source_pcd

    def read_source_image(self):
        source_imgpath = f"{self.package_path}/libs/datasets/white_connector/images/ref_image.jpg"
        source_image = cv2.imread(source_imgpath)
        return source_image
    

    def get_pose(self,target_image, target_depth):
        # Start fresh each time.
        o3d.utility.random.seed(42)
        print("Inside get pose")
        source_pcd = copy.deepcopy(self.source_pcd)
        
        # TODO: Save all the values for reference image.
        det_1, _ = self.model.get_rect(self.source_image)
        det_2, contour = self.model.get_rect(target_image)

        if (det_1 != None) or (det_2 != None):
            contour_crop_start_time = time.time()
            target_pcd = self.pointcloud.crop_pcd(contour, target_image, target_depth,dof_6d=True)
            # o3d.io.write_point_cloud("current_pcd.pcd", target_pcd)
            # return None, None

            _, centerpoint = self.pointcloud.filter_point_cloud(target_pcd, display=False)

            cl, ind = target_pcd.remove_radius_outlier(nb_points=200, radius=0.0009)

            if not isinstance(centerpoint, np.ndarray):
                return None, True
            
            print(f"contour crop time: ", time.time() - contour_crop_start_time)

            icp_start_time = time.time()
            if self.display:
                o3d.visualization.draw_geometries([target_pcd, source_pcd], window_name="Initial orientation of Source and Target Pointclouds")

            # Finds slope b/w two contours

            slope_m1 = get_slope(det_1)
            slope_m2 = get_slope(det_2)

            angle_rad, angle_degrees = compute_angle(m1= slope_m1, m2=slope_m2)

            print(f"Rotation angle: {angle_rad:.2f} rads, {angle_degrees:.2f} degrees ({'Clockwise' if angle_degrees < 0 else 'Anticlockwise'})")

            # Align centers or the two pointclouds
            target_pcd = align_centers(source=source_pcd, target=target_pcd)

            # TODO: This depends on rotation
            angle_rad = -1*angle_rad

            source_pcd, rot_ang_marix = apply_rotation(source_pcd, angle_rad)

            if self.display:
                o3d.visualization.draw_geometries([target_pcd, source_pcd], window_name="PCD Orientation after Applying rotation")


            #Steps for applying Global ICP

            source_pcd, target_pcd, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size=self.voxel_size,
                                                                                             source=source_pcd,
                                                                                             target=target_pcd,
                                                                                             )
            print("Over here")
            result_ransac = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                self.voxel_size)
            
            rot_glob_matrix = get_rotation_matrix_from_transformation(result_ransac.transformation)

            if self.display:
                draw_registration_result(source_down, target_down, result_ransac.transformation,window_name="Global ICP Registration")

            #Local Icp
            icp = icp_registration(source=source_down, target=target_down,result_ransac=result_ransac.transformation,
                                    max_correspondence_distance_fine=self.voxel_size*15)

            rot_icp_matrix = get_rotation_matrix_from_transformation(icp.transformation)

            if self.display:
                draw_registration_result(source_pcd, target_pcd, icp.transformation,window_name="Local ICP Registration")

            rot_glob_ang = combine_icp_rotations(R1=rot_ang_marix, R2=rot_glob_matrix)
            rot_final = combine_icp_rotations(rot_glob_ang, rot_icp_matrix)
            # r_combined = rotate_yaw_90(rot_final)

            roll, pitch, yaw = rotation_matrix_to_euler(rot_final)

            print(f"X: {centerpoint[0]:.5f}, Y: {centerpoint[1]:.5f}, Z: {centerpoint[2]:.5f}, "
                f"Roll: {roll:.2f} degrees, Pitch: {pitch:.2f} degrees, Yaw: {yaw:.2f} degrees")
            
            print(f"Time for icp registration: {time.time() - icp_start_time}")

            # Debug
            target_pcd.paint_uniform_color([0, 1, 0])
            source_pcd.paint_uniform_color([1,0, 0])
            
            source_pcd.transform(icp.transformation)
            result_pcd = target_pcd + source_pcd
            o3d.io.write_point_cloud("current_pcd.pcd", result_pcd)

            return (centerpoint[0], centerpoint[1], centerpoint[2], roll, pitch ,yaw), True
        
        return None, False



def rotate_yaw_90(rot_final):
    theta = -np.pi / 2  # -90 degrees in radians
    rotation_matrix = np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta),  np.cos(theta), 0],
        [0,              0,             1]
    ])

    # Apply rotation to the point cloud
    # pcd.rotate(rotation_matrix, center=(0, 0, 0))
    R_combined = rotation_matrix @ rot_final 
    return R_combined