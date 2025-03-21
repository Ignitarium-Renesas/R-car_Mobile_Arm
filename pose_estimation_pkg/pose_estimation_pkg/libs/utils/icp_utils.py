# Reference: https://github.com/isl-org/Open3D/issues/2119
import open3d as o3d
import numpy as np
import copy

# from pcd import PointCloud
# from utils.utils import get_slope, compute_angle, apply_rotation,get_rotation_matrix_from_transformation, combine_icp_rotations, rotation_matrix_to_euler


def align_centers(source, target):
    center_source = np.mean(np.asarray(source.points), axis=0)
    center_target = np.mean(np.asarray(target.points), axis=0)

    # Compute translation vector
    translation = center_source -center_target # Move source to target

    # Apply translation to source point cloud
    target.translate(translation)
    
    return target


def draw_registration_result(source, target, transformation,window_name=None):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.transform(transformation)

    source_temp.estimate_normals()
    target_temp.estimate_normals()

    source_temp.paint_uniform_color([1.0, 0.1, 0.1])  
    # source_temp.paint_uniform_color(np.array([[0.5],[0.5],[0.5]]))
    target_temp.paint_uniform_color([0.1, 1.0, 0.1])
    # if axis == None:
    o3d.visualization.draw_geometries([source_temp, target_temp], window_name=window_name)

    # else:
    #     o3d.visualization.draw_geometries([source_temp, target_temp, axis])

    
def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def prepare_dataset(voxel_size, source,target):
    print(":: Load two point clouds and disturb initial pose.")

    # source = o3d.io.read_point_cloud(source_path)
    # target = o3d.io.read_point_cloud(target_path)
    
    # target = mesh2pcd(mesh_path=target_path, depthscale=depthscale)

    # o3d.visualization.draw_geometries([source, target])
    # trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
    #                          [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    # source.transform(trans_init)

    target = translate_pcd(crop_pcd=source, mesh_pcd=target)

    # draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh


def translate_pcd(crop_pcd, mesh_pcd,debug=False):
    """
    Finds the height difference between point clouds, and translates/moves the reference pcd based on the difference 
    """
        
    min_height_diff = np.median(
                    np.asarray(crop_pcd.points)[:, 2]
                ) - min(np.asarray(mesh_pcd.points)[:, 2])
    mesh_pcd.translate((0, 0, min_height_diff))
        
    return mesh_pcd

def mesh2pcd(mesh_path,depthscale):
        # o3d.utility.random.seed(42)
        np.random.seed(42)
        mesh_pcd = o3d.io.read_triangle_mesh(mesh_path)
        mesh_pcd = mesh_pcd.sample_points_poisson_disk(5000)
        mesh_pcd.translate(-mesh_pcd.get_center())
        mesh_pcd.scale(1.0/depthscale, center=mesh_pcd.get_center())

        return mesh_pcd

# def point2pointIcp():
#     distance_threshold = 5
#     result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
#             source_down, target_down, source_fpfh, target_fpfh, True,
#             distance_threshold,
#             o3d.pipelines.registration.TransformationEstimationPointToPoint(True),  # with_scaling set to True
#             3, [
#                 o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
#                     0.2), # correspondence_edge_length lowered
#                 o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
#                     distance_threshold)
#             ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))


def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 50
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.2),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(1000000, 0.999999))
    return result

# def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size):
#     distance_threshold = voxel_size * 0.4
#     print(":: Point-to-plane ICP registration is applied on original point")
#     print("   clouds to refine the alignment. This time we use a strict")
#     print("   distance threshold %.3f." % distance_threshold)
#     result = o3d.pipelines.registration.registration_icp(
#         source, target, distance_threshold, result_ransac.transformation,
#         o3d.pipelines.registration.TransformationEstimationPointToPlane())
#     return result


def icp_registration(source, target, result_ransac, max_correspondence_distance_fine):
    icp_reg = o3d.pipelines.registration.registration_icp(
        source,
        target,
        max_correspondence_distance_fine,
        result_ransac,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100)
    )
        
    return icp_reg

def align_centers(source, target):
    center_source = np.mean(np.asarray(source.points), axis=0)
    center_target = np.mean(np.asarray(target.points), axis=0)

    # Compute translation vector
    translation = center_source -center_target # Move source to target

    # Apply translation to source point cloud
    target.translate(translation)
    
    return target