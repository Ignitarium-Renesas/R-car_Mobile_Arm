import numpy as np
import cv2
import open3d as o3d


def get_pcd_crop(contour, rgb_img, depth_img, intrinsic_mat, save_path=None, show_flag=False):
    img_height, img_width, _ = rgb_img.shape
    mask = np.zeros_like(rgb_img)
    img_mask = cv2.fillPoly(mask, [np.array(contour)], [255, 255, 255])
    img_mask = mask[:, :, 0]
    depth_mask = img_mask / 255
    depth_mask = depth_mask.astype("uint16")
    img_crop = cv2.bitwise_and(rgb_img, rgb_img, mask = img_mask)
    depth_crop = np.multiply(depth_img, depth_mask)

    if show_flag:
        visualize_depth_image(depth_img.copy())
    if show_flag:
        visualize_depth_image(depth_crop.copy())
        cv2.imshow("color image", img_crop)
        cv2.waitKey(0)
    image_3d = o3d.geometry.Image(np.ascontiguousarray(img_crop))
    depth_3d = o3d.geometry.Image(np.ascontiguousarray(depth_crop))
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(image_3d, depth_3d)
    fx, cx, fy, cy = intrinsic_mat[0], intrinsic_mat[2], intrinsic_mat[1], intrinsic_mat[2]
    pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(img_width, img_height, fx, fy, cx, cy)
    target_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,
                                pinhole_camera_intrinsic)
    if show_flag:
        o3d.visualization.draw_geometries([target_pcd])
    if save_path is not None:
        o3d.io.write_point_cloud(save_path, target_pcd)

    return target_pcd


def visualize_depth_image(depth_image):

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