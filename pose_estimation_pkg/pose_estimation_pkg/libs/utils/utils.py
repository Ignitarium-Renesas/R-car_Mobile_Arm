# -*- coding: utf-8 -*-
# --------------------------------------------------------------------------
#                         Copyright © by
#           Ignitarium Technology Solutions Pvt. Ltd.
#                         All rights reserved.
#  This file contains confidential information that is proprietary to
#  Ignitarium Technology Solutions Pvt. Ltd. Distribution,
#  disclosure or reproduction of this file in part or whole is strictly
#  prohibited without prior written consent from Ignitarium.
# --------------------------------------------------------------------------
#  Filename    : utils.py
# --------------------------------------------------------------------------
#  Description : utilities functions and helper functions
# --------------------------------------------------------------------------

"""utilities needed for main code"""

# imports
import json
import os
import cv2
import torch
import shutil
import numpy as np
from PIL import Image
import argparse
from scipy.spatial import ConvexHull
import math
from pynvml import nvmlInit, nvmlShutdown, nvmlDeviceGetHandleByIndex, nvmlDeviceGetMemoryInfo



def read_json(jsonfile):
    """Reads the json file and retuns the data"""
    with open(jsonfile, "r", encoding="UTF-8") as config_json:
        config = json.loads(config_json.read())
    return config


def write_json(jsonpath, jsondata: dict):
    """Saves the data to a json file for loading later."""
    # saving config to json file for loading later
    with open(jsonpath, "w", encoding="UTF-8") as config_json:
        json.dump(jsondata, config_json, indent=2)


def extract_boxes_from_labelme(json_file_path):
    with open(json_file_path, "r") as f:
        data = json.load(f)

    boxes = []

    for shape in data["shapes"]:
        if shape["shape_type"] == "rectangle":
            box = shape["points"]
            boxes.append(box)

    return boxes


def ansi_colors():
    """
    Generate ANSI codes for different colors used in print statements.

    Returns:
        dict: A dictionary containing color names as keys and their corresponding
              ANSI escape codes as values.

    Example:
        color_codes = ansi_colors()
        print(f"{color_codes['GREEN']}This text is in green color.{color_codes['RESET']}")
    """
    colors = {
        "BLACK": "\033[30m",
        "RED": "\033[31m",
        "GREEN": "\033[32m",
        "YELLOW": "\033[33m",
        "BLUE": "\033[34m",
        "MAGENTA": "\033[35m",
        "CYAN": "\033[36m",
        "WHITE": "\033[37m",
        "RESET": "\033[0m",
    }
    return colors


def get_image_data(image_path):
    """
    Read image data from a file and convert it to a valid numpy array.

    Attributes:
        image_path (str): The path to the image file.

    Returns:
        numpy.ndarray: The image data as a numpy array with RGB color channels.

    Raises:
        AssertionError: If the image data could not be read or loaded,
                        an assertion error is raised.

    Example:
        image_path = 'image.jpg'
        image_array = get_image_data(image_path)
    """
    image_data = cv2.imread(image_path)
    image_data = cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
    assert image_data is not None, "Could not read image, please check the path"
    return image_data


def get_depth_data(image_path):
    """
    Read image depth data from a file and convert it to a valid numpy array.

    Attributes:
        image_path (str): The path to the image file.

    Returns:
        numpy.ndarray: The image data as a numpy array with RGB color channels.

    Raises:
        AssertionError: If the image data could not be read or loaded,
                        an assertion error is raised.

    Example:
        image_path = 'image.jpg'
        image_array = get_depth_data(image_path)
    """
    if image_path.endswith(".png"):
        image_data = np.array(Image.open(image_path), dtype=np.float32) / 100
    else:
        image_data = np.load(image_path)
    assert image_data is not None, "Could not read image, please check the path"
    return image_data


def get_json_data(json_path):
    with open(json_path, "r") as file_:
        data = json.load(file_)
    return data


def select_device(mode="auto"):
    """
    Select and return an appropriate device for running computations.

    Attributes:
        mode (str, optional): The mode for selecting the device (default is "auto").
            - "auto": Automatically select CUDA (GPU) if available, otherwise use CPU.
            - "gpu": Forcefully use CUDA (GPU) if available, raise error if not.
            - "cpu": Use CPU for computations.

    Returns:
        torch.device: The selected device for computations (either CPU or GPU).

    Raises:
        ValueError: If an invalid mode value is provided.

    Example:
        device = select_device(mode="auto")
        model.to(device)
    """
    colors = ansi_colors()
    if mode == "auto":
        if torch.cuda.is_available():
            device = torch.device("cuda")
            gpu_available = check_gpu_available()
            if not gpu_available:
                device = torch.device("cpu")
        else:
            device = torch.device("cpu")

    elif mode == "gpu":
        if torch.cuda.is_available():
            device = torch.device("cuda")
            gpu_available = check_gpu_available()
            if not gpu_available:
                raise ValueError(
                    colors["RED"]
                    + "Insufficient GPU memory."
                    + " Check help (-h) for more details"
                    + colors["RESET"]
                )
        else:
            raise ValueError(
                colors["RED"]
                + "GPU device is not available."
                + " Please use CPU. Check help (-h) for more details"
                + colors["RESET"]
            )

    elif mode == "cpu":
        device = torch.device("cpu")
    else:
        raise ValueError(
            colors["RED"]
            + "Device mode must be one of [auto, gpu, cpu]."
            + " Please check help (-h) for more details."
            + colors["RESET"]
        )
    print("Device used: ", device)
    return device

def check_gpu_available(index=0):
    """Returns tuple (gpu_available, free_gpu_memory)"""
    try:
        nvmlInit()
        h = nvmlDeviceGetHandleByIndex(index)
        info = nvmlDeviceGetMemoryInfo(h)
        free_gpu = info.free // 1024**3
        print("GPU memory check: {} GB available".format(free_gpu))
        nvmlShutdown()
        return free_gpu >= 5
    except Exception as e:
        print("Error checking GPU availability:", e)
        return False



def create_safe_directory(dir_path, remove=False):
    """
    Create a directory at the given path if it does not already exist.

    Attributes:
        dir_path (str): The path for the directory to be created.
        remove (bool): The flag used to decide if directory needs to be removed in case of existed.

    Example:
        output_directory = "results/images"
        create_safe_directory(output_directory)
    """
    # Get the file extension from the path
    _, file_extension = os.path.splitext(dir_path)
    if file_extension:
        dir_path = os.path.dirname(dir_path)
    if remove:
        if os.path.exists(dir_path):
            shutil.rmtree(dir_path)
    os.makedirs(dir_path, exist_ok=True)


def get_frame_counter(object_name, mode):
    frame_counter = 0
    if os.path.exists(os.path.join("dataset", object_name, mode)):
        images = os.listdir(os.path.join("dataset", object_name, mode))
        images = [image for image in images if not image.endswith(".json")]
        if len(images) > 0:
            for image in images:
                if "frame_" in image:
                    counter = int(image.split("frame_")[-1].split(".")[0])
                    if counter > frame_counter:
                        frame_counter = counter
    return frame_counter + 1


def get_registered_object_names():
    objects = sorted(os.listdir("dataset"))
    objects = [
        item
        for item in objects
        if (not item.startswith(".") and item not in ["README.md", "bg"])
    ]
    return objects


def str_to_bool(value):
    if value.lower() == "true":
        return True
    elif value.lower() == "false":
        return False
    else:
        raise argparse.ArgumentTypeError("Boolean value expected (true/false)")


def remove_boundary_boxes(annotations):
    boxes = annotations["bbox"]
    updated_boxes = []
    for box in boxes:
        if (box[0] == 0) or (box[1] == 0) or (box[2] == 0) or (box[3] == 0):
            continue
        updated_boxes.append(box)
    annotations["bbox"] = np.array(updated_boxes)
    return annotations


def filter_segmentations(annotations, threshold):
    """Filter segmentations based on segmenation threshold"""

    filtered_segments = {}
    filtered_segments["segmentation"] = []
    filtered_bbox = []
    filtered_conf = []
    for ind, conf in enumerate(annotations["sam_conf"]):
        segment_conf = conf[0]
        if segment_conf < threshold:
            print("Segmentation removed due to less threshold")
        else:
            filtered_segments["segmentation"].append(annotations["segmentation"][ind])
            filtered_bbox.append(annotations["bbox"][ind].tolist())
            filtered_conf.append(conf)

    filtered_segments["bbox"] = np.array(filtered_bbox)
    filtered_segments["sam_conf"] = np.array(filtered_conf)

    return filtered_segments


def filter_boxes(annotations, cfg):
    boxes = annotations["bbox"]
    updated_boxes = []
    min_dim_small = cfg["detection_threshold"]["min_dim_small"]
    max_dim_small = cfg["detection_threshold"]["max_dim_small"]
    min_dim_large = cfg["detection_threshold"]["min_dim_large"]
    max_dim_large = cfg["detection_threshold"]["max_dim_large"]

    min_dim_small = max(max_dim_small * 0.5, min_dim_small * 0.8)
    min_dim_large = max(max_dim_large * 0.5, min_dim_large * 0.8)
    max_dim_small = max_dim_small * 1.2
    max_dim_large = max_dim_large * 1.2

    for box in boxes:
        width = box[2] - box[0]
        height = box[3] - box[1]
        dim_small = min(width, height)
        dim_large = max(width, height)

        # print('box:',box)
        # print("min_dim_small: ", min_dim_small)
        # print("dim_small: ", dim_small)
        # print("max_dim_small: ", max_dim_small,"\n")
        # print("min_dim_large: ", min_dim_large)
        # print("dim_large: ", dim_large)
        # print("max_dim_large: ", max_dim_large,"\n")

        if (min_dim_small < dim_small < max_dim_small) and (
            min_dim_large < dim_large < max_dim_large
        ):
            updated_boxes.append(box)
        else:
            continue
    annotations["bbox"] = np.array(updated_boxes)
    return annotations


def get_largest_n_boxes(annotations, larger_n):
    boxes = annotations["bbox"]
    if (larger_n is not None) and (larger_n < len(boxes)):
        # Calculate the area of each bounding box
        areas = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])

        # Sort indices based on area in descending order
        sorted_indices = np.argsort(areas)[::-1]

        # Select the top 3 bounding boxes
        top_n_boxes = boxes[sorted_indices[:larger_n]]
        annotations["bbox"] = top_n_boxes
    return annotations


def euclidean_distance(pts1, pts2):
    """
    This function returns eclidean distance between 2 points
    """
    x1, y1, z1 = pts1[0], pts1[1], pts1[2]
    x2, y2, z2 = pts2[0], pts2[1], pts2[2]

    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)

def check_for_captured_images(obj_name, mode, train_module="classifier"):
    if train_module == "classifier":
        dataset_dir = f"dataset/{obj_name}/{train_module}/class_{mode}"
    else:
        dataset_dir = f"dataset/{obj_name}/{mode}"
    
    if os.path.exists(dataset_dir):
        if train_module == "classifier":
            classes = os.listdir(dataset_dir)
            available_images_list = []
            for class_ in classes:
                available_images = os.listdir(os.path.join(dataset_dir, class_))
                available_images = [
                    image for image in available_images if (image.endswith(".png") or image.endswith(".jpg"))
                ]
                available_images_list.extend(available_images)

            if len(available_images_list) > 0:
                usr_inp = input(
                    f"{obj_name} contains {len(available_images_list)} {mode} images, Do you want to generate images again? (y/n): "
                )

                if usr_inp.lower() == "n":
                    return False
                else:
                    shutil.rmtree(dataset_dir, ignore_errors=True)
                    return True
        else:
            available_images = os.listdir(dataset_dir)
            available_images = [
                image for image in available_images if (image.endswith(".png") or image.endswith(".jpg"))
                ]
            if len(available_images) > 0:
                usr_inp = input(
                    f"{obj_name} contains {len(available_images)} {mode} images, Do you want to generate images again? (y/n): "
                )
                if usr_inp.lower() == "n":
                    return False
                else:
                    return True
    return True

def get_save_index(save_dir):
    file_names = os.listdir(save_dir)
    if len(file_names) == 0:
        return 0
    else:
        file_names = [int(file_name.split(".")[0]) for file_name in file_names]
        last_idx = max(file_names)
    
    return last_idx+1


def determine_text_size(height):
    """Takes height as input and changes it by a fraction (fraction_val)
    Returns the rescaled height
    """
    fraction_val = 2 / 2000
    new_text_ht = height * fraction_val
    # if new_text_ht < 1:
    #     new_text_ht = 1
    return new_text_ht


def put_texts(
    img,
    test_tuple_list=None,
    txt_thickness=3,
    v_space=50,
    txt_color=(0, 255, 0),
    default_align=None,
    offsetval=0,
    font=cv2.FONT_HERSHEY_SIMPLEX,
    draw_bg=True,
    bg_color=(0, 0, 0),
):
    """Given an image(img) and a list of texts(test_tuple_list),
    it displays the text on the image in the given position.
    Returns:
    img: image with text
    """
    if test_tuple_list is None:
        test_tuple_list = []
    ht, wd = img.shape[:2]
    l_ct = 1
    r_ct = 1
    align_left = None
    text_height = determine_text_size(ht)
    side_margin = 50

    # if not len(test_tuple_list):
    if len(test_tuple_list) == 0:
        return img

    for index, txt_tuple in enumerate(test_tuple_list):
        if isinstance(txt_tuple, str):
            text = txt_tuple
            if default_align is None:
                align_left = True
            if txt_color is None:
                txt_color = (255, 255, 255)

        elif isinstance(txt_tuple, bool):
            text = f"Oclusion {txt_tuple}"

        elif len(txt_tuple) == 3:
            text, txt_color, align_left = txt_tuple

        elif len(txt_tuple) == 0:
            break

        else:
            text = txt_tuple[0]
            if default_align is None:
                align_left = True
            if txt_color is None:
                txt_color = (255, 255, 255)
        text_size = cv2.getTextSize(text, fontFace=font, fontScale=text_height, thickness=txt_thickness)

        if align_left:
            y_ = v_space * (l_ct) + text_height
            if offsetval:
                y_ += int(offsetval[1])
                left_gap = int(offsetval[0])
            else:
                left_gap = side_margin
            l_ct += 1
        else:
            y_ = v_space * (r_ct) + text_height
            if offsetval:
                y_ += int(offsetval[1])
                left_gap = int(offsetval[0])
            else:
                left_gap = wd - text_size[0][0] - side_margin
            r_ct += 1
        put_text(
            text,
            img,
            left_gap,
            int(y_),
            txt_color,
            text_height,
            txt_thickness,
            font=font,
            draw_bg=draw_bg,
            bg_color=bg_color,
        )
    return img


def put_text(
    text,
    image,
    x,
    y,
    color=(255, 255, 255),
    font_scale=1,
    thickness=1,
    font=None,
    draw_bg=False,
    bg_color=(0, 0, 0),
    auto_align_h=True,
    auto_align_v=True,
):
    """Puts text on image. Given an image and the input text,
     it is displayed in the input position provided
    Args:
        text (str): text to put on image
        image (numpy.ndarray): input image
        x (int): x coordinate of text
        y (int): y coordinate of text
        color (tuple): color of text
        font_scale (float): font size of text
        thickness (int): thickness of text
        font (str): font of text
        draw_bg (bool): draw background or not
        bg_color (tuple): background color
    Returns:
    image: image with text
    """
    if font is None:
        font = cv2.FONT_HERSHEY_SIMPLEX
    (label_width, label_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    label_width, label_height = int(label_width), int(label_height)
    h, w = image.shape[:2]

    if auto_align_h:  # Adjust text to ensure it's enclosd within image
        if x + label_width > w:
            x = w - label_width
    if auto_align_v:
        if y + label_height > h:
            y = h - label_height

    if draw_bg:
        assert bg_color is not None, "bg_color should be given for draw bg"
        image = cv2.rectangle(
            image,
            (x, max(0, y - label_height)),
            (x + label_width, y + (label_height - baseline)),
            bg_color,
            -1,
        )
    image = cv2.putText(image, text, (int(x), int(y)), font, font_scale, color, thickness, cv2.LINE_AA)
    return image


def put_text_non_overlap(
    text,
    image,
    curr_box,
    text_box,
    color=None,
    font_scale=1,
    thickness=1,
    font=None,
    draw_bg=False,
    bg_color=(0, 0, 255),
):
    """Put text on image without overlap.
    Args:
        text (str): text to put on image
        x (int): x coordinate of text
        y (int): y coordinate of text
        color (list): color of text
        font_scale (float): font scale of text
        thickness (int): thickness of text
        font (str): font of text
        draw_bg (bool): draw background or not
        bg_color (list): background color
    Returns:
    image: image with text
    """
    [x, y, w, h] = curr_box
    [x1, y1, w1, h1] = text_box
    if font is None:
        font = cv2.FONT_HERSHEY_SIMPLEX

    if draw_bg:
        assert bg_color is not None, "bg_color should be given for draw bg"
        (label_width, label_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)
        image = cv2.rectangle(image, (x, y), (x + w, y + h), bg_color, thickness)

    # import pdb;pdb.set_trace()
    image = cv2.putText(
        image,
        text,
        (x, y + label_height),
        font,
        font_scale,
        color,
        thickness,
        cv2.LINE_AA,
    )
    return image

def get_leftmost_bbox(annotations,object_name):
    bboxes = []
    x_coords = []

    for annotation in annotations:
        for annots in annotation:
            if (len(annots) != 0) and (annots["name"] == object_name):
                bbox = annots["bbox"]
                bboxes.append(bbox)
                x_coords.append(bbox[0])

    if len(x_coords) != 0:
        min_x = min(x_coords)
        min_x_idx = x_coords.index(min_x)
        return bboxes[min_x_idx]
    return []


def get_slope(bbox):
    bbox = np.array(bbox, dtype=np.float64)
    
    if np.any(np.isinf(bbox)) or np.any(np.isnan(bbox)):
        raise ValueError("Error: bbox contains inf or NaN values!")
    
    denominator = bbox[2] - bbox[0]
    if abs(denominator) < 1e-10:  # Small threshold to prevent division by zero
        slope = float('inf')  # Or assign a default value
    else:
        slope = (bbox[3] - bbox[1]) / denominator

    # slope = (bbox[3] - bbox[1]) / (bbox[2] - bbox[0])
    return slope

# Function to compute angle between two lines
# def compute_angle(m1, m2):
#     theta_rad = np.arctan((m2 - m1) / (1 + m1 * m2))  # Signed angle
#     theta_deg = np.degrees(theta_rad)  # Convert to degrees
#     return theta_rad, theta_deg

# def compute_angle(m1, m2):
#     theta_rad = np.arctan((m2 - m1) / (1 + m1 * m2))  # Signed angle
#     # theta_rad = np.arctan(abs(m1))
#     theta_deg = np.degrees(theta_rad)  # Convert to degrees
#     return theta_rad, theta_deg

def compute_angle(m1, m2):
    # If one of the lines is vertical
    if np.isinf(m1):  # First line is vertical
        theta_rad = np.arctan(abs(m2))
    elif np.isinf(m2):  # Second line is vertical
        theta_rad = np.arctan(abs(m1))
    else:
        # Standard angle formula
        theta_rad = np.arctan(abs((m2 - m1) / (1 + m1 * m2)))

    theta_deg = np.degrees(theta_rad)  # Convert to degrees
    return theta_rad, theta_deg


def apply_rotation(pcd, theta_rad):
    # Rotation matrix for 2D (XY plane)
    R_xy = np.array([
        [np.cos(theta_rad), -np.sin(theta_rad), 0],
        [np.sin(theta_rad),  np.cos(theta_rad), 0],
        [0,                 0,                 1]
    ])
    
    # Apply rotation to point cloud
    pcd.rotate(R_xy, center=(0,0,0))
    return pcd, R_xy

def get_bounding_rectangle(cntr):
    x, y, w, h = cv2.boundingRect(cntr)
    
    # # Compute corners
    # top_left = (x, y)
    # bottom_right = (x + w, y + h)

    return (x, y, x + w, y + h)

def get_min_rectangle(cntr):
    rect = cv2.minAreaRect(cntr)
    box = cv2.boxPoints(rect)
    box = np.int8(box)
    print(box)
    box = [box[0][0], box[0][1], box[2][0], box[2][1]]
    # print(box)
    return box
    # cv2.drawContours(img,[box],0,(0,0,255),2)


def get_slope_from_cntr(cntr):
    [vx, vy, x0, y0] = cv2.fitLine(cntr, cv2.DIST_L2, 0, 0.01, 0.01)

    # Compute the slope
    slope = vy / vx  # m = vy / vx
    print(slope[0])
    return slope[0]


def get_rotation_matrix_from_transformation(T):
    """
    Extracts the 3x3 rotation matrix from a 4x4 transformation matrix.

    Parameters:
    - T: (4x4 numpy array) Transformation matrix

    Returns:
    - R: (3x3 numpy array) Rotation matrix
    """
    return T[:3, :3]  # Extract the upper-left 3×3 matrix


def combine_icp_rotations(R1, R2):
    """
    Combines two 3x3 rotation matrices from ICP transformations.

    Parameters:
    - R1: (3x3 numpy array) First rotation matrix
    - R2: (3x3 numpy array) Second rotation matrix

    Returns:
    - R_combined: (3x3 numpy array) Combined rotation matrix
    """
    # Matrix multiplication to combine rotations
    R_combined = R2 @ R1  # Equivalent to np.dot(R2, R1)
    return R_combined


def rotation_matrix_to_euler(R):
    """
    Converts a 3x3 rotation matrix into roll, pitch, and yaw (Euler angles).

    Parameters:
    - R: (3x3 numpy array) Rotation matrix

    Returns:
    - roll (float): Rotation around X-axis (degrees)
    - pitch (float): Rotation around Y-axis (degrees)
    - yaw (float): Rotation around Z-axis (degrees)
    """
    # Extract angles
    yaw = np.arctan2(R[1, 0], R[0, 0])   # Yaw (Z-axis)
    pitch = np.arcsin(-R[2, 0])          # Pitch (Y-axis)
    roll = np.arctan2(R[2, 1], R[2, 2])  # Roll (X-axis)

    # Convert to degrees
    yaw = np.degrees(yaw)
    pitch = np.degrees(pitch)
    roll = np.degrees(roll)

    return roll, pitch, yaw



def distance_to_line(px, py, x1, y1, x2, y2):
    """Compute perpendicular distance from point (px, py) to line (x1, y1) - (x2, y2)."""
    line_length = np.hypot(y2 - y1, x2 - x1)  # Store line length to avoid recomputation
    return abs((y2 - y1) * px - (x2 - x1) * py + x2 * y1 - y2 * x1) / line_length

def smallest_angle_with_x_axis(p1, p2):
    """Find the smallest positive angle (0-90°) between a line (p1, p2) and the x-axis."""
    x1, y1 = p1
    x2, y2 = p2

    # Compute the angle in radians
    theta_rad = np.arctan2(abs(y2 - y1), abs(x2 - x1))

    # Convert to degrees
    return np.degrees(theta_rad)  # Always between [0, 90]

def find_parallel_line_through_center(contour):
    """Find a line parallel to the closest side of the min-area rectangle that passes through the center."""
    # Get the minimum area bounding rectangle
    rect = cv2.minAreaRect(contour)  
    box = cv2.boxPoints(rect)  # Get 4 corner points
    box = np.intp(box)  # Convert to integer
    (cx, cy) = map(int, rect[0])  # Center of rectangle (ensure integer)

    # Define the four sides of the rotated rectangle
    sides = [
        ("side1", box[0], box[1]),  # Between point 0 and 1
        ("side2", box[1], box[2]),  # Between point 1 and 2
        ("side3", box[2], box[3]),  # Between point 2 and 3
        ("side4", box[3], box[0])   # Between point 3 and 0
    ]

    # Find the closest side to the center
    min_distance = float('inf')
    best_p1, best_p2 = None, None

    for _, p1, p2 in sides:
        dist = distance_to_line(cx, cy, p1[0], p1[1], p2[0], p2[1])
        if dist < min_distance:
            min_distance = dist
            best_p1, best_p2 = p1, p2  # Store the endpoints of the closest side

    # Compute the direction vector of the closest side
    dx = best_p2[0] - best_p1[0]
    dy = best_p2[1] - best_p1[1]

    # Find the new line passing through the center and parallel to the closest side
    new_p1 = (int(cx - dx / 2), int(cy - dy / 2))
    new_p2 = (int(cx + dx / 2), int(cy + dy / 2))

    # Compute the smallest angle with the x-axis
    angle = smallest_angle_with_x_axis(new_p1, new_p2)

    # Determine the slope sign
    slope_sign = "Positive" if dy * dx > 0 else "Negative"

    return new_p1, new_p2, angle, slope_sign