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
