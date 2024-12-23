import os
import math
from collections import OrderedDict

import json
import cv2


class Labelme2YOLO(object):
    def __init__(self, json_dir, to_seg=False, output_dir="outputs"):
        self._json_dir = json_dir

        self._label_id_map = self._get_label_id_map(self._json_dir)
        self._to_seg = to_seg
        self._save_path_pfx = output_dir

    def _get_label_id_map(self, json_dir):
        label_set = set()

        for file_name in os.listdir(json_dir):
            if file_name.endswith("json") and not file_name.endswith("_result.json"):
                json_path = os.path.join(json_dir, file_name)
                data = json.load(open(json_path))
                for shape in data["shapes"]:
                    if  shape["shape_type"] == "rectangle":
                        label_set.add(shape["label"])

        return OrderedDict(
            [(label, label_id) for label_id, label in enumerate(label_set)]
        )

    def convert(self):
        json_names = [
            file_name
            for file_name in os.listdir(self._json_dir)
            if os.path.isfile(os.path.join(self._json_dir, file_name))
            and file_name.endswith(".json") and not file_name.endswith("_result.json")
        ]

        phase = os.path.basename(self._json_dir)
        self.image_dir_path = os.path.join(self._save_path_pfx, "images")
        self.label_dir_path = os.path.join(self._save_path_pfx, "labels")

        # convert labelme object to yolo format object, and save them to files
        # also get image from labelme json file and save them under images folder
        for json_name in json_names:
            json_path = os.path.join(self._json_dir, json_name)
            json_data = json.load(open(json_path))
            print("Converting %s" % (json_name))

            img_path = self._save_yolo_image(
                json_data, json_path, os.path.join(self.image_dir_path, phase)
            )

            yolo_obj_list = self._get_yolo_object_list(json_data, img_path)
            self._save_yolo_label(
                json_name, os.path.join(self.label_dir_path, phase), yolo_obj_list
            )

    def _get_yolo_object_list(self, json_data, img_path):
        yolo_obj_list = []

        img_h, img_w, _ = cv2.imread(img_path).shape
        for shape in json_data["shapes"]:
            # labelme circle shape is different from others
            # it only has 2 points, 1st is circle center, 2nd is drag end point
            if shape["shape_type"] == "circle":
                yolo_obj = self._get_circle_shape_yolo_object(shape, img_h, img_w)
                yolo_obj_list.append(yolo_obj)
            elif shape["shape_type"] =="rectangle":
                yolo_obj = self._get_other_shape_yolo_object(shape, img_h, img_w)
                yolo_obj_list.append(yolo_obj)

        return yolo_obj_list

    def _get_circle_shape_yolo_object(self, shape, img_h, img_w):
        label_id = self._label_id_map[shape["label"]]
        obj_center_x, obj_center_y = shape["points"][0]

        radius = math.sqrt(
            (obj_center_x - shape["points"][1][0]) ** 2
            + (obj_center_y - shape["points"][1][1]) ** 2
        )

        if self._to_seg:
            retval = [label_id]

            n_part = radius / 10
            n_part = int(n_part) if n_part > 4 else 4
            n_part2 = n_part << 1

            pt_quad = [None for i in range(0, 4)]
            pt_quad[0] = [
                [
                    obj_center_x + math.cos(i * math.pi / n_part2) * radius,
                    obj_center_y - math.sin(i * math.pi / n_part2) * radius,
                ]
                for i in range(1, n_part)
            ]
            pt_quad[1] = [[obj_center_x * 2 - x1, y1] for x1, y1 in pt_quad[0]]
            pt_quad[1].reverse()
            pt_quad[3] = [[x1, obj_center_y * 2 - y1] for x1, y1 in pt_quad[0]]
            pt_quad[3].reverse()
            pt_quad[2] = [[obj_center_x * 2 - x1, y1] for x1, y1 in pt_quad[3]]
            pt_quad[2].reverse()

            pt_quad[0].append([obj_center_x, obj_center_y - radius])
            pt_quad[1].append([obj_center_x - radius, obj_center_y])
            pt_quad[2].append([obj_center_x, obj_center_y + radius])
            pt_quad[3].append([obj_center_x + radius, obj_center_y])

            for i in pt_quad:
                for j in i:
                    j[0] = round(float(j[0]) / img_w, 6)
                    j[1] = round(float(j[1]) / img_h, 6)
                    retval.extend(j)
            return retval

        obj_w = 2 * radius
        obj_h = 2 * radius

        yolo_center_x = round(float(obj_center_x / img_w), 6)
        yolo_center_y = round(float(obj_center_y / img_h), 6)
        yolo_w = round(float(obj_w / img_w), 6)
        yolo_h = round(float(obj_h / img_h), 6)

        return label_id, yolo_center_x, yolo_center_y, yolo_w, yolo_h

    def _get_other_shape_yolo_object(self, shape, img_h, img_w):
        label_id = self._label_id_map[shape["label"]]

        if self._to_seg:
            retval = [label_id]
            for i in shape["points"]:
                i[0] = round(float(i[0]) / img_w, 6)
                i[1] = round(float(i[1]) / img_h, 6)
                retval.extend(i)
            return retval

        def __get_object_desc(obj_port_list):
            __get_dist = lambda int_list: max(int_list) - min(int_list)

            x_lists = [port[0] for port in obj_port_list]
            y_lists = [port[1] for port in obj_port_list]

            return min(x_lists), __get_dist(x_lists), min(y_lists), __get_dist(y_lists)

        obj_x_min, obj_w, obj_y_min, obj_h = __get_object_desc(shape["points"])

        yolo_center_x = round(float((obj_x_min + obj_w / 2.0) / img_w), 6)
        yolo_center_y = round(float((obj_y_min + obj_h / 2.0) / img_h), 6)
        yolo_w = round(float(obj_w / img_w), 6)
        yolo_h = round(float(obj_h / img_h), 6)

        return label_id, yolo_center_x, yolo_center_y, yolo_w, yolo_h

    def _save_yolo_label(self, json_name, target_dir, yolo_obj_list):
        os.makedirs(target_dir, exist_ok=True)
        txt_path = os.path.join(target_dir, json_name.replace(".json", ".txt"))

        with open(txt_path, "w+") as f:
            for yolo_obj_idx, yolo_obj in enumerate(yolo_obj_list):
                yolo_obj_line = ""
                for i in yolo_obj:
                    yolo_obj_line += f"{i} "
                yolo_obj_line = yolo_obj_line[:-1]
                if yolo_obj_idx != len(yolo_obj_list) - 1:
                    yolo_obj_line += "\n"
                f.write(yolo_obj_line)

    def _save_yolo_image(self, json_data, json_path, target_dir):
        os.makedirs(target_dir, exist_ok=True)
        json_name = os.path.basename(json_path)
        img_name = json_name.replace(".json", ".png")
        img_path = os.path.join(target_dir, img_name)

        dirname = os.path.dirname(json_path)
        image_name = json_data["imagePath"]
        src_image_name = os.path.join(dirname, image_name)
        src_image = cv2.imread(src_image_name)
        cv2.imwrite(img_path, src_image)

        return img_path

    def save_dataset_yaml(self, object_name):
        yaml_path = os.path.join(self._save_path_pfx, os.pardir, "data.yaml")

        with open(yaml_path, "w+") as yaml_file:
            yaml_file.write(f"path: {os.getcwd()}\n")
            yaml_file.write(
                f"train: {os.path.join('dataset', object_name, 'yolo_train/images')}\n"
            )
            yaml_file.write(
                f"val: {os.path.join('dataset', object_name, 'yolo_val/images')}\n"
            )
            yaml_file.write("nc: %i\n\n" % len(self._label_id_map))

            names_str = ""
            for label, _ in self._label_id_map.items():
                names_str += "'%s', " % label
            names_str = names_str.rstrip(", ")
            yaml_file.write("names: [%s]" % names_str)
