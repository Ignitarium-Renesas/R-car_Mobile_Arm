"""json edit util functions"""
import json
import random
import unittest


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


def compare_json(json_path1, json_path2, assert_flag=True):
    """Compare two json to if they are the same and 
    Returns True if they are the same else False
    """
    json1 = read_json(json_path1)
    json2 = read_json(json_path2)
    equality = json1 == json2
    if assert_flag and not equality:
        print("\njson1:", json1)
        print("\njson2:", json2)
        assert equality, "jsons not matching"
    return equality


def edit_json(filename, key_list, value):
    """Edit a json file and save it.Eg: d['a']['b']=3"""
    data = {}
    data = read_json(filename)
    new_data = edit_dict(data, key_list, value)
    write_json(filename, new_data)


def edit_dict(data, key_list, value):
    """Edit dict with hierarchical key.Eg: d['a']['b']=3
    Returns the edited dictionary
    """
    if len(key_list) == 1:
        data[key_list[0]] = value
    else:
        if data.get(key_list[0]) is None:
            data[key_list[0]] = {}
        edit_dict(data[key_list[0]], key_list[1:], value)
    return data

def check_label_json(json_file, labelnames):
    """Return True if label exist in json file"""
    json_dict = read_json(json_file)
    shape_list = json_dict["shapes"]
    for shape_dict in shape_list:
        if shape_dict["label"] in labelnames:
            return True
    return False



class TestJsonUtils(unittest.TestCase):
    """test json utils"""

    @classmethod
    def setUpClass(cls):
        """Edit sample docker daemon config file with nvidia runtime."""
        cls.FILENAME = "samples/daemon_sample.json"
        cls.key_list = ("runtimes", "nvidia", "path")
        cls.val = random.random()
        cls.VALUE = "/usr/bin/nvidia-container-runtime - " + str(cls.val)

    def test_edit_json(self):
        """tests the edit json function"""
        print("Testing edit json function \n")
        edit_json(self.FILENAME, self.key_list, self.VALUE)
        config = read_json(self.FILENAME)
        assert config["runtimes"]["nvidia"]["path"] == "/usr/bin/nvidia-container-runtime - " + str(self.val), "Else the json contains different data"

    def test_compare_jsons(self):
        """test the compare jsons function"""
        print("Testing compare json function \n")
        compare_flg = compare_json(self.FILENAME, self.FILENAME)
        assert compare_flg is True, "Else check the compare jsons function"


if __name__ == "__main__":
    test_obj = TestJsonUtils()
    test_obj.setUpClass()
    test_obj.test_edit_json()
    test_obj.test_compare_jsons()
