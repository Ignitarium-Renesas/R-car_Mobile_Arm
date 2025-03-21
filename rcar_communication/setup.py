from setuptools import find_packages, setup, __version__ as setuptools_version
import os
from packaging.version import Version
from glob import glob

package_name = 'rcar_communication'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, "launch"), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, "meshes"), glob('meshes/*')),
        (os.path.join('share', package_name, "urdf"), glob('urdf/*')),
        (os.path.join('share', package_name, "config"), glob('config/*')),
        (os.path.join('share', package_name, "data"), glob('data/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mbasith',
    maintainer_email='mohammed.k@ignitarium.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'rcar_communication_node = rcar_communication.rcar_communication_node:main',
            'arm_services = rcar_communication.arm_services:main',
            'arm_set_pose_client = rcar_communication.arm_set_pose_client:main',
            'arm_set_pick_pose_client = rcar_communication.arm_set_pick_pose_client:main',
            'arm_linear_control_client = rcar_communication.arm_linear_control_client:main',
            'arm_gripper_control_client = rcar_communication.arm_gripper_control_client:main',
            'arm_search_client = rcar_communication.arm_search_client:main',

            'set_initial_pose= rcar_communication.base_set_pose_services:main',
            'set_initial_pose_client= rcar_communication.set_initial_pose_client:main',
            'go_to_goal_srv= rcar_communication.base_go_to_pose:main',
            'go_to_goal_client= rcar_communication.base_go_to_goal_client:main',

        ],
    },
)