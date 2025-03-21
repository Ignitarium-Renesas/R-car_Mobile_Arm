from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'pose_estimation_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, "launch"), glob('launch/*.launch.py')),
    ] + [
        (os.path.join('share', package_name, root[len(f"{package_name}/"):]), 
         [os.path.join(root, file) for file in files])
        for root, _, files in os.walk(f'{package_name}/libs')
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
            'camera_pose = pose_estimation_pkg.camera_pose:main',
            'camera_pose_srv = pose_estimation_pkg.camera_pose_srv:main',
            'camera_pose_srv_test = pose_estimation_pkg.camera_pose_srv_test:main',
            'pose_est_client = pose_estimation_pkg.pose_est_client:main',

        ],
    },
)
