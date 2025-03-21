from setuptools import find_packages, setup

package_name = 'myagv_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'myagv_teleop_node = myagv_teleop.myagv_teleop_node:main'
        ],
    },
)
