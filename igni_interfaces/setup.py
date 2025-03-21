from setuptools import find_packages, setup, __version__ as setuptools_version
import os
from packaging.version import Version
from glob import glob

package_name = 'igni_interfaces'
use_dash_separated_options = Version(setuptools_version) < Version("58.0.0")


setup_cfg_content = """
[develop]
{script_option}=$base/lib/{package_name}

[install]
{install_scripts_option}=$base/lib/{package_name}
""".format(
    package_name=package_name,
    script_option='script-dir' if use_dash_separated_options else 'script_dir',
    install_scripts_option='install-scripts' if use_dash_separated_options else 'install_scripts'
)

# 将内容写入 setup.cfg
with open("setup.cfg", "w") as f:
    f.write(setup_cfg_content)

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
            'igni_server = igni_interfaces.igni_server:main',
            'pick_place = igni_interfaces.pick_place:main',
            # 'pose_estimation = igni_interfaces.pose_estimation:main'
        ],
    },
)

