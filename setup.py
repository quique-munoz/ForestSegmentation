from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'lidar_camera_fusion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='enrique',
    maintainer_email='enrique.munoz@itcl.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'lidar_camera_fusion_node = lidar_camera_fusion.lidar_camera_fusion_node:main',
            'snapshot_saver = lidar_camera_fusion.snapshot_saver:main',
            'snapshot_saver_sin_tf = lidar_camera_fusion.snapshot_saver_sin_tf:main',
        ],
    },
)
