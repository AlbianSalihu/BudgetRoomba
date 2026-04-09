from setuptools import setup
from glob import glob
import os

package_name = 'br_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, "urdf"), glob('urdf/budget_roomba.urdf.xacro')),
        (os.path.join('share', package_name, "urdf/motion"), glob('urdf/motion/diffdrive.xacro')),
        (os.path.join('share', package_name, "urdf/sensors"), glob('urdf/sensors/imu.xacro')),
        (os.path.join('share', package_name, "urdf/sensors"), glob('urdf/sensors/nav_unit.xacro')),

        (os.path.join('share', package_name, "urdf/sensors"), glob('urdf/sensors/raspberry_pi_camera_2_1.xacro')),
        (os.path.join('share', package_name, "urdf/sensors"), glob('urdf/sensors/rplidar_a1m8.xacro')),

        (os.path.join('share', package_name, "meshes/sensors"), glob('meshes/sensors/raspberry_pi_camera_2_1.stl')),
        (os.path.join('share', package_name, "meshes/sensors"), glob('meshes/sensors/rplidar_a1m8.stl')),
        (os.path.join('share', package_name, "launch"), glob('launch/**')),






    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='svenbecker',
    maintainer_email='sven.becker@epfl.ch',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
