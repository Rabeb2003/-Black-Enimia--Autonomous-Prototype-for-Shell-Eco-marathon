from setuptools import setup
import os
from glob import glob

package_name = 'lane_detection'
model_files = glob('models/*')

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        (os.path.join('share', package_name, 'models'), model_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wafa',
    maintainer_email='wafa.ammar75@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_detection_node = lane_detection.lane_detection_node:main',
            'ccd_lane_detection = lane_detection.ccd_lane_detection:main',
            'robust_lane_detector = lane_detection.robust_lane_detector:main',
            'fused_lane_detector = lane_detection.fused_lane_detector:main',
            'stop_sign_node = lane_detection.stop_sign_node:main',
            'cmd_vel_safety_gate = lane_detection.cmd_vel_safety_gate:main',
            'lane_keeping_controller = lane_detection.lane_keeping_controller:main',
            'fused_circuit_controller = lane_detection.fused_circuit_controller:main',
            'vehicle_encoder_bridge = lane_detection.vehicle_encoder_bridge:main',
            'lidar_obstacle_monitor = lane_detection.lidar_obstacle_monitor:main',
        ],
    },
)
