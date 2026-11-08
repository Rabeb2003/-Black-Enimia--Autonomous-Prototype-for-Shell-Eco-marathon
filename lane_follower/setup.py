from setuptools import setup

package_name = 'lane_follower'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='ihw',
    author_email='ihw@example.com',
    maintainer='ihw',
    maintainer_email='ihw@example.com',
    keywords=['ROS2'],
    description='Lane detection, path planning, and pure-pursuit control nodes for a small autonomous vehicle.',
    license='MIT',
)
