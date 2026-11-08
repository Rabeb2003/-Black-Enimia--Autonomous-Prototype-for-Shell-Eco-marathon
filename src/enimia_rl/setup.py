from setuptools import setup

package_name = 'enimia_rl'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ("share/" + package_name + "/config", ["config/rl_config.yaml"]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rabeb',
    maintainer_email='rabeb@example.com',
    description='Hybrid RL + classical control for Black Enimia lane keeping',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'train_rl = enimia_rl.train:main',
            'rl_lane_assist = enimia_rl.rl_wrapper:main',
        ],
    },
)
