from setuptools import setup
import os
from glob import glob

package_name = 'pyroguard'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'worlds', 'textures'), glob('worlds/textures/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Firefighting DQN for TurtleBot 4',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dqn_agent_node = pyroguard.dqn_agent_node:main',
            'image_preprocessor_node = pyroguard.image_preprocessor:main',
            'lidar_vla_processor_node = pyroguard.lidar_vla_processor:main',
            'reward_publisher_node = pyroguard.reward_publisher:main',
            'fire_suppression_handler_node = pyroguard.fire_suppression_handler:main',
            'train_node = pyroguard.train:main',
            'fire_node = pyroguard.fire_node:main',
            'map_coverage_node = pyroguard.map_coverage_node:main'
        ],
    },
)