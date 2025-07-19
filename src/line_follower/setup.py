from setuptools import setup
from glob import glob
import os

package_name = 'line_follower'

# Collect all files in the models directory recursively
model_files = []
for dirpath, _, filenames in os.walk('models'):
    for f in filenames:
        full_path = os.path.join(dirpath, f)
        install_path = os.path.join('share', package_name, dirpath)
        model_files.append((install_path, [full_path]))

# Add any other non-model files
data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/line_follower']),
    ('share/line_follower', ['package.xml']),
    ('share/line_follower/launch', glob('launch/*.launch.py')),
    ('share/line_follower/worlds', glob('worlds/*.world')),
]

# Combine with model files
data_files += model_files

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pc',
    maintainer_email='pc@todo.todo',
    description='Line follower node for ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_follower_node = line_follower.line_follower_node:main',
        ],
    },
)
