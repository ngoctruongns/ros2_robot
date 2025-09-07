from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'learning_tf2_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tvn',
    maintainer_email='vungoctruong8x@gmail.com',
    description='Learning ROS 2 with Python: TF2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_tf2_message_broadcaster = learning_tf2_py.turtle_tf2_message_broadcaster:main',
        ],
    },
)
