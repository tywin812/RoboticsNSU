from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtles_delay'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dmitriy',
    maintainer_email='chrisz0r@mail.ru',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_tf_broadcaster = turtles_delay.turtle_tf_broadcaster:main',
            'turtle_tf_listener = turtles_delay.turtle_tf_listener:main'
        ],
    },
)
