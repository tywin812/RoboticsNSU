from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'turtle_tf2_carrot'

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
            'turtle1_tf_broadcaster = turtle_tf2_carrot.turtle1_tf_broadcaster:main',
            'carrot_tf_broadcaster = turtle_tf2_carrot.carrot_tf_broadcaster:main',
            'turtle2_tf_listener = turtle_tf2_carrot.turtle2_tf_listener:main',
        ],
    },
)
