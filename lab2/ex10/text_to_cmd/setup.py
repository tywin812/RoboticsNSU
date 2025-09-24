from setuptools import find_packages, setup

package_name = 'text_to_cmd'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dmitriy',
    maintainer_email='chrisz0r@mail.ru',
    description='Convert text commands to messages for turtlesim',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'text_to_cmd_vel = text_to_cmd.text_to_cmd_vel:main'
        ],
    },
)
