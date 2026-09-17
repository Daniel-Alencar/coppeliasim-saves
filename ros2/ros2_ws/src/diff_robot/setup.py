from glob import glob

from setuptools import find_packages, setup

package_name = 'diff_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel.carvalho',
    maintainer_email='danielalencar746@gmail.com',
    description='Ponte ROS 2 com o robô diferencial do CoppeliaSim e motorista de teste.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'coppelia_bridge = diff_robot.coppelia_bridge:main',
            'dummy_driver = diff_robot.dummy_driver:main',
        ],
    },
)
