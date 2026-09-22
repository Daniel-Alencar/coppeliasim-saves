from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_docking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel.carvalho',
    maintainer_email='daniel.carvalho@cnpem.br',
    description='Ponte ROS 2 com os sinais de docking do myRobot no CoppeliaSim.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'coppelia_bridge = robot_docking.coppelia_bridge:main',
        ],
    },
)
