from glob import glob

from setuptools import find_packages, setup

package_name = 'slam_field'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel.carvalho',
    maintainer_email='danielalencar746@gmail.com',
    description='Campo potencial com obstáculos do mapa do SLAM Toolbox e parada pelo laser.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'coppelia_bridge = slam_field.coppelia_bridge:main',
            'navigator = slam_field.navigator:main',
        ],
    },
)
