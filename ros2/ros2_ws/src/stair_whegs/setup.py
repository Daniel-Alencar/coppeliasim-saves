from glob import glob

from setuptools import find_packages, setup

package_name = 'stair_whegs'

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
    description='Rover de whegs que sobe escada, com marcha de fases '
                'coordenadas e percepção de terreno.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'coppelia_bridge = stair_whegs.coppelia_bridge:main',
            'gait = stair_whegs.gait:main',
            'climber = stair_whegs.climber:main',
        ],
    },
)
