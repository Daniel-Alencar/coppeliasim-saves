from glob import glob

from setuptools import find_packages, setup

package_name = 'a_star_banana'

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
    description='A* sobre mapa de ocupação, executado pelo controlador de posição.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'coppelia_bridge = a_star_banana.coppelia_bridge:main',
            'position_control = a_star_banana.position_control:main',
            'planner = a_star_banana.planner:main',
        ],
    },
)
