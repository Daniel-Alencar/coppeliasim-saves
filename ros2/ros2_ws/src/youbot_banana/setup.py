from glob import glob

from setuptools import find_packages, setup

package_name = 'youbot_banana'

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
    description='Manipulador móvel: A* na base holonômica e preensão com o '
                'braço de 5 juntas do KUKA youBot.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'coppelia_bridge = youbot_banana.coppelia_bridge:main',
            'base_control = youbot_banana.base_control:main',
            'arm_control = youbot_banana.arm_control:main',
            'mission = youbot_banana.mission:main',
        ],
    },
)
