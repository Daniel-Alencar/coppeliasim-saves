from glob import glob

from setuptools import find_packages, setup

package_name = 'car_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/coppeliasim', glob('coppeliasim/*.lua')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel.carvalho',
    maintainer_email='danielalencar746@gmail.com',
    description='Controle do robô diferencial do CoppeliaSim por tópicos de motor.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'car_control = car_control.car_control:main',
        ],
    },
)
