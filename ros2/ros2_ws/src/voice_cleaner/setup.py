from glob import glob

from setuptools import find_packages, setup

package_name = 'voice_cleaner'

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
    description='Robô aspirador que recebe comandos por voz: fala -> texto -> comando ROS 2.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Camada de voz (o que este projeto acrescenta)
            'voice_listener = voice_cleaner.voice_listener:main',
            'command_parser = voice_cleaner.command_parser:main',
            'command_console = voice_cleaner.command_console:main',
            # Base herdada do potential_field
            'coppelia_bridge = voice_cleaner.coppelia_bridge:main',
            'navigator = voice_cleaner.navigator:main',
            'fake_world = voice_cleaner.fake_world:main',
            'yolo_vision = voice_cleaner.yolo_vision:main',
            'perception_map = voice_cleaner.perception_map:main',
        ],
    },
)
