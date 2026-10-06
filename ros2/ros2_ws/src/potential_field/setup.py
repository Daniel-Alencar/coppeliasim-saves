from glob import glob

from setuptools import find_packages, setup

package_name = 'potential_field'

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
    description='Campos potenciais no CoppeliaSim: atração pelas bananas, repulsão dos poops.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'coppelia_bridge = potential_field.coppelia_bridge:main',
            'navigator = potential_field.navigator:main',
            'fake_world = potential_field.fake_world:main',
            'yolo_vision = potential_field.yolo_vision:main',
            'perception_map = potential_field.perception_map:main',
        ],
    },
)
