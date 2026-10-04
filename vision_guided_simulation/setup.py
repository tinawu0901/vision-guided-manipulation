from glob import glob
from setuptools import find_packages, setup

setup(
    name='vision_guided_simulation', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/vision_guided_simulation']),
        ('share/vision_guided_simulation', ['package.xml']),
        ('share/vision_guided_simulation/launch', glob('launch/*.launch.py')),
        ('share/vision_guided_simulation/config', glob('config/*.yaml')),
        ('share/vision_guided_simulation/models/water_bottle', glob('models/water_bottle/*')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Tina Wu', maintainer_email='asd28018807@gmail.com',
    description='Camera-guided Panda reach demo in Gazebo Harmonic.', license='MIT',
    entry_points={'console_scripts': [
        'scene_monitor = vision_guided_simulation.scene_monitor:main',
        'demo_recorder = vision_guided_simulation.demo_recorder:main',
    ]},
)
