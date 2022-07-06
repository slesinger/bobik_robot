from setuptools import setup
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'bobik_robot'

# !!! If colcon build does not copy files, make sure current dir is ~/ros2_foxy !!!

# data files have format (target, source)
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xml')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'params'), glob('params/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('lib', package_name, 'skills'), glob('bobik_robot/skills/*.py')),
        (os.path.join('lib', package_name, 'pyogg'), glob('bobik_robot/pyogg/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Honza',
    maintainer_email='slesinger@gmail.com',
    description='Files specific to Bobik robot, containing drived HW interfaces and launch files.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = bobik_robot.state_publisher:main',
            'bobik_robot = bobik_robot.bobik_robot:main'
        ],
    },
)
