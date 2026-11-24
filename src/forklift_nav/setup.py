from glob import glob

from setuptools import setup

package_name = 'forklift_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/maps', glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Prajwal Alekar',
    maintainer_email='prajwalalekar2004@gmail.com',
    description='SLAM, Nav2 configuration and pick-and-place mission for the forklift robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goto = forklift_nav.goto:main',
            'mission = forklift_nav.mission:main',
            'robot_monitor = forklift_nav.robot_monitor:main',
            'pick_place = forklift_nav.mission:main',
        ],
    },
)
