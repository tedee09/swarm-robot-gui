from setuptools import find_packages, setup
import os

package_name = 'swarm_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/swarm_launch']),
        ('share/swarm_launch', ['package.xml']),
        ('share/swarm_launch/launch', ['launch/swarm_simulation.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tedee',
    maintainer_email='septedy44@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
