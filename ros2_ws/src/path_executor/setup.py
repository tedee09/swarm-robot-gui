from setuptools import find_packages, setup

package_name = 'path_executor'

setup(
    name=package_name,
    version='0.0.0',
    packages=['path_executor'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'path_executor_node = path_executor.path_executor_node:main',
        ],
    },
)
