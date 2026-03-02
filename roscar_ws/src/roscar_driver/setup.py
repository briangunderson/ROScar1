import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'roscar_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROScar1',
    maintainer_email='todo@todo.com',
    description='Hardware driver for ROScar1 4WD Mecanum robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'driver_node = roscar_driver.driver_node:main',
        ],
    },
)
