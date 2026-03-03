import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'roscar_web'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Install web assets (all files in web/ recursively)
        *[
            (os.path.join('share', package_name, dirpath), [
                os.path.join(dirpath, f) for f in files
            ])
            for dirpath, dirs, files in os.walk('web')
            if files
        ],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROScar1',
    maintainer_email='todo@todo.com',
    description='Web dashboard for ROScar1 robot operation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'launch_manager = roscar_web.launch_manager_node:main',
            'http_server = roscar_web.http_server_node:main',
        ],
    },
)
