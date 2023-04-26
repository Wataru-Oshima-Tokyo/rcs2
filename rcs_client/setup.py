import os
from setuptools import setup
from glob import glob

package_name = 'rcs_client'
library = 'rcs_client/library'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, library],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('launch/*.launch.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='woshima@pocketradar.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rcs_client_node = rcs_client.rcs_client_node:main',
            'turtlebot3_rcs_proxy = rcs_client.turtlebot3_rcs_proxy:main',
            'move_base_proxy = rcs_client.move_base_proxy:main'
        ],
    },
)
