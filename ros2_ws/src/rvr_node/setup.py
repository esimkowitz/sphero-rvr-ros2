from setuptools import setup
from glob import glob
import os

package_name = 'rvr_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('lib', 'python3.10', 'site-packages', 'sphero_sdk'), glob('sphero_sdk_raspberry_python/sphero_sdk/**/*.py', recursive=True)),
        (os.path.join('lib', 'python3.10', 'site-packages', 'sphero_rvr_interface'), glob('sphero_rvr_interface/**/*.py', recursive=True))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Evan Simkowitz',
    maintainer_email='esimkowitz@wustl.edu',
    description='Relays commands between ROS2 and the Sphero RVR SDK',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rvr_node = rvr_node.rvr_node_app:main'
        ],
    },
)
