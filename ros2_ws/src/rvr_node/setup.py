from setuptools import setup
from glob import glob
import os
from setuptools import find_namespace_packages, find_packages

package_name = 'rvr_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'sphero_sdk.sphero_rvr_interface'] + find_packages() + find_namespace_packages(where='sphero_sdk/sphero_sdk_raspberry_python/sphero_sdk'),
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'] + [
        'aiohttp == 3.7.4',
        'requests == 2.21.0',
        'websocket-client == 0.54.0',
        'pyserial == 3.4',
        'pyserial-asyncio == 0.4',
        'twine == 1.13.0'
    ],
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

# packages=[
#         package_name,
#         'sphero_sdk.sphero_rvr_interface',
#         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk',
#         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.common.enums.colors_enums',
#         'sphero_sdk.common.enums.infrared_codes_enums',
#         'sphero_sdk.common.enums.rvr_led_groups_enum',
#         'sphero_sdk.common.enums.sphero_rvr_enums',
#         'sphero_sdk.common.rvr_streaming_services',
#         'sphero_sdk.common.enums.api_and_shell_enums'],