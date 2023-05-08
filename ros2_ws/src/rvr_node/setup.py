from setuptools import setup
from glob import glob
import os
from setuptools import find_namespace_packages, find_packages

package_name = 'rvr_node'

setup(
    name=package_name,
    version='0.0.0',

    # packages=[
    #         package_name,
    #         'sphero_sdk.sphero_rvr_interface',
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk',
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.common',
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.observer'
    #         ],
    # py_modules=[
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.common.enums.colors_enums',
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.common.enums.infrared_codes_enums',
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.common.enums.rvr_led_groups_enum',
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.common.enums.sphero_rvr_enums',
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.common.rvr_streaming_services',
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.common.enums.api_and_shell_enums',
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.common.enums.drive_enums',
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.common.enums.io_enums',
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.common.enums.power_enums',
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.common.enums.sensor_enums',
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.observer.observer_base',
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.observer.client.firmware.rvr_fw_check_observer',
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.observer.events.event_dispatcher',
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.observer.controls.led_control_observer',
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.observer.controls.drive_control_observer',
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.observer.controls.infrared_control_observer',
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.observer.controls.sensor_control_observer',
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.observer.client.dal.observer_parser',
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.observer.client.dal.serial_observer_port',
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.observer.client.dal.serial_observer_dal',
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.observer.client.toys.sphero_rvr_observer',
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.common.commands.api_and_shell',
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.common.commands.connection',
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.common.commands.drive',
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.common.commands.io',
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.common.commands.power',
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.common.commands.sensor',
    #         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.common.commands.system_info'],
    # packages=[package_name, 'sphero_sdk.sphero_rvr_interface'] + find_packages() + find_namespace_packages(where='sphero_sdk/sphero_sdk_raspberry_python/sphero_sdk'),
    packages=find_namespace_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
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

# packages=[
#         package_name,
#         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.sphero_rvr_interface',
#         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk',
#         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.common.enums.colors_enums',
#         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.common.enums.infrared_codes_enums',
#         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.common.enums.rvr_led_groups_enum',
#         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.common.enums.sphero_rvr_enums',
#         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.common.rvr_streaming_services',
#         'sphero_sdk.sphero_sdk_raspberry_python.sphero_sdk.common.enums.api_and_shell_enums'],