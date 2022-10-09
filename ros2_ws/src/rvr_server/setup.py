from setuptools import setup

package_name = 'rvr_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Evan Simkowitz',
    maintainer_email='esimkowitz@wustl.edu',
    description='Server to execute commands on RVR',
    license='MIT',
    entry_points={
        'console_scripts': [
            "rvr_server = rvr_server.sphero_node:main",
        ],
    },
)