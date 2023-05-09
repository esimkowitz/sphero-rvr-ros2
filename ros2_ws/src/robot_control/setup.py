from setuptools import setup

import os

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        ('share/' + package_name, 
            ['package.xml']),
    ],
    include_package_data=True,
    install_requires=[
        'Flask>=5.0.0',
        'Flask-SocketIO>=5.0.0',
        "eventlet>=0.33.0"],
    zip_safe=True,
    maintainer='Evan Simkowitz',
    maintainer_email='esimkowitz@wustl.edu',
    description='Control the RVR via a local web portal',
    license='MIT',
    tests_require=['pytest'],
    classifiers=[
          'Development Status :: 2 - Beta',
          'Operating System :: POSIX :: Linux',
          'License :: OSI Approved :: MIT License',
          'Intended Audience :: Developers',
          'Programming Language :: Python :: 3.10',
          'Topic :: Software Development',
          'Topic :: System :: Hardware'],
    entry_points={
        'console_scripts': [
            'robot_control = robot_control.robot_control_app:main'
        ],
    },
)
