from setuptools import setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['Flask>=0.12.2'],
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
          'Programming Language :: Python :: 3',
          'Topic :: Software Development',
          'Topic :: System :: Hardware'],
    entry_points={
        'console_scripts': [
            'portal_app = robot_control.portal_app:main'
        ],
    },
)
