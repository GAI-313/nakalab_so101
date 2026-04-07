from setuptools import find_packages, setup

package_name = 'nakalab_so101_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GAI-313',
    maintainer_email='nakatogawagai@gmail.com',
    description='SO-101 driver for ROS2 python package. Support running in OSX',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "find_port = nakalab_so101_py.find_port:main",
            "leader_arm_driver_node = nakalab_so101_py.so101:leader"
        ],
    },
)
