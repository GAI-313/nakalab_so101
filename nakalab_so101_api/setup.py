from setuptools import find_packages, setup

package_name = 'nakalab_so101_api'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GAI-313',
    maintainer_email='nakatogawagai@gmail.com',
    description='Python MoveIt API for the SO-101 arm.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'api_smoke = nakalab_so101_api.api_smoke:main',
        ],
    },
)
