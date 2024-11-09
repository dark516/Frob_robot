from setuptools import find_packages, setup

package_name = 'ros2_arduino_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'pyserial~=3.5',
        'pytest~=8.3.2',
        'pyserial',
        'typing',
    ],
    zip_safe=True,
    maintainer='Alex Kulagin',
    maintainer_email='sashakulagin2007@gmail.com',
    description='ros2 node to contact with arduino',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
    'console_scripts': [
        'arduino_bridge = ros2_arduino_bridge.arduino_bridge:main',
        ],
    },
)
