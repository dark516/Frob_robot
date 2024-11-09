from setuptools import find_packages, setup

package_name = 'task_main_pkg'

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
    maintainer='dark516',
    maintainer_email='sashakulagin2007@gmail.com',
    description='Ros2 package for task main code',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'city_main_node = task_main_pkg.city_node:main',
            'go_wall_node = task_main_pkg.move_wall:main'
        ],
    },
)
