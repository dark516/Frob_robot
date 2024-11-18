from setuptools import find_packages, setup

package_name = 'city_task_nodes'

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
    description='nodes for rtk cup, city task',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'main_node = city_task_nodes.main_node:main',
            'go_wall_node = city_task_nodes.move_wall:main',
            'ceil_definition = city_task_nodes.ceil_definition:main'
        ],
    },

)
