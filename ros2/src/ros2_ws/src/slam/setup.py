from setuptools import find_packages, setup

package_name = 'slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/slam.launch.py']),
        ('share/' + package_name + '/config', ['config/slam_toolbox_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wodo',
    maintainer_email='moswodocanal@gmail.com',
    description='i try to make slam',
    license='no license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam_node = slam.slam:main',
        ],
    },
)
