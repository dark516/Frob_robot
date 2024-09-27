from setuptools import find_packages, setup

package_name = 'pi_camera'

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
    maintainer='Rafael Papallas',
    maintainer_email='rpapallas@gmail.com',
    description='Simple ROS2 package to publish PI camera frames.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publish_compressed = pi_camera.publish_compressed:main',
            'view_compressed = pi_camera.view_compressed:main',
        ],
    },
)
