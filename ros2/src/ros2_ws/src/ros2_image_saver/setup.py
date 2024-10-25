from setuptools import find_packages, setup

package_name = 'ros2_image_saver'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'cv_bridge',
        'rclpy',
        'sensor_msgs',
    ],
    zip_safe=True,    maintainer='Alex Kulagin',
    maintainer_email='sashakulagin2007@gmail.com',
    description='a simple ros2 node for saving all images from the ros topic',
    license='MIT license',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'image_saver = ros2_image_saver.image_saver:main',
        ],
    },
)
