from setuptools import setup

package_name = 'frob_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/slam_launch.py']),  # Добавляем launch файл
        ('share/' + package_name + '/config', ['config/slam_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alex Kulagin',
    maintainer_email='sashakulagin2007@gmail.com',
    description='Ros2 frob_navigation packae',
    license='Apache2',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)

