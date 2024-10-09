from setuptools import setup

package_name = 'frob_odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alex Kulagin',
    maintainer_email='sashakulagin2007@gmail.com',
    description='Package for odometry publishing using encoder data',
    license='Apache2',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_publisher = frob_odometry.odometry:main',
        ],
    },
)
