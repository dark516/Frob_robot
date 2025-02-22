from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'frob_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dark516',
    maintainer_email='sashakulagin@gmail.com',
    description='robot description',
    license='Apache2',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': []
    },
)

