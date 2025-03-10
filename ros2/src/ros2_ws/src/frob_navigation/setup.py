from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'frob_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.pgm'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alex Kulagin',
    maintainer_email='sashakulagin2007@gmail.com',
    description='Frob navigation package',
    license='Apache2',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
