from setuptools import find_packages, setup

package_name = 'frob_control'

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
    maintainer='Alex Kulagin',
    maintainer_email='sashakulagin2007@gmail.com',
    description='Frob control package',
    license='Apache2',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'turn_server = frob_control.turn_service:main',
            'forward_server = frob_control.forward_service:main',
            'robot_control = frob_control.client:main',
            'turn_client = frob_control.turn_client:main'
        ],
    },
)
