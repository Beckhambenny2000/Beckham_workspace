import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'detection_data'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='beckh',
    maintainer_email='beckh@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
	    "sender   = detection_data.sender:main",
            "receiver = detection_data.receiver:main",
            "detectiondata = detection_data.detectiondata:main",
	    "manual_handler = detection_data.manual_handler:main",
        ],
    },
)
