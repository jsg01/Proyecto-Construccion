from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'detection_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ricardo',
    maintainer_email='ricardo.ortizpozo@luxpmsoft.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
              #'detection_node = detection_node.piece_detection_node:main',
              #'detection_publisher = detection_node.piece_detection_publisher:main',
              'detection_node = detection_node.piece_detection_node_second:main'
        ],
    },
)
