from setuptools import find_packages, setup

package_name = 'piece_detection_pkg'

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
    maintainer='paula',
    maintainer_email='paula@todo.todo',
    description='Piece detection package',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'piece_detection_sinBackground = piece_detection_pkg.piece_detection_sinBackground:main',
            'tower_detection_phase_node = piece_detection_pkg.tower_detection_phase_node:main',
            'loose_piece_detection_node = piece_detection_pkg.loose_piece_detection_node:main',
        ],
    },
)
