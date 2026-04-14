from setuptools import setup

package_name = 'calibration_services'

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
    maintainer='paula',
    maintainer_email='pauli03ta@gmail.com',
    description='Servicios de transformación para calibración',
    license='MIT',
    entry_points={
        'console_scripts': [
            'plane_transform_server = calibration_services.plane_transform_server:main',
        ],
    },
)
