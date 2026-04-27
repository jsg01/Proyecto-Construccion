from setuptools import find_packages, setup

package_name = 'ejecutor_completo'

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
    maintainer='julia',
    maintainer_email='julia@todo.todo',
    description='Nodo ejecutor para coordinar percepción, planificación y ejecución',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'executor_node = ejecutor_completo.executor_node:main',
        ],
    },
)
