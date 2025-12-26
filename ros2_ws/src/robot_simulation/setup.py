from setuptools import setup

package_name = 'robot_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=['scripts.simulation_controller'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Simulation environment for the humanoid robot with Gazebo and Unity integration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulation_controller = scripts.simulation_controller:main',
        ],
    },
)