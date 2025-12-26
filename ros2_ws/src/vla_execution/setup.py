from setuptools import setup

package_name = 'vla_execution'

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
    maintainer='TODO',
    maintainer_email='TODO',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'execution_node = vla_execution.execution_node:main',
            'navigation_node = vla_execution.navigation_node:main',
            'manipulation_node = vla_execution.manipulation_node:main',
            'action_sequencer_node = vla_execution.action_sequencer_node:main',
        ],
    },
)