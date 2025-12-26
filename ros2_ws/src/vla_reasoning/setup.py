from setuptools import setup

package_name = 'vla_reasoning'

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
            'reasoning_node = vla_reasoning.reasoning_node:main',
            'llm_interface_node = vla_reasoning.llm_interface_node:main',
            'plan_validator_node = vla_reasoning.plan_validator_node:main',
        ],
    },
)