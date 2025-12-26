from setuptools import setup

package_name = 'vla_perception'

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
            'object_detection_node = vla_perception.object_detection_node:main',
            'scene_analysis_node = vla_perception.scene_analysis_node:main',
            'visual_grounding_node = vla_perception.visual_grounding_node:main',
        ],
    },
)