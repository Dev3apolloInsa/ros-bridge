from setuptools import setup

package_name = 'carla_vehicle_trajectory'

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
    maintainer='boubacar',
    maintainer_email='boubacar@insa-toulouse.fr',
    description='The carla_vehicle_trajectory',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'vehicle_target_trajectory = carla_vehicle_trajectory.draw_target_trajectory:main',
        ],
    },
)
