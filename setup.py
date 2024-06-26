from setuptools import find_packages, setup

package_name = 'usv_controller'

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
    maintainer='g-poulos',
    maintainer_email='cs04480@uoi.gr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "wave_subscriber = usv_controller.wave_subscriber:main",
            "boat_controller = usv_controller.boat_control:main",
            "bag_writer = usv_controller.bag_writer:main",
            "vereniki_controller = usv_controller.vereniki_control:main",
            "vereniki_p_controller = usv_controller.vereniki_p_controller:main"
        ],
    },
)
