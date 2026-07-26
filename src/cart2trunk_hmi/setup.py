from setuptools import find_packages, setup

package_name = 'cart2trunk_hmi'

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
    maintainer='jiwan',
    maintainer_email='gwanshin12301230@gmail.com',
    description=(
        "ROS2 bridge for the web/ HMI (Flask+React) - Flask itself lives "
        "outside this workspace's colcon build, see web/README.md"
    ),
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ros_bridge_node = cart2trunk_hmi.ros_bridge_node:main',
        ],
    },
)
