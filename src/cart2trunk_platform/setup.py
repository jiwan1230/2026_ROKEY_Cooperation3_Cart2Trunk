from setuptools import find_packages, setup

package_name = 'cart2trunk_platform'

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
    description='Hardware control layer for the omni-wheel mobile base, lift, M0609 arm, suction gripper, and RealSense camera (Isaac Sim / real-robot adapters)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
