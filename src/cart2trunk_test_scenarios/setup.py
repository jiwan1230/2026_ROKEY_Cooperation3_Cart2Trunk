from setuptools import find_packages, setup

package_name = 'cart2trunk_test_scenarios'

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
    description='Reduced-scale integration test rig (table + crate) for validating the perception-planning-motion communication flow',
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
