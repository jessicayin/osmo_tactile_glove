from setuptools import find_packages, setup

package_name = 'bowie_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial', 'cobs','betterproto'],
    zip_safe=True,
    maintainer='gumdev',
    maintainer_email='jessicakyin@meta.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ["bowie_node = bowie_ros.bowie_node_synced:main",
        ],
    },
)
