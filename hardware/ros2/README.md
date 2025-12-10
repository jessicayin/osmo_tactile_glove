###  First Time Set Up
1.   **Create a new package**:

    `ros2 pkg create --build-type ament_python bowie_ros`

2.   **Navigate to the package directory** and replace `setup.py` with the following:
```
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
    maintainer='jessica',
    maintainer_email='jessicayin98@gmail.com',
    description='Synced node for tactile glove data',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ["bowie_node = bowie_ros.bowie_node_synced:main",
        ],
    },
)

```
3.  **Create a directory for the node**:
```
    [ros2_ws]/bowie_ros/bowie_ros/bowie_node_synced.py

Directory structure:
[ros2_ws]/bowie_ros/
     --bowie_ros/
            -- bowie_node.py
            -- glove2robot/utils/
     --package.xml
     --setup.cfg
     --setup.py
     --build/
     --install/
```

###  Build and Run the Node

1.  **Build the package**:

    `colcon build --packages-select bowie_ros`

2.  **Source the setup file**:

    `. install/setup.bash`

3.  **Run the node**:

    `ros2 run bowie_ros bowie_node`


### Rebuild package after making changes to bowie_node_synced.py

`colcon build --packages-select bowie_ros`
