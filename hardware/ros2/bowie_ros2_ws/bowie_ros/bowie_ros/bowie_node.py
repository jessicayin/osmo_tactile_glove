"""
###  First Time Set Up
1.   **Create a new package**:

    `ros2 pkg create --build-type ament_python bowie_ros`

2.   **Navigate to the package directory** and replace `setup.py` with the following:

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
        'console_scripts': ["bowie_node = bowie_ros.bowie_node:main",
        ],
    },
)


3.  **Create a directory for the node**:

    [ros2_ws]/bowie_ros/bowie_ros/bowie_node.py

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


###  Build and Run the Node

1.  **Build the package**:

    `colcon build --packages-select bowie_ros`

2.  **Source the setup file**:

    `. install/setup.bash`

3.  **Run the node**:

    `ros2 run bowie_ros bowie_node`


rebuild package after making changes to bowie_node.py

colcon build --packages-select bowie_ros

"""
import sys
sys.path.append('/home/gumdev/human2robot/hardware/ros2/bowie_ros2_ws/bowie_ros/bowie_ros/glove2robot/')
import rclpy
from rclpy.node import Node
import serial
from cobs import cobs
import utils.bowiepb as bpb
# from bpb import Data
# from .glove2robot.utils import bowiepb as bpb
# from .glove2robot.utils.bowiepb import Data, Finger
from std_msgs.msg import Float32MultiArray
import time
import numpy as np
# from foxglove_websocket import FoxgloveServer, Scalars

class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s

    def readline(self):
        i = self.buf.find(b"\x00")
        if i >= 0:
            r = self.buf[: i + 1]
            self.buf = self.buf[i + 1 :]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\x00")
            if i >= 0:
                r = self.buf + data[: i + 1]
                self.buf[0:] = data[i + 1 :]
                return r
            else:
                self.buf.extend(data)

class BowieNode(Node):
    def __init__(self):
        super().__init__('bowie_node')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.mag_publishers = {}
        self.imu_publishers = {}
        self.timer = self.create_timer(0.000001, self.read_and_publish_data)
        self.drop = 0
        self.min_data_size = 37

        self.bowie = self.connect_bowie(self.port)
        self.reader = ReadLine(self.bowie)

        #foxglove
        # self.server = FoxgloveServer("0.0.0.0", 8765)
        # self.server.start()

    def connect_bowie(self, port):
        try:
            ser = serial.Serial(port)
            ser.reset_output_buffer()
            ser.reset_input_buffer()
            self.get_logger().info(f"Connected to BowieGlove on {port}")
            return ser
        except Exception as e:
            self.get_logger().error(f"Failed to connect to BowieGlove: {e}")
            return None

    def read_and_publish_data(self):
        if self.bowie is None:
            return
        # try:
        else:
            data = self.reader.readline()
            if len(data) < self.min_data_size:
                # self.get_logger().warning(f"[WARNING] Data too short, skipping")
                pass
            else:
                decoded_data = self.decode_data(data)
                for item in decoded_data:
                    self.publish_data(item)
        # except Exception as e:
        #     self.get_logger().erro(f"Error reading data: {e}")

    def decode_data(self, data):
        cobs_data = data.split(b"\x00")
        frames = []
        for c_data in cobs_data:
            bowie_frame = bpb.Data()
            try:
                pb_data = cobs.decode(c_data)
                bowie_frame = bowie_frame.parse(pb_data)
                frames.append(bowie_frame)
            except cobs.DecodeError:
                self.drop += 1
                self.get_logger().info(f">> COBS decode error on data: {cobs_data}")
            except Exception as e:
                self.get_logger().error(f"Decode error: {e}")
                self.drop += 1
                self.get_logger().info(f"Dropped frames: {self.drop}")

        return frames

    def publish_data(self, item):
        item_dict = item.to_dict()
        timestamp = time.monotonic_ns()
        if 'mag' in item_dict:
            self.publish_mag_data(item_dict, timestamp)
        if 'quat' in item_dict:
            self.publish_imu_data(item_dict, timestamp)

    def publish_mag_data(self, item, timestamp):
        finger = item['finger']
        sensor_id = item['sensorId']
        mag = item['mag']
        topic = f"bowie/mag/{finger}/sensor{sensor_id}"
        if topic not in self.mag_publishers:
            self.mag_publishers[topic] = self.create_publisher(Float32MultiArray, topic, 10)
        msg = Float32MultiArray()
        try:
            msg.data = [float(timestamp), float(mag['x']), float(mag['y']), float(mag['z'])]
            self.mag_publishers[topic].publish(msg)
        # self.server.send_message(topic, Scalars([float(timestamp), float(mag['x']), float(mag['y']), float(mag['z'])]))
        except Exception as e:
            self.get_logger().error(f"Error publishing mag data: {e}")
            self.get_logger().info(f"quat: {item}")
            self.get_logger().info(f"msg: {msg.data}")
            pass

    def publish_imu_data(self, item, timestamp):
        finger = item['finger']
        sensor_id = item['sensorId']
        quat = item['quat']
        topic = f"bowie/imu/{finger}/sensor{sensor_id}"
        if topic not in self.imu_publishers:
            self.imu_publishers[topic] = self.create_publisher(Float32MultiArray, topic, 10)
        msg = Float32MultiArray()
        try:
            msg.data = [float(timestamp), float(quat['x']), float(quat['y']), float(quat['z']), float(quat['w'])]
            self.imu_publishers[topic].publish(msg)
            # self.server.send_message(topic, Scalars([float(timestamp), float(quat['x']), float(quat['y']), float(quat['z']), float(quat['w'])]))
        except Exception as e:
            self.get_logger().error(f"Error publishing IMU data: {e}")
            self.get_logger().info(f"quat: {item}")
            self.get_logger().info(f"msg: {msg.data}")
            pass

def main(args=None):
    rclpy.init(args=args)
    node = BowieNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
