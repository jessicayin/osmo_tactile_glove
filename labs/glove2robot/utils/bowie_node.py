"""
*   **Create a new package**:

    `ros2 pkg create --build-type ament_python bowie_ros`

*   **Navigate to the package directory** and open the `setup.py` file to add dependencies:

    `'install_requires': ['setuptools', 'pyserial', 'cobs', 'betterproto'],`

*   **Create a directory for your node**:

    `mkdir bowie_ros/bowie_ros touch bowie_ros/bowie_ros/bowie_node.py`
### Step 3: Build and Run the Node

1.  **Build the package**:

    `colcon build --packages-select bowie_ros`

2.  **Source the setup file**:

    `. install/setup.bash`

3.  **Run the node**:

    `ros2 run bowie_ros bowie_node`
"""

import rclpy
from rclpy.node import Node
import serial
from cobs import cobs
import glove2robot.utils.bowiepb as bpb
from std_msgs.msg import Float32MultiArray
import time

class BowieNode(Node):
    def __init__(self):
        super().__init__('bowie_node')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.bowie = self.connect_bowie(self.port)
        self.mag_publishers = {}
        self.imu_publishers = {}
        self.timer = self.create_timer(0.05, self.read_and_publish_data)

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

        try:
            data = self.bowie.readline()
            decoded_data = self.decode_data(data)
            for item in decoded_data:
                self.publish_data(item)
        except Exception as e:
            self.get_logger().error(f"Error reading data: {e}")

    def decode_data(self, data):
        cobs_data = data.split(b"\x00")
        frames = []
        for c_data in cobs_data:
            bowie_frame = bpb.Data()
            try:
                pb_data = cobs.decode(c_data)
                bowie_frame = bowie_frame.parse(pb_data)
                frames.append(bowie_frame)
            except Exception as e:
                self.get_logger().error(f"Decode error: {e}")
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
        topic = f"bowie/mag/{finger}/{sensor_id}"
        if topic not in self.mag_publishers:
            self.mag_publishers[topic] = self.create_publisher(Float32MultiArray, topic, 10)
        msg = Float32MultiArray()
        msg.data = [timestamp, mag['x'], mag['y'], mag['z']]
        self.mag_publishers[topic].publish(msg)

    def publish_imu_data(self, item, timestamp):
        finger = item['finger']
        sensor_id = item['sensorId']
        quat = item['quat']
        topic = f"bowie/imu/{finger}/{sensor_id}"
        if topic not in self.imu_publishers:
            self.imu_publishers[topic] = self.create_publisher(Float32MultiArray, topic, 10)
        msg = Float32MultiArray()
        msg.data = [timestamp, quat['x'], quat['y'], quat['z'], quat['w']]
        self.imu_publishers[topic].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BowieNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
