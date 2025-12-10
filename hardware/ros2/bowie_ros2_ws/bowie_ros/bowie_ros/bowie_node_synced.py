import sys
# sys.path.append('/home/gumdev/human2robot/hardware/ros2/bowie_ros2_ws/bowie_ros/bowie_ros/glove2robot/')
sys.path.append('/home/gumdev/human2robot/data_collect/glove/hardware/ros2/bowie_ros2_ws/bowie_ros/bowie_ros/glove2robot/')
import utils.bowiepb as bpb
import rclpy
from rclpy.node import Node
import serial
from cobs import cobs
from std_msgs.msg import Float32MultiArray
import time
from collections import defaultdict, deque


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
        self.declare_parameter('port', '/dev/ttyACM0')
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.mag_publishers = {}
        self.imu_publishers = {}
        self.synced_publisher = self.create_publisher(Float32MultiArray, 'bowie/synced', 5)
        self.timer = self.create_timer(0.0000005, self.read_and_publish_data)  
        self.synced_timer = self.create_timer(0.04, self.publish_synced_data)
        self.drop = 0
        self.min_data_size = 37
        self.data_buffer = defaultdict(lambda: defaultdict(lambda: deque(maxlen=3))) 

        self.bowie = self.connect_bowie(self.port)
        self.get_logger().info("Running bowie glove synced node")
        self.reader = ReadLine(self.bowie)

        #data buffer settings
        self.sync_data_ready = False #buffer starts empty
        self.time_start = time.time()
        self.buffer_delay = 5 #seconds

        #sensor addresses (TODO: move to config file)
        #if sensor type is 0x1D
        self.mag_sensor_ids= {
            "index": [13, 33],
            "middle": [9, 29],
            "ring": [5, 25],
            "pinky": [1, 21],
            "thumb": [17, 37]
        }

        self.synced_msg = {
            "index": ["mag0", "mag1","imu"],
            "middle": ["mag0", "mag1","imu"],
            "ring": ["mag0", "mag1","imu"],
            "pinky": ["mag0", "mag1","imu"],
            "thumb": ["mag0", "mag1","imu"]
        }

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
        data = self.reader.readline()
        if len(data) < self.min_data_size:
            return
        decoded_data = self.decode_data(data)
        for item in decoded_data:
            self.publish_data(item)

        # if self.sync_data_ready:
        #     self.publish_synced_data()
        # else:
        #     if time.time() - self.time_start > self.buffer_delay:
        #         self.agg_data_ready = True
        #         self.get_logger().info("synced data ready to publish")
        #     else:
        #         self.get_logger().info(f"Filling synced data buffer, {time.time() - self.time_start:.2f} seconds elapsed")

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
        # Check if the sensor ID is in the predefined list
        if sensor_id == self.mag_sensor_ids[finger][0]:
            mag_num = 0
        elif sensor_id == self.mag_sensor_ids[finger][1]:
            mag_num = 1
        else:
            self.get_logger().error(f"Sensor ID {sensor_id} not recognized for finger {finger}.")
        topic = f"bowie/mag/{finger}/mag{mag_num}"
        if topic not in self.mag_publishers:
            self.mag_publishers[topic] = self.create_publisher(Float32MultiArray, topic, 10)
        msg = Float32MultiArray()
        try:
            float_arr = [float(timestamp), float(mag['x']), float(mag['y']), float(mag['z'])]
            msg.data = float_arr
            self.mag_publishers[topic].publish(msg)
            self.data_buffer[finger][f"mag{mag_num}"].append(float_arr)  # Add to fixed-size buffer
        except Exception as e:
            self.get_logger().error(f"Error publishing mag data: {e}")

    def publish_imu_data(self, item, timestamp):
        finger = item['finger']
        quat = item['quat']
        topic = f"bowie/imu/{finger}"
        if topic not in self.imu_publishers:
            self.imu_publishers[topic] = self.create_publisher(Float32MultiArray, topic, 10)
        msg = Float32MultiArray()
        try:
            float_arr = [float(timestamp), float(quat['x']), float(quat['y']), float(quat['z']), float(quat['w'])]
            msg.data = float_arr
            self.imu_publishers[topic].publish(msg)
            self.data_buffer[finger]["imu"].append(float_arr)  # Add to fixed-size buffer
        except Exception as e:
            self.get_logger().error(f"Error publishing IMU data: {e}")


    def publish_synced_data(self):
        if self.sync_data_ready:
            synced_data = []
            for finger,sensor in self.synced_msg.items():
                if self.data_buffer[finger]["mag0"]: #mag0
                    synced_data.extend(self.data_buffer[finger]["mag0"][-1])
                if self.data_buffer[finger]["mag1"]:
                    synced_data.extend(self.data_buffer[finger]["mag1"][-1])
                if self.data_buffer[finger]["imu"]: #imu
                    synced_data.extend(self.data_buffer[finger]["imu"][-1])
            msg = Float32MultiArray()
            msg.data = synced_data
            try:
                self.synced_publisher.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Error publishing synced data: {e}")
        else:
            if time.time() - self.time_start > self.buffer_delay:
                self.sync_data_ready = True
                self.get_logger().info("synced data ready to publish")
            else:
                self.get_logger().info(f"Filling synced data buffer, {time.time() - self.time_start:.2f} seconds elapsed")


def main(args=None):
    rclpy.init(args=args)
    node = BowieNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()