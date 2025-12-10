"""
### Step 1: Create a ROS 2 Package

1.  **Create a new package**:

    `ros2 pkg create --build-type ament_python realsense`

2.  **Navigate to the package directory** and open the `setup.py` file to add dependencies:

    `'install_requires': ['setuptools', 'pyrealsense2', 'opencv-python', 'numpy'],`

3.  **Create a directory for your node**:

    `mkdir realsense_ros/realsense touch realsense_ros/realsense/realsense_node.py`

### Step 3: Build and Run the Node

1.  **Build the package**:

    `colcon build --packages-select realsense_ros`

2.  **Source the setup file**:

    `. install/setup.bash`

3.  **Run the node**:

    `ros2 run realsense_ros realsense_node`
"""
import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# from foxglove_websocket import FoxgloveServer, Image as FoxgloveImage

class RealSenseNode(Node):
    def __init__(self):
        super().__init__('realsense_node')
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.bridge = CvBridge()

        # Configure streams
        # self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        # self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        # self.config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
        # self.config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)

        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.infrared, 1, 1280, 720, rs.format.y8, 30)
        self.config.enable_stream(rs.stream.infrared, 2, 1280, 720, rs.format.y8, 30)

        # Get aligned streams
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        # Start streaming
        self.profile = self.pipeline.start(self.config)

        color_sensor = self.profile.get_device().query_sensors()[1] # Index 1 for color
        color_sensor.set_option(rs.option.enable_auto_exposure, False)
        color_sensor.set_option(rs.option.exposure, 200.0)          # example value (tune)

        depth_sensor = self.profile.get_device().query_sensors()[0] # Index 0 for depth
        depth_sensor.set_option(rs.option.emitter_enabled, 0) # 0 for off

        # Create publishers
        self.rgb_pub = self.create_publisher(Image, '/realsense/rgb', 10)
        self.depth_pub = self.create_publisher(Image, '/realsense/depth', 10)
        self.left_ir_pub = self.create_publisher(Image, '/realsense/left_ir', 10)
        self.right_ir_pub = self.create_publisher(Image, '/realsense/right_ir', 10)


        # Create a timer to periodically publish images
        self.timer = self.create_timer(0.04, self.publish_images)

        # Info
        self.get_logger().info("RealSense Node started. Publishing images...")

        # # Set up Foxglove WebSocket server
        # self.server = FoxgloveServer("0.0.0.0", 8765)
        # self.server.start()

    def publish_images(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        left_ir_frame = frames.get_infrared_frame(1)
        right_ir_frame = frames.get_infrared_frame(2)

        if not color_frame or not depth_frame or not left_ir_frame or not right_ir_frame:
            return
        
        aligned_frames = self.align.process(frames)
        aligned_depth_image = np.asanyarray(aligned_frames.get_depth_frame().get_data())


        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        left_ir_image = np.asanyarray(left_ir_frame.get_data())
        right_ir_image = np.asanyarray(right_ir_frame.get_data())

        # self.get_logger().info(f"Publishing images: Color {color_image.shape}, Depth {aligned_depth_image.shape}")

        # Publish images to ROS 2 topics
        self.rgb_pub.publish(self.bridge.cv2_to_imgmsg(color_image, "bgr8"))
        self.depth_pub.publish(self.bridge.cv2_to_imgmsg(aligned_depth_image, "16UC1"))
        self.left_ir_pub.publish(self.bridge.cv2_to_imgmsg(left_ir_image, "mono8"))
        self.right_ir_pub.publish(self.bridge.cv2_to_imgmsg(right_ir_image, "mono8"))

        # # Publish images to Foxglove
        # self.server.send_message("/realsense/rgb", FoxgloveImage(color_image, "bgr8"))
        # self.server.send_message("/realsense/depth", FoxgloveImage(depth_image, "16UC1"))
        # self.server.send_message("/realsense/left_ir", FoxgloveImage(left_ir_image, "mono8"))
        # self.server.send_message("/realsense/right_ir", FoxgloveImage(right_ir_image, "mono8"))

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
