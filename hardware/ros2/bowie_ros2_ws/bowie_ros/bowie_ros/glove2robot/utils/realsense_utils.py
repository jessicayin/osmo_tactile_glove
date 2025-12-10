import pyrealsense2 as rs
import cv2
import numpy as np
import threading
import time
import os

from collections import deque

def init_realsense(width=640, height=480, fps=30):
    """
    Sets up realsense camera for (640, 480) color and depth and fps=30 by default

    Returns:
        pipeline object for sampling frames
    """
    # camera initialization
    pipeline = rs.pipeline()
    config = rs.config()

    try:
        pipeline_wrapper = rs.pipeline_wrapper(pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))
    except Exception as e:
        print(e)
        print(f"Camera not found. Is the realsense connected? Try power cycling a few times.")

    if device_product_line == "D400":
        # Print available streams
        print("Available streams:")
        for stream in [rs.stream.depth, rs.stream.color, rs.stream.infrared]:
            print(f"- {stream}")
            
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        config.enable_stream(rs.stream.infrared, 1, width, height, rs.format.y8, fps)
        config.enable_stream(rs.stream.infrared, 2, width, height, rs.format.y8, fps)
        # config.enable_stream(rs.stream.infrared2, width, height, rs.format.y8, fps)

    else:
        raise NotImplementedError(f"Wrong camera type. Found {device_product_line} instead of D400")
    
    try:
        realsense = RealsenseStream(pipeline, config)
    except Exception as e:
        # RuntimeError: no device connected
        # Segmentation fault: 11
        print(e)
        print("Try power cycling the realsense and re-running script a few times. If problem persists, debug using check_realsense.py")
        exit(0)
    return realsense

class RealsenseStream:

    def __init__(self, pipeline, config):
        self.pipeline = pipeline
        self.config = config

        self.realsense_deque = deque(maxlen=10)
        self.realsense_lock = threading.Lock()

        print(f"Starting realsense pipeline...")
        self.pipeline.start(self.config)
        

    def camera_thread(self, stop_event):
        
        try:
            while not stop_event.is_set():
                frames = self.pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()

                if not depth_frame or not color_frame:
                    continue
                depth_image = np.asanyarray(depth_frame.get_data()).copy()
                color_image = np.asanyarray(color_frame.get_data()).copy()
                left_ir_image = np.asanyarray(frames.get_infrared_frame(1).get_data()).copy()
                right_ir_image = np.asanyarray(frames.get_infrared_frame(2).get_data()).copy()
                # Apply colormap to visualize depth data
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.03), 
                    cv2.COLORMAP_JET
                )

                depth_colormap_dim = depth_colormap.shape
                color_colormap_dim = color_image.shape

                if depth_colormap_dim != color_colormap_dim:
                    resized_color_image = cv2.resize(
                        color_image,
                        dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
                        interpolation=cv2.INTER_AREA,
                    )
                    color_image = resized_color_image
                self.realsense_deque.append((time.monotonic_ns(), color_image, depth_colormap, left_ir_image, right_ir_image))
                # print('adding realsense!')

        except Exception as e:
            print(f"Error in reading data: {e}")

        return None

    def save_images(self, save_dir, frame_idx):
        """
        Save all images (color, depth, left IR, right IR) to the specified directory.
        
        Args:
            save_dir (str): Directory to save images
            frame_idx (int): Frame index for naming files
        """
        if not self.realsense_deque:
            return
        
        # Get the latest frame
        _, color_image, depth_colormap, left_ir_image, right_ir_image = self.realsense_deque[-1]
        
        # Create save directory if it doesn't exist
        os.makedirs(save_dir, exist_ok=True)
        
        # Save color image
        color_path = os.path.join(save_dir, f"color_{frame_idx:06d}.png")
        cv2.imwrite(color_path, color_image)
        
        # Save depth colormap
        depth_path = os.path.join(save_dir, f"depth_{frame_idx:06d}.png")
        cv2.imwrite(depth_path, depth_colormap)
        
        # Save IR images (mono8 format)
        left_ir_path = os.path.join(save_dir, f"ir_left_{frame_idx:06d}.png")
        cv2.imwrite(left_ir_path, left_ir_image)
        
        right_ir_path = os.path.join(save_dir, f"ir_right_{frame_idx:06d}.png")
        cv2.imwrite(right_ir_path, right_ir_image)
