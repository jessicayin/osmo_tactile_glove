# align realsense camera to training data by showing difference between current and past images

import cv2
import numpy as np
import matplotlib.pyplot as plt
import pyrealsense2 as rs
import pickle


def main():
    # Load images files
    with open("/home/gumdev/human2robot/data/wipe_sea3/rosbag2_2025_09_14-00_58_10/rgbs_aligned.pkl", "rb") as f:
        rgbs = pickle.load(f)
    with open("/home/gumdev/human2robot/data/wipe_sea3/rosbag2_2025_09_14-00_58_10/left_ir_aligned.pkl", "rb") as f:
        left_irs = pickle.load(f)
    with open("/home/gumdev/human2robot/data/wipe_sea3/rosbag2_2025_09_14-00_58_10/right_ir_aligned.pkl", "rb") as f:
        right_irs = pickle.load(f)
    
    avg_rgb = np.mean(rgbs, axis=0).astype(np.uint8)
    avg_left_ir = np.mean(left_irs, axis=0).astype(np.uint8)
    avg_right_ir = np.mean(right_irs, axis=0).astype(np.uint8)
    avg_left_ir_color = cv2.cvtColor(avg_left_ir, cv2.COLOR_GRAY2BGR)
    avg_right_ir_color = cv2.cvtColor(avg_right_ir, cv2.COLOR_GRAY2BGR)
    # import ipdb; ipdb.set_trace()
    train_imgs = np.hstack((avg_rgb, avg_left_ir_color, avg_right_ir_color))
    # cv2.imshow("Training Data Average (RGB | Left IR | Right IR)", train_imgs)
    # cv2.waitKey(1)

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.infrared, 1, 1280, 720, rs.format.y8, 30)
    config.enable_stream(rs.stream.infrared, 2, 1280, 720, rs.format.y8, 30)

    # Start streaming
    profile = pipeline.start(config)

    # Create an align object
    align_to = rs.stream.color
    align = rs.align(align_to)

    color_sensor = profile.get_device().query_sensors()[1] # Index 1 for color
    color_sensor.set_option(rs.option.enable_auto_exposure, True)
    # color_sensor.set_option(rs.option.exposure, 200.0)          # example value (tune)

    depth_sensor = profile.get_device().query_sensors()[0] # Index 0 for depth
    depth_sensor.set_option(rs.option.emitter_enabled, 0) # 0 for off

    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            left_ir_frame = frames.get_infrared_frame(1)
            right_ir_frame = frames.get_infrared_frame(2)

            if not color_frame or not depth_frame or not left_ir_frame or not right_ir_frame:
                return

            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            left_ir_image = np.asanyarray(left_ir_frame.get_data())
            right_ir_image = np.asanyarray(right_ir_frame.get_data())

            left_ir_color_image = cv2.cvtColor(left_ir_image, cv2.COLOR_GRAY2BGR)
            right_ir_color_image = cv2.cvtColor(right_ir_image, cv2.COLOR_GRAY2BGR)

            # Stack both images horizontally
            images = np.hstack((color_image, left_ir_color_image, right_ir_color_image))
            display_images = train_imgs - images

            # Show images
            cv2.namedWindow('Color, Left IR, Right IR', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('Color, Left IR, Right IR', display_images)
            key = cv2.waitKey(1)
            if key == 27:
                break
    finally:
        # Stop streaming
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()