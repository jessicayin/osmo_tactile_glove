"""
ros2 bag topics
        # "/bowie/mag/index/mag0",
        # "/bowie/mag/index/mag1",
        # "/bowie/mag/middle/mag0",
        # "/bowie/mag/middle/mag1",
        # "/bowie/mag/ring/mag0",
        # "/bowie/mag/ring/mag1",
        # "/bowie/mag/pinky/mag0",
        # "/bowie/mag/pinky/mag1",
        # "/bowie/mag/thumb/mag0",
        # "/bowie/mag/thumb/mag1",
        "/bowie/synced"

/bowie/synced message format
index_mag0_ts, mag0_x, mag0_y, mag0_z,
index_mag1_ts, mag1_x, mag1_y, mag1_z,
index_imu_ts, imu_x, imu_y, imu_z, imu_w,
middle_mag0_ts, mag0_x, mag0_y, mag0_z,
middle_mag1_ts, mag1_x, mag1_y, mag1_z,
middle_imu_ts, imu_x, imu_y, imu_z, imu_w,
ring_mag0_ts, mag0_x, mag0_y, mag0_z,
ring_mag1_ts, mag1_x, mag1_y, mag1_z,
ring_imu_ts, imu_x, imu_y, imu_z, imu_w,
pinky_mag0_ts, mag0_x, mag0_y, mag0_z,
pinky_mag1_ts, mag1_x, mag1_y, mag1_z,
pinky_imu_ts, imu_x, imu_y, imu_z, imu_w,
        
/realsense/depth
/realsense/left_ir
/realsense/rgb
/realsense/right_ir

"""


import rosbag2_py
import pickle
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import numpy as np
from tqdm import tqdm
import hydra
from omegaconf import DictConfig
import os
import matplotlib.pyplot as plt
from scipy import interpolate
import bisect
# from sensor_msgs.msg import Image

def read_bag_to_list(bag_path):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    bridge = CvBridge()

    # Store data with timestamps for proper alignment
    rgb_data = []          # (timestamp, image)
    left_ir_data = []      # (timestamp, image)  
    right_ir_data = []     # (timestamp, image)
    depth_data = []        # (timestamp, image)
    raw_synced_mags = []
    raw_synced_imus = []
    synced_mags = []       # this will hold the synced mag data without timestamps


    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == '/realsense/rgb':
            img_msg = Image()
            img_msg = deserialize_message(data, Image)
            img_array = bridge.imgmsg_to_cv2(img_msg, "bgr8")
            rgb_data.append((t, img_array))
        elif topic == "/realsense/left_ir":
            img_msg = Image()
            img_msg = deserialize_message(data, Image)
            img_array = bridge.imgmsg_to_cv2(img_msg, "mono8")
            left_ir_data.append((t, img_array))
        elif topic == "/realsense/right_ir":
            img_msg = Image()
            img_msg = deserialize_message(data, Image)
            img_array = bridge.imgmsg_to_cv2(img_msg, "mono8")
            right_ir_data.append((t, img_array))
        elif topic in "/bowie/synced": #assumes only mag data!
            bowie_msg = Float32MultiArray()
            bowie_msg = deserialize_message(data, Float32MultiArray)
            mag_data = bowie_msg.data[0:8] + bowie_msg.data[13:21] + bowie_msg.data[26:34] + bowie_msg.data[39:47] + bowie_msg.data[52:60]
            imu_data = bowie_msg.data[8:13] + bowie_msg.data[21:26] + bowie_msg.data[34:39] + bowie_msg.data[47:52] + bowie_msg.data[60:]
            no_ts_mags = np.asarray(mag_data).reshape((10,4))[:,1:].flatten().tolist() #remove individual sensor timestamps for postprocessing
            synced_mags.append((no_ts_mags))
            raw_synced_mags.append((t, mag_data))
            raw_synced_imus.append((t, imu_data))

    # Extract timestamps and data for return
    # Use RGB as reference timestamps
    rgb_timestamps = [item[0] for item in rgb_data]
    rgb_images = [item[1] for item in rgb_data]
    
    print(f"Raw data counts:")
    print(f"  RGB: {len(rgb_data)} frames")
    print(f"  Left IR: {len(left_ir_data)} frames") 
    print(f"  Right IR: {len(right_ir_data)} frames")
    print(f"  Depth: {len(depth_data)} frames")
    print(f"  Mag data: {len(synced_mags)} samples")

    return (rgb_timestamps, rgb_images, left_ir_data, right_ir_data, raw_synced_mags, raw_synced_imus, synced_mags)


def closest_idx(before_idx, after_idx, before_ts, after_ts, target_ts):
    if abs(target_ts - before_ts) <= abs(target_ts - after_ts):
        return before_idx
    else:
        return after_idx

def align_realsense_cams(rgb_timestamps, rgb_data, left_ir_data, right_ir_data):
    rs_ts = np.asarray(rgb_timestamps)
    rs_ts_start = rs_ts[0]

    aligned_left_ir = []
    aligned_right_ir = []
    time_alignment_error_left = []
    time_alignment_error_right = []

    left_ir_ts = np.asarray([item[0] for item in left_ir_data])
    left_ir_ts_start = left_ir_ts[0]
    right_ir_ts = np.asarray([item[0] for item in right_ir_data])
    right_ir_ts_start = right_ir_ts[0]

    left_offset = (left_ir_ts_start - rs_ts_start)/1e9
    right_offset = (right_ir_ts_start - rs_ts_start)/1e9


    #find left start index
    if left_offset < 0: #left ir started before realsense rgb
        target_start_ts = rs_ts_start #time
        for i in range(len(left_ir_ts)):
            if left_ir_ts[i] > target_start_ts:
                left_ir_start_before_idx = i-1
                left_ir_start_after_idx = i
                break
        closest_start_idx = closest_idx(left_ir_start_before_idx, left_ir_start_after_idx, left_ir_data[left_ir_start_before_idx][0], left_ir_data[left_ir_start_after_idx][0], target_start_ts)

        left_ir_ts = left_ir_ts[closest_start_idx:]
        left_ir_data = left_ir_data[closest_start_idx:]
        left_ir_ts_start = left_ir_ts[0]
    
    if right_offset < 0: #right ir started before realsense rgb
        target_start_ts = rs_ts_start #time
        for i in range(len(right_ir_ts)):
            if right_ir_ts[i] > target_start_ts:
                right_ir_start_before_idx = i-1
                right_ir_start_after_idx = i
                break
        closest_start_idx = closest_idx(right_ir_start_before_idx, right_ir_start_after_idx, right_ir_data[right_ir_start_before_idx][0], right_ir_data[right_ir_start_after_idx][0], target_start_ts)

        right_ir_ts = right_ir_ts[closest_start_idx:]
        right_ir_data = right_ir_data[closest_start_idx:]
        right_ir_ts_start = right_ir_ts[0]
        
    assert len(right_ir_ts) == len(right_ir_data), "Right IR timestamps and data length mismatch after trimming!"
    assert  len(left_ir_ts) == len(left_ir_data), "Left IR timestamps and data length mismatch after trimming!"

    # import ipdb; ipdb.set_trace()
    

    print(f"Time offset between Left IR and RGB: {(left_ir_ts_start - rs_ts_start)/1e9} s")
    print(f"Time offset between Right IR and RGB: {(right_ir_ts_start - rs_ts_start)/1e9} s")


    # if left_ir_ts_start < rs_ts_start:
    # target_start_ts = rs_ts_start #time
    # for i in range(len(left_ir_data)):
    #     if left_ir_data[i][0] > target_start_ts:
    #         left_ir_start_before_idx = i-1
    #         left_ir_start_after_idx = i
    #         break
    # # closest_start_idx = closest_idx(left_ir_start_before_idx, left_ir_start_after_idx, left_ir_data[left_ir_start_before_idx][0], left_ir_data[left_ir_start_after_idx][0], target_start_ts)
    # error_ts = (left_ir_data[left_ir_start_before_idx][0]- rs_ts_start)/1e9
    # print(f"Error between starting timestamps (Left IR to RGB): {(left_ir_data[left_ir_start_before_idx][0]- rs_ts_start)/1e9} s")
    # assert abs(error_ts) < 0.03, "Starting timestamps are too far apart!"

    # left_ir_data = left_ir_data[left_ir_start_before_idx:] #trim left ir data to start just before rgb start time
    
    # # if right_ir_ts_start < rs_ts_start:
    # target_start_ts = rs_ts_start #time
    # for i in range(len(right_ir_data)):
    #     if right_ir_data[i][0] > target_start_ts:
    #         right_ir_start_before_idx = i-1
    #         right_ir_start_after_idx = i
    #         break
    # # closest_start_idx = closest_idx(right_ir_start_before_idx, right_ir_start_after_idx, right_ir_data[right_ir_start_before_idx][0], right_ir_data[right_ir_start_after_idx][0], target_start_ts)
    # error_ts = (right_ir_data[right_ir_start_before_idx][0]- rs_ts_start)/1e9
    # print(f"Error between starting timestamps (Right IR to RGB): {(right_ir_data[right_ir_start_before_idx][0]- rs_ts_start)/1e9} s")
    # assert abs(error_ts) < 0.03, "Starting timestamps are too far apart!"
    
    # right_ir_data = right_ir_data[right_ir_start_before_idx:] #trim right ir data to start at or just before rgb start time


    print(f"Aligning Left IR to RGB...")
    for i in range(rs_ts.shape[0]):
        target_ts = rs_ts[i]
        #find closest left ir timestamp to realsense rgb timestamp
        for j in range(len(left_ir_data)):
            if left_ir_data[j][0] > target_ts:
                left_ir_before_idx = j-1
                # left_ir_after_idx = j
                break
        aligned_left_ir.append(left_ir_data[left_ir_before_idx][1])
        aligned_left_ir_ts = left_ir_data[left_ir_before_idx][0]
        time_diff = abs(aligned_left_ir_ts - target_ts)/1e9
        time_alignment_error_left.append(time_diff)
    print(f"Average time alignment error (Left IR to RGB): {np.mean(time_alignment_error_left)} s")
    # print(f"Max time alignment error (Left IR to RGB): {np.max(time_alignment_error_left)} s")
    # import ipdb; ipdb.set_trace()
    # assert max(time_alignment_error_left) < 0.1, "Left IR alignment error too high!"
    

    print(f"Aligning Right IR to RGB...")
    for i in range(rs_ts.shape[0]):
        target_ts = rs_ts[i]
        #find closest right ir timestamp to realsense rgb timestamp
        for j in range(len(right_ir_data)):
            if right_ir_data[j][0] > target_ts:
                right_ir_before_idx = j-1
                # right_ir_after_idx = j
                break
        aligned_right_ir.append(right_ir_data[right_ir_before_idx][1])
        aligned_right_ir_ts = right_ir_data[right_ir_before_idx][0]
        time_diff = abs(aligned_right_ir_ts - target_ts)/1e9
        time_alignment_error_right.append(time_diff)
    print(f"Average time alignment error (Right IR to RGB): {np.mean(time_alignment_error_right)} s")
    # print(f"Max time alignment error (Right IR to RGB): {np.max(time_alignment_error_right)} s")
    # assert max(time_alignment_error_right) < 0.1, "Right IR alignment error too high!"

    assert len(rgb_data) == len(aligned_left_ir) == len(aligned_right_ir), "Aligned data lengths do not match!"

    return aligned_left_ir, aligned_right_ir



def align_bowie_to_realsense(raw_synced_mags, rgb_timestamps, rgb_images, left_ir_data, right_ir_data):
    raw_synced_mags = np.asarray(raw_synced_mags, dtype="object")
    bowie_ts = raw_synced_mags[:,0]
    bowie_ts_start = bowie_ts[0]
    bowie_data = raw_synced_mags[:,1]

    rs_ts = np.asarray(rgb_timestamps)
    rs_ts_start = rs_ts[0]

    offset = rs_ts_start - bowie_ts_start
    offset = offset / 1e9
    bowie_start_idx = 0
    rs_start_idx = 0

    aligned_bowie = []
    time_alignment_error = []

    print(f"Time offset between Bowie and Realsense: {offset} ns")
    if offset < 0: #bowie started later than realsense:
        #find index where bowie timestamp > realsense start time
        target_start_ts = bowie_ts_start #time
        for i in range(rs_ts.shape[0]):
            if rs_ts[i] > target_start_ts:
                rs_start_before_idx = i-1
                rs_start_after_idx = i
                break
        closest_start_idx = closest_idx(rs_start_before_idx, rs_start_after_idx, rs_ts[rs_start_before_idx], rs_ts[rs_start_after_idx], target_start_ts)
        error_ts = (rs_ts[closest_start_idx]- bowie_ts_start)/1e9
        print(f"Error between closest starting timestamps: {(rs_ts[closest_start_idx]- bowie_ts_start)/1e9} s")
        assert error_ts < 0.03, "Starting timestamps are too far apart!"
    else: #bowie started earlier than realsense
        # target_start_ts = rs_ts_start #time
        # for i in range(len(raw_synced_mags)):
        #     if raw_synced_mags[i][0] > target_start_ts:
        #         bowie_start_before_idx = i-1
        #         bowie_start_after_idx = i
        #         break
        
        # closest_start_idx = closest_idx(bowie_start_before_idx, bowie_start_after_idx, bowie_ts[bowie_start_before_idx], bowie_ts[bowie_start_after_idx], target_start_ts)
        # error_ts = (bowie_ts[closest_start_idx]- rs_ts_start)/1e9
        # assert error_ts < 0.03, "Starting timestamps are too far apart!"
        # print(f"Error between closest starting timestamps: {(bowie_ts[closest_start_idx]- rs_ts_start)/1e9} s")
        closest_start_idx = 0

    rs_start_idx = closest_start_idx
    aligned_rgbs = rgb_images[closest_start_idx:]
    aligned_rs_ts = rs_ts[closest_start_idx:]
    aligned_left_ir = left_ir_data[closest_start_idx:]
    aligned_right_ir = right_ir_data[closest_start_idx:]
    
    

    for i in range(aligned_rs_ts.shape[0]):
        target_ts = aligned_rs_ts[i]
        #find closest bowie timestamp to realsense timestamp
        for j in range(bowie_ts.shape[0]):
            if bowie_ts[j] > target_ts:
                bowie_before_idx = j-1
                # bowie_after_idx = j
                break
        # closest_ts_idx = closest_idx(bowie_before_idx, bowie_after_idx, bowie_ts[bowie_before_idx], bowie_ts[bowie_after_idx], target_ts)
        aligned_bowie.append(bowie_data[bowie_before_idx])
        aligned_bowie_ts = bowie_ts[bowie_before_idx]
        time_diff = abs(aligned_bowie_ts - target_ts)/1e9
        time_alignment_error.append(time_diff)
    
    assert len(aligned_bowie) == len(aligned_rgbs) == len(aligned_rs_ts) == len(aligned_left_ir) == len(aligned_right_ir), "Final aligned data lengths do not match!"
    print(f"Final aligned data counts:")
    print(f"  RGB: {len(aligned_rgbs)} frames")
    print(f"  Left IR: {len(aligned_left_ir)} frames") 
    print(f"  Right IR: {len(aligned_right_ir)} frames")
    print(f"  Mag data: {len(aligned_bowie)} samples")

    print(f"Average time alignment error: {np.mean(time_alignment_error)} s")
    print(f"Max time alignment error: {np.max(time_alignment_error)} s")

    return aligned_bowie, aligned_rgbs, aligned_left_ir, aligned_right_ir

        




    #this case does not matter because we will align to realsense ts

    




@hydra.main(version_base=None, config_path="../../../glove/labs/glove2robot/config", config_name="config_extract_hamer")
def main(cfg: DictConfig):
    # Get data paths from configuration
    data_paths = cfg.paths.data_path
    
    # Handle both single path and list of paths
    if isinstance(data_paths, str):
        data_paths = [data_paths]
    
    print(f"Processing {len(data_paths)} data directories from config:")
    for path in data_paths:
        print(f"  - {path}")
    
    for bag_path in tqdm(data_paths, desc="Processing directories"):
        # Remove trailing slash if present
        bag_path = bag_path.rstrip('/')
        
        # Check if directory exists
        if not os.path.exists(bag_path):
            print(f"Warning: Directory {bag_path} does not exist, skipping...")
            continue
            
        print(f"\nProcessing: {bag_path}")
        
        try:
            # Read raw data from bag - now with proper timestamps for all streams
            rgb_timestamps, rgb_images, left_ir_data, right_ir_data, raw_synced_mags, raw_synced_imus, synced_mags = read_bag_to_list(bag_path)
            # final_aligned_mags = []
            cam_aligned_left_ir, cam_aligned_right_ir = align_realsense_cams(rgb_timestamps, rgb_images, left_ir_data, right_ir_data)
            aligned_bowie, aligned_rgbs, aligned_left_ir, aligned_right_ir = align_bowie_to_realsense(raw_synced_mags, rgb_timestamps, rgb_images, cam_aligned_left_ir, cam_aligned_right_ir)
            # import ipdb; ipdb.set_trace()
            with open(bag_path+"/rgbs_aligned.pkl", 'wb') as f:
                pickle.dump(aligned_rgbs, f)
            with open(bag_path+"/left_ir_aligned.pkl", 'wb') as f:
                pickle.dump(aligned_left_ir, f)
            with open(bag_path+"/right_ir_aligned.pkl", 'wb') as f:
                pickle.dump(aligned_right_ir, f)
            with open(bag_path+"/synced_mags_aligned.pkl", 'wb') as f:
                pickle.dump(aligned_bowie, f)
            
            print(f"Data saved successfully: {bag_path}")
        
        except Exception as e:
            print(f"Error processing {bag_path}: {e}")

if __name__ == "__main__":
    main()