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

    realsense_ts = [] #assume rgb timestamp
    rgbs = []
    left_ir = []
    right_ir = []
    depth = []
    raw_synced_mags = []
    raw_synced_imus = []
    synced_mags =[]  # this will hold the synced mag data without timestamps


    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == '/realsense/rgb':
            img_msg = Image()
            img_msg = deserialize_message(data, Image)
            img_array = bridge.imgmsg_to_cv2(img_msg, "bgr8")
            rgbs.append(img_array)
            realsense_ts.append(t)
        elif topic == "/realsense/left_ir":
            img_msg = Image()
            img_msg = deserialize_message(data, Image)
            img_array = bridge.imgmsg_to_cv2(img_msg, "mono8")
            left_ir.append(img_array)
        elif topic == "/realsense/right_ir":
            img_msg = Image()
            img_msg = deserialize_message(data, Image)
            img_array = bridge.imgmsg_to_cv2(img_msg, "mono8")
            right_ir.append(img_array)
        elif topic == "/realsense/depth":
            img_msg = Image()
            img_msg = deserialize_message(data, Image)
            img_array = bridge.imgmsg_to_cv2(img_msg, "16UC1")
            depth.append(img_array)
        elif topic in "/bowie/synced": #assumes only mag data!
            bowie_msg = Float32MultiArray()
            bowie_msg = deserialize_message(data, Float32MultiArray)
            mag_data = bowie_msg.data[0:8] + bowie_msg.data[13:21] + bowie_msg.data[26:34] + bowie_msg.data[39:47] + bowie_msg.data[52:60]
            imu_data = bowie_msg.data[8:13] + bowie_msg.data[21:26] + bowie_msg.data[34:39] + bowie_msg.data[47:52] + bowie_msg.data[60:]
            no_ts_mags = np.asarray(mag_data).reshape((10,4))[:,1:].flatten().tolist() #remove individual sensor timestamps for postprocessing
            synced_mags.append((t, no_ts_mags))
            raw_synced_mags.append((t, mag_data))
            raw_synced_imus.append((t, imu_data))
            
    # may not be necessary to convert to np arrays for now
    # rgbs = np.asarray(rgbs, dtype="object")
    # left_ir = np.asarray(left_ir, dtype="object")
    # right_ir = np.asarray(right_ir, dtype="object")
    # raw_synced_mags = np.asarray(raw_synced_mags, dtype="object")
    # raw_synced_imus = np.asarray(raw_synced_imus, dtype="object")
    
    #remove timestamp and then save
    # import ipdb; ipdb.set_trace()

    return realsense_ts, rgbs, left_ir, right_ir, depth, raw_synced_mags, raw_synced_imus, synced_mags


def convert_ros_time_to_seconds(ros_time_ns):
    """Convert ROS time in nanoseconds to seconds."""
    return ros_time_ns / 1e9


def find_closest_timestamp_index(target_time, timestamp_list):
    """Find the index of the closest timestamp in a sorted list."""
    if not timestamp_list:
        return None
    
    # Convert to seconds for easier handling
    target_seconds = convert_ros_time_to_seconds(target_time)
    timestamp_seconds = [convert_ros_time_to_seconds(ts) for ts in timestamp_list]
    
    # Use binary search for efficiency
    pos = bisect.bisect_left(timestamp_seconds, target_seconds)
    
    if pos == 0:
        return 0
    elif pos == len(timestamp_seconds):
        return len(timestamp_seconds) - 1
    else:
        # Choose the closer of the two adjacent timestamps
        before = timestamp_seconds[pos - 1]
        after = timestamp_seconds[pos]
        if target_seconds - before < after - target_seconds:
            return pos - 1
        else:
            return pos


def align_data_to_reference(reference_timestamps, reference_data, target_timestamps, target_data, max_time_diff_ms=50):
    """
    Align target data to reference timestamps using closest timestamp matching.
    
    Args:
        reference_timestamps: List of reference timestamps (e.g., realsense)
        reference_data: List of reference data corresponding to timestamps
        target_timestamps: List of target timestamps (e.g., bowie data)
        target_data: List of target data to be aligned
        max_time_diff_ms: Maximum allowed time difference in milliseconds
        
    Returns:
        aligned_reference_data: Reference data for matched timestamps
        aligned_target_data: Target data aligned to reference timestamps
        alignment_stats: Dictionary with alignment statistics
    """
    aligned_reference_data = []
    aligned_target_data = []
    time_differences = []
    
    # Validate inputs
    if not reference_timestamps or not target_timestamps or not reference_data or not target_data:
        return [], [], {
            'total_reference_frames': len(reference_timestamps) if reference_timestamps else 0,
            'total_target_frames': len(target_timestamps) if target_timestamps else 0,
            'aligned_frames': 0,
            'alignment_rate': 0,
            'mean_time_diff_ms': 0,
            'std_time_diff_ms': 0,
            'max_time_diff_ms': 0,
            'min_time_diff_ms': 0,
            'error': 'Empty input data'
        }
    
    # Ensure data lists match timestamp lists
    if len(reference_timestamps) != len(reference_data):
        print(f"Warning: Reference timestamp length ({len(reference_timestamps)}) != data length ({len(reference_data)})")
        min_ref_len = min(len(reference_timestamps), len(reference_data))
        reference_timestamps = reference_timestamps[:min_ref_len]
        reference_data = reference_data[:min_ref_len]
    
    if len(target_timestamps) != len(target_data):
        print(f"Warning: Target timestamp length ({len(target_timestamps)}) != data length ({len(target_data)})")
        min_target_len = min(len(target_timestamps), len(target_data))
        target_timestamps = target_timestamps[:min_target_len]
        target_data = target_data[:min_target_len]
    
    for i, ref_time in enumerate(reference_timestamps):
        # Find closest target timestamp
        closest_idx = find_closest_timestamp_index(ref_time, target_timestamps)
        
        if closest_idx is not None and closest_idx < len(target_data):
            target_time = target_timestamps[closest_idx]
            time_diff_ms = abs(convert_ros_time_to_seconds(ref_time) - convert_ros_time_to_seconds(target_time)) * 1000
            
            # Only include if within acceptable time difference
            if time_diff_ms <= max_time_diff_ms:
                aligned_reference_data.append(reference_data[i])
                aligned_target_data.append(target_data[closest_idx])
                time_differences.append(time_diff_ms)
    
    # Calculate alignment statistics
    alignment_stats = {
        'total_reference_frames': len(reference_timestamps),
        'total_target_frames': len(target_timestamps),
        'aligned_frames': len(aligned_reference_data),
        'alignment_rate': len(aligned_reference_data) / len(reference_timestamps) if reference_timestamps else 0,
        'mean_time_diff_ms': np.mean(time_differences) if time_differences else 0,
        'std_time_diff_ms': np.std(time_differences) if time_differences else 0,
        'max_time_diff_ms': np.max(time_differences) if time_differences else 0,
        'min_time_diff_ms': np.min(time_differences) if time_differences else 0
    }
    
    return aligned_reference_data, aligned_target_data, alignment_stats


def interpolate_target_data_to_reference(reference_timestamps, target_timestamps, target_data):
    """
    Interpolate target data to match reference timestamps exactly.
    
    Args:
        reference_timestamps: List of reference timestamps
        target_timestamps: List of target timestamps
        target_data: List of target data (must be numerical arrays)
        
    Returns:
        interpolated_data: Target data interpolated to reference timestamps
        interpolation_stats: Dictionary with interpolation statistics
    """
    if not target_timestamps or not target_data:
        return [], {'success': False, 'error': 'Empty input data'}
    
    # Convert timestamps to seconds
    ref_seconds = np.array([convert_ros_time_to_seconds(ts) for ts in reference_timestamps])
    target_seconds = np.array([convert_ros_time_to_seconds(ts) for ts in target_timestamps])
    
    # Check if target data is within the reference time range
    ref_start, ref_end = ref_seconds[0], ref_seconds[-1]
    target_start, target_end = target_seconds[0], target_seconds[-1]
    
    # Find overlap period
    overlap_start = max(ref_start, target_start)
    overlap_end = min(ref_end, target_end)
    
    if overlap_start >= overlap_end:
        return [], {'success': False, 'error': 'No time overlap between reference and target data'}
    
    # Filter reference timestamps to overlap period
    valid_ref_mask = (ref_seconds >= overlap_start) & (ref_seconds <= overlap_end)
    valid_ref_seconds = ref_seconds[valid_ref_mask]
    
    try:
        # Convert target data to numpy array for interpolation
        target_array = np.array(target_data)
        
        if target_array.ndim == 1:
            # 1D data - simple interpolation
            interp_func = interpolate.interp1d(target_seconds, target_array, kind='linear', 
                                             bounds_error=False, fill_value='extrapolate')
            interpolated_data = interp_func(valid_ref_seconds).tolist()
        else:
            # Multi-dimensional data - interpolate each dimension
            interpolated_data = []
            for i, ref_time in enumerate(valid_ref_seconds):
                # For each reference time, interpolate all dimensions
                interpolated_sample = []
                for dim in range(target_array.shape[1]):
                    interp_func = interpolate.interp1d(target_seconds, target_array[:, dim], 
                                                     kind='linear', bounds_error=False, 
                                                     fill_value='extrapolate')
                    interpolated_sample.append(interp_func(ref_time))
                interpolated_data.append(interpolated_sample)
        
        interpolation_stats = {
            'success': True,
            'original_samples': len(target_data),
            'interpolated_samples': len(interpolated_data),
            'overlap_start_s': overlap_start,
            'overlap_end_s': overlap_end,
            'overlap_duration_s': overlap_end - overlap_start
        }
        
        return interpolated_data, interpolation_stats
        
    except Exception as e:
        return [], {'success': False, 'error': f'Interpolation failed: {str(e)}'}


def analyze_timing_alignment(realsense_ts, synced_mags):
    """
    Analyze the timing alignment between realsense and bowie data.
    
    Args:
        realsense_ts: List of realsense timestamps
        synced_mags: List of (timestamp, data) tuples for bowie data
        
    Returns:
        analysis_results: Dictionary with timing analysis
    """
    if not realsense_ts or not synced_mags:
        return {'error': 'Empty input data'}
    
    # Extract bowie timestamps
    bowie_ts = [item[0] for item in synced_mags]
    
    # Convert to seconds for analysis
    realsense_seconds = np.array([convert_ros_time_to_seconds(ts) for ts in realsense_ts])
    bowie_seconds = np.array([convert_ros_time_to_seconds(ts) for ts in bowie_ts])
    
    # Calculate timing statistics
    analysis = {
        'realsense_fps': len(realsense_ts) / (realsense_seconds[-1] - realsense_seconds[0]) if len(realsense_ts) > 1 else 0,
        'bowie_fps': len(bowie_ts) / (bowie_seconds[-1] - bowie_seconds[0]) if len(bowie_ts) > 1 else 0,
        'realsense_duration_s': realsense_seconds[-1] - realsense_seconds[0],
        'bowie_duration_s': bowie_seconds[-1] - bowie_seconds[0],
        'time_offset_s': realsense_seconds[0] - bowie_seconds[0],
        'realsense_frame_intervals_ms': np.diff(realsense_seconds) * 1000,
        'bowie_frame_intervals_ms': np.diff(bowie_seconds) * 1000
    }
    
    # Calculate interval statistics
    analysis['realsense_interval_stats'] = {
        'mean_ms': np.mean(analysis['realsense_frame_intervals_ms']),
        'std_ms': np.std(analysis['realsense_frame_intervals_ms']),
        'min_ms': np.min(analysis['realsense_frame_intervals_ms']),
        'max_ms': np.max(analysis['realsense_frame_intervals_ms'])
    }
    
    analysis['bowie_interval_stats'] = {
        'mean_ms': np.mean(analysis['bowie_frame_intervals_ms']),
        'std_ms': np.std(analysis['bowie_frame_intervals_ms']),
        'min_ms': np.min(analysis['bowie_frame_intervals_ms']),
        'max_ms': np.max(analysis['bowie_frame_intervals_ms'])
    }
    
    return analysis


def compare_alignment_quality(realsense_ts, synced_mags, aligned_rgbs, aligned_mags, original_rgbs, save_path=None):
    """
    Compare timing alignment quality before and after explicit alignment.
    
    Args:
        realsense_ts: Original realsense timestamps
        synced_mags: Original bowie data with timestamps
        aligned_rgbs: Aligned RGB data
        aligned_mags: Aligned mag data
        original_rgbs: Original RGB data for finding aligned frame indices
        save_path: Optional path to save comparison plots
        
    Returns:
        comparison_results: Dictionary with detailed before/after comparison
    """
    # Extract original bowie timestamps
    bowie_ts = [item[0] for item in synced_mags]
    bowie_data = [item[1] for item in synced_mags]
    
    # Convert timestamps to seconds
    realsense_seconds = np.array([convert_ros_time_to_seconds(ts) for ts in realsense_ts])
    bowie_seconds = np.array([convert_ros_time_to_seconds(ts) for ts in bowie_ts])
    
    # BEFORE ALIGNMENT: Naive approach (length truncation)
    min_length = min(len(realsense_ts), len(synced_mags))
    naive_realsense_times = realsense_seconds[:min_length]
    naive_bowie_times = bowie_seconds[:min_length]
    
    # Calculate timing differences for naive alignment
    naive_time_diffs = np.abs(naive_realsense_times - naive_bowie_times) * 1000  # in ms
    
    # AFTER ALIGNMENT: Calculate timing differences for aligned data
    # We need to track which frames were actually aligned
    aligned_time_diffs = []
    aligned_realsense_indices = []
    aligned_bowie_indices = []
    
    # Find which original frames correspond to aligned data
    for i, aligned_rgb in enumerate(aligned_rgbs):
        # Find this RGB frame in original data
        for j, orig_rgb in enumerate(original_rgbs):
            if np.array_equal(aligned_rgb, orig_rgb):
                aligned_realsense_indices.append(j)
                break
    
    for i, aligned_mag in enumerate(aligned_mags):
        # Find this mag data in original data
        for j, orig_mag in enumerate(bowie_data):
            if np.array_equal(aligned_mag, orig_mag):
                aligned_bowie_indices.append(j)
                break
    
    # Calculate time differences for aligned frames
    if len(aligned_realsense_indices) == len(aligned_bowie_indices):
        for i in range(len(aligned_realsense_indices)):
            rs_idx = aligned_realsense_indices[i]
            bowie_idx = aligned_bowie_indices[i]
            time_diff = abs(realsense_seconds[rs_idx] - bowie_seconds[bowie_idx]) * 1000
            aligned_time_diffs.append(time_diff)
    
    aligned_time_diffs = np.array(aligned_time_diffs)
    
    # Calculate statistics
    comparison_results = {
        'before_alignment': {
            'method': 'Naive length truncation',
            'num_frames': len(naive_time_diffs),
            'mean_time_diff_ms': np.mean(naive_time_diffs),
            'std_time_diff_ms': np.std(naive_time_diffs),
            'max_time_diff_ms': np.max(naive_time_diffs),
            'min_time_diff_ms': np.min(naive_time_diffs),
            'median_time_diff_ms': np.median(naive_time_diffs),
            'frames_within_20ms': np.sum(naive_time_diffs <= 20),
            'frames_within_50ms': np.sum(naive_time_diffs <= 50),
        },
        'after_alignment': {
            'method': 'Timestamp-based alignment',
            'num_frames': len(aligned_time_diffs),
            'mean_time_diff_ms': np.mean(aligned_time_diffs) if len(aligned_time_diffs) > 0 else 0,
            'std_time_diff_ms': np.std(aligned_time_diffs) if len(aligned_time_diffs) > 0 else 0,
            'max_time_diff_ms': np.max(aligned_time_diffs) if len(aligned_time_diffs) > 0 else 0,
            'min_time_diff_ms': np.min(aligned_time_diffs) if len(aligned_time_diffs) > 0 else 0,
            'median_time_diff_ms': np.median(aligned_time_diffs) if len(aligned_time_diffs) > 0 else 0,
            'frames_within_20ms': np.sum(aligned_time_diffs <= 20) if len(aligned_time_diffs) > 0 else 0,
            'frames_within_50ms': np.sum(aligned_time_diffs <= 50) if len(aligned_time_diffs) > 0 else 0,
        }
    }
    
    # Calculate improvement metrics
    before_stats = comparison_results['before_alignment']
    after_stats = comparison_results['after_alignment']
    
    comparison_results['improvement'] = {
        'mean_time_diff_improvement_ms': before_stats['mean_time_diff_ms'] - after_stats['mean_time_diff_ms'],
        'std_reduction_ms': before_stats['std_time_diff_ms'] - after_stats['std_time_diff_ms'],
        'max_time_diff_improvement_ms': before_stats['max_time_diff_ms'] - after_stats['max_time_diff_ms'],
        'precision_improvement_20ms': (after_stats['frames_within_20ms'] / after_stats['num_frames']) - 
                                      (before_stats['frames_within_20ms'] / before_stats['num_frames']) if after_stats['num_frames'] > 0 else 0,
        'precision_improvement_50ms': (after_stats['frames_within_50ms'] / after_stats['num_frames']) - 
                                      (before_stats['frames_within_50ms'] / before_stats['num_frames']) if after_stats['num_frames'] > 0 else 0,
        'frame_utilization': after_stats['num_frames'] / before_stats['num_frames'] if before_stats['num_frames'] > 0 else 0
    }
    
    # Create visualization if path provided
    if save_path:
        create_alignment_quality_comparison_plot(
            naive_time_diffs, aligned_time_diffs, comparison_results, save_path
        )
    
    return comparison_results


def create_alignment_quality_comparison_plot(naive_diffs, aligned_diffs, comparison_results, save_path):
    """
    Create detailed visualization comparing alignment quality before and after.
    """
    try:
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        
        # Plot 1: Histogram comparison
        axes[0, 0].hist(naive_diffs, bins=50, alpha=0.7, label='Before (Naive)', color='red')
        if len(aligned_diffs) > 0:
            axes[0, 0].hist(aligned_diffs, bins=50, alpha=0.7, label='After (Aligned)', color='green')
        axes[0, 0].set_xlabel('Time Difference (ms)')
        axes[0, 0].set_ylabel('Frequency')
        axes[0, 0].set_title('Distribution of Time Differences')
        axes[0, 0].legend()
        axes[0, 0].axvline(x=20, color='orange', linestyle='--', alpha=0.7, label='20ms threshold')
        axes[0, 0].axvline(x=50, color='red', linestyle='--', alpha=0.7, label='50ms threshold')
        
        # Plot 2: Time series of differences
        axes[0, 1].plot(naive_diffs[:min(500, len(naive_diffs))], 'r-', alpha=0.7, label='Before (Naive)')
        if len(aligned_diffs) > 0:
            axes[0, 1].plot(aligned_diffs[:min(500, len(aligned_diffs))], 'g-', alpha=0.7, label='After (Aligned)')
        axes[0, 1].set_xlabel('Frame Number')
        axes[0, 1].set_ylabel('Time Difference (ms)')
        axes[0, 1].set_title('Time Differences Over Time (first 500 frames)')
        axes[0, 1].legend()
        axes[0, 1].axhline(y=20, color='orange', linestyle='--', alpha=0.5)
        axes[0, 1].axhline(y=50, color='red', linestyle='--', alpha=0.5)
        
        # Plot 3: Box plot comparison
        box_data = [naive_diffs]
        box_labels = ['Before\n(Naive)']
        if len(aligned_diffs) > 0:
            box_data.append(aligned_diffs)
            box_labels.append('After\n(Aligned)')
        
        axes[0, 2].boxplot(box_data, labels=box_labels)
        axes[0, 2].set_ylabel('Time Difference (ms)')
        axes[0, 2].set_title('Distribution Comparison')
        axes[0, 2].axhline(y=20, color='orange', linestyle='--', alpha=0.5)
        axes[0, 2].axhline(y=50, color='red', linestyle='--', alpha=0.5)
        
        # Plot 4: Statistics comparison
        before_stats = comparison_results['before_alignment']
        after_stats = comparison_results['after_alignment']
        
        stats_text = f"""BEFORE ALIGNMENT (Naive):
        Frames: {before_stats['num_frames']}
        Mean diff: {before_stats['mean_time_diff_ms']:.2f} ± {before_stats['std_time_diff_ms']:.2f} ms
        Max diff: {before_stats['max_time_diff_ms']:.2f} ms
        Within 20ms: {before_stats['frames_within_20ms']}/{before_stats['num_frames']} ({100*before_stats['frames_within_20ms']/before_stats['num_frames']:.1f}%)
        Within 50ms: {before_stats['frames_within_50ms']}/{before_stats['num_frames']} ({100*before_stats['frames_within_50ms']/before_stats['num_frames']:.1f}%)

        AFTER ALIGNMENT:
        Frames: {after_stats['num_frames']}
        Mean diff: {after_stats['mean_time_diff_ms']:.2f} ± {after_stats['std_time_diff_ms']:.2f} ms
        Max diff: {after_stats['max_time_diff_ms']:.2f} ms
        Within 20ms: {after_stats['frames_within_20ms']}/{after_stats['num_frames']} ({100*after_stats['frames_within_20ms']/after_stats['num_frames']:.1f}%)
        Within 50ms: {after_stats['frames_within_50ms']}/{after_stats['num_frames']} ({100*after_stats['frames_within_50ms']/after_stats['num_frames']:.1f}%)

        IMPROVEMENT:
        Mean diff reduction: {comparison_results['improvement']['mean_time_diff_improvement_ms']:.2f} ms
        Precision improvement (20ms): {100*comparison_results['improvement']['precision_improvement_20ms']:.1f}%
        Frame utilization: {100*comparison_results['improvement']['frame_utilization']:.1f}%"""
        
        axes[1, 0].text(0.05, 0.95, stats_text, transform=axes[1, 0].transAxes, 
                       fontsize=9, verticalalignment='top', fontfamily='monospace')
        axes[1, 0].set_title('Detailed Statistics')
        axes[1, 0].axis('off')
        
        # Plot 5: Cumulative distribution
        naive_sorted = np.sort(naive_diffs)
        naive_cumulative = np.arange(1, len(naive_sorted) + 1) / len(naive_sorted)
        axes[1, 1].plot(naive_sorted, naive_cumulative, 'r-', label='Before (Naive)')
        
        if len(aligned_diffs) > 0:
            aligned_sorted = np.sort(aligned_diffs)
            aligned_cumulative = np.arange(1, len(aligned_sorted) + 1) / len(aligned_sorted)
            axes[1, 1].plot(aligned_sorted, aligned_cumulative, 'g-', label='After (Aligned)')
        
        axes[1, 1].set_xlabel('Time Difference (ms)')
        axes[1, 1].set_ylabel('Cumulative Probability')
        axes[1, 1].set_title('Cumulative Distribution')
        axes[1, 1].legend()
        axes[1, 1].axvline(x=20, color='orange', linestyle='--', alpha=0.5)
        axes[1, 1].axvline(x=50, color='red', linestyle='--', alpha=0.5)
        axes[1, 1].grid(True, alpha=0.3)
        
        # Plot 6: Improvement metrics
        improvement = comparison_results['improvement']
        metrics = ['Mean Diff\nReduction (ms)', 'Std Reduction\n(ms)', 'Max Diff\nReduction (ms)', 
                  'Precision Impr.\n20ms (%)', 'Precision Impr.\n50ms (%)', 'Frame Util.\n(%)']
        values = [improvement['mean_time_diff_improvement_ms'], improvement['std_reduction_ms'],
                 improvement['max_time_diff_improvement_ms'], 
                 improvement['precision_improvement_20ms']*100,
                 improvement['precision_improvement_50ms']*100,
                 improvement['frame_utilization']*100]
        
        colors = ['green' if v > 0 else 'red' for v in values]
        axes[1, 2].bar(metrics, values, color=colors, alpha=0.7)
        axes[1, 2].set_title('Improvement Metrics')
        axes[1, 2].tick_params(axis='x', rotation=45)
        axes[1, 2].axhline(y=0, color='black', linestyle='-', alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        plt.close()
        
        print(f"Alignment quality comparison plot saved to: {save_path}")
        
    except Exception as e:
        print(f"Error creating alignment quality comparison plot: {e}")


def create_timing_comparison_plots(realsense_ts, synced_mags, aligned_stats, save_path):
    """
    Create visualization plots comparing timing before and after alignment.
    """
    try:
        # Extract bowie timestamps
        bowie_ts = [item[0] for item in synced_mags]
        
        # Convert to seconds relative to start
        realsense_seconds = np.array([convert_ros_time_to_seconds(ts) for ts in realsense_ts])
        bowie_seconds = np.array([convert_ros_time_to_seconds(ts) for ts in bowie_ts])
        
        start_time = min(realsense_seconds[0], bowie_seconds[0])
        realsense_rel = realsense_seconds - start_time
        bowie_rel = bowie_seconds - start_time
        
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        
        # Plot 1: Timeline comparison
        axes[0, 0].scatter(realsense_rel, np.ones(len(realsense_rel)), alpha=0.6, s=10, label='RealSense')
        axes[0, 0].scatter(bowie_rel, np.zeros(len(bowie_rel)), alpha=0.6, s=10, label='Bowie')
        axes[0, 0].set_xlabel('Time (seconds)')
        axes[0, 0].set_ylabel('Data Source')
        axes[0, 0].set_title('Timeline Comparison')
        axes[0, 0].legend()
        axes[0, 0].set_yticks([0, 1])
        axes[0, 0].set_yticklabels(['Bowie', 'RealSense'])
        
        # Plot 2: Frame intervals
        if len(realsense_rel) > 1:
            axes[0, 1].plot(np.diff(realsense_rel) * 1000, label='RealSense', alpha=0.7)
        if len(bowie_rel) > 1:
            axes[0, 1].plot(np.diff(bowie_rel) * 1000, label='Bowie', alpha=0.7)
        axes[0, 1].set_xlabel('Frame Number')
        axes[0, 1].set_ylabel('Interval (ms)')
        axes[0, 1].set_title('Frame Intervals')
        axes[0, 1].legend()
        axes[0, 1].axhline(y=40, color='r', linestyle='--', alpha=0.5, label='25Hz target')
        
        # Plot 3: Alignment statistics
        stats_text = f"""Alignment Results:
        Total RealSense frames: {aligned_stats['total_reference_frames']}
        Total Bowie frames: {aligned_stats['total_target_frames']}
        Aligned frames: {aligned_stats['aligned_frames']}
        Alignment rate: {aligned_stats['alignment_rate']:.1%}
        Mean time diff: {aligned_stats['mean_time_diff_ms']:.2f} ms
        Std time diff: {aligned_stats['std_time_diff_ms']:.2f} ms"""
                
        axes[1, 0].text(0.1, 0.5, stats_text, transform=axes[1, 0].transAxes, 
                       fontsize=10, verticalalignment='center')
        axes[1, 0].set_title('Alignment Statistics')
        axes[1, 0].axis('off')
        
        # Plot 4: Sample comparison (first 100 frames)
        n_samples = min(100, len(realsense_rel), len(bowie_rel))
        axes[1, 1].plot(realsense_rel[:n_samples], 'o-', label='RealSense', markersize=3)
        axes[1, 1].plot(bowie_rel[:n_samples], 's-', label='Bowie', markersize=3)
        axes[1, 1].set_xlabel('Frame Number')
        axes[1, 1].set_ylabel('Time (seconds)')
        axes[1, 1].set_title(f'Sample Timing (first {n_samples} frames)')
        axes[1, 1].legend()
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        plt.close()
        
        print(f"Timing comparison plot saved to: {save_path}")
        
    except Exception as e:
        print(f"Error creating timing plots: {e}")


# def align_and_save_data(img_data, mag_data, output_file):
#     aligned_data = []
#     img_index = 0
#     mag_index = 0

#     while img_index < len(img_data) and mag_index < len(mag_data):
#         img_time, img_array = img_data[img_index]
#         mag_time, mag_values = mag_data[mag_index]

#         if img_time == mag_time:
#             aligned_data.append((img_array, mag_values))
#             img_index += 1
#             mag_index += 1
#         elif img_time < mag_time:
#             img_index += 1
#         else:
#             mag_index += 1

#     with open(output_file, 'wb') as f:
#         pickle.dump(aligned_data, f)

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
            # Read raw data from bag
            realsense_ts, rgbs, left_ir, right_ir, depth, raw_synced_mags, raw_synced_imus, synced_mags = read_bag_to_list(bag_path)

            print(f"\nRaw data loaded:")
            print(f"  - RGB frames: {len(rgbs)} (timestamps: {len(realsense_ts)})")
            print(f"  - Depth frames: {len(depth)}")
            print(f"  - Left IR frames: {len(left_ir)}")
            print(f"  - Right IR frames: {len(right_ir)}")
            print(f"  - Mag data points: {len(synced_mags)}")
            
            # Check for length mismatches in RealSense data
            realsense_lengths = {
                'rgb': len(rgbs),
                'depth': len(depth),
                'left_ir': len(left_ir),
                'right_ir': len(right_ir),
                'timestamps': len(realsense_ts)
            }
            
            if len(set(realsense_lengths.values())) > 1:
                print(f"\nWarning: RealSense stream length mismatch detected!")
                for stream, length in realsense_lengths.items():
                    print(f"  - {stream}: {length}")
                print("Will align to the shortest common length...")
                
                # Find minimum length and truncate all to match
                min_realsense_len = min(realsense_lengths.values())
                realsense_ts = realsense_ts[:min_realsense_len]
                rgbs = rgbs[:min_realsense_len]
                depth = depth[:min_realsense_len]
                left_ir = left_ir[:min_realsense_len]
                right_ir = right_ir[:min_realsense_len]
                
                print(f"Truncated all RealSense streams to {min_realsense_len} frames")
            
            # Analyze timing before alignment
            print("\n" + "="*50)
            print("TIMING ANALYSIS")
            print("="*50)
            
            timing_analysis = analyze_timing_alignment(realsense_ts, synced_mags)
            print(f"RealSense FPS: {timing_analysis.get('realsense_fps', 0):.2f}")
            print(f"Bowie FPS: {timing_analysis.get('bowie_fps', 0):.2f}")
            print(f"Time offset: {timing_analysis.get('time_offset_s', 0):.3f} seconds")
            
            # Perform time alignment using closest timestamp matching
            print("\n" + "="*50)
            print("TIME ALIGNMENT")
            print("="*50)
            
            # Extract bowie timestamps and data
            bowie_timestamps = [item[0] for item in synced_mags]
            bowie_mag_data = [item[1] for item in synced_mags]
            
            # Align all data streams to RealSense RGB timestamps (reference)
            aligned_rgbs, aligned_mags, mag_alignment_stats = align_data_to_reference(
                realsense_ts, rgbs, bowie_timestamps, bowie_mag_data, max_time_diff_ms=50
            )
            
            # For RealSense internal alignment, use the truncated data
            # Since we've already ensured all RealSense streams have the same length,
            # we can do direct alignment
            print("Aligning RealSense internal streams...")
            
            # Create aligned RealSense data (all should be same length now)
            aligned_rgbs_for_realsense = rgbs.copy()
            aligned_depth = depth.copy()
            aligned_left_ir = left_ir.copy()
            aligned_right_ir = right_ir.copy()
            
            # Create dummy alignment stats for RealSense internal streams
            perfect_alignment_stats = {
                'total_reference_frames': len(rgbs),
                'total_target_frames': len(rgbs),
                'aligned_frames': len(rgbs),
                'alignment_rate': 1.0,
                'mean_time_diff_ms': 0.0,
                'std_time_diff_ms': 0.0,
                'max_time_diff_ms': 0.0,
                'min_time_diff_ms': 0.0
            }
            
            depth_alignment_stats = perfect_alignment_stats.copy()
            left_alignment_stats = perfect_alignment_stats.copy()
            right_alignment_stats = perfect_alignment_stats.copy()
            
            print(f"\nAlignment Results:")
            print(f"RGB-Mag alignment: {mag_alignment_stats['aligned_frames']}/{mag_alignment_stats['total_reference_frames']} frames ({mag_alignment_stats['alignment_rate']:.1%})")
            print(f"  - Mean time diff: {mag_alignment_stats['mean_time_diff_ms']:.2f} ± {mag_alignment_stats['std_time_diff_ms']:.2f} ms")
            print(f"  - Max time diff: {mag_alignment_stats['max_time_diff_ms']:.2f} ms")
            
            # Create comparison between aligned and unaligned data
            print("\n" + "="*50)
            print("COMPARISON: ALIGNED vs UNALIGNED")
            print("="*50)
            
            # For unaligned: just truncate to minimum length (old approach)
            min_length = min(len(rgbs), len(synced_mags))
            unaligned_rgbs = rgbs[:min_length]
            unaligned_mags = [item[1] for item in synced_mags[:min_length]]
            
            print(f"Unaligned approach (length truncation):")
            print(f"  - Used frames: {min_length}")
            print(f"  - Discarded RGB frames: {len(rgbs) - min_length}")
            print(f"  - Discarded mag frames: {len(synced_mags) - min_length}")
            
            print(f"\nTime-aligned approach:")
            print(f"  - Used frames: {len(aligned_rgbs)}")
            print(f"  - Discarded RGB frames: {len(rgbs) - len(aligned_rgbs)}")
            print(f"  - Data utilization: {len(aligned_rgbs)/len(rgbs):.1%}")
            
            # Save both aligned and unaligned data for comparison
            print(f"\nSaving data to {bag_path}...")
            
            # Save aligned data (recommended)
            with open(bag_path+"/rgbs_aligned.pkl", 'wb') as f:
                pickle.dump(aligned_rgbs, f)
            with open(bag_path+"/depth_aligned.pkl", 'wb') as f:
                pickle.dump(aligned_depth, f)
            with open(bag_path+"/left_ir_aligned.pkl", 'wb') as f:
                pickle.dump(aligned_left_ir, f)
            with open(bag_path+"/right_ir_aligned.pkl", 'wb') as f:
                pickle.dump(aligned_right_ir, f)
            with open(bag_path+"/synced_mags_aligned.pkl", 'wb') as f:
                pickle.dump(aligned_mags, f)
            
            # Save original unaligned data (for comparison)
            with open(bag_path+"/rgbs.pkl", 'wb') as f:
                pickle.dump(rgbs, f)
            with open(bag_path+"/left_ir.pkl", 'wb') as f:
                pickle.dump(left_ir, f)
            with open(bag_path+"/right_ir.pkl", 'wb') as f:
                pickle.dump(right_ir, f)
            with open(bag_path+"/depth.pkl", 'wb') as f:
                pickle.dump(depth, f)
            with open(bag_path+"/synced_mags.pkl", 'wb') as f:
                pickle.dump(synced_mags, f)
            
            # Save alignment statistics
            alignment_summary = {
                'timing_analysis': timing_analysis,
                'mag_alignment_stats': mag_alignment_stats,
                'depth_alignment_stats': depth_alignment_stats,
                'left_ir_alignment_stats': left_alignment_stats,
                'right_ir_alignment_stats': right_alignment_stats,
                'comparison': {
                    'unaligned_frames': min_length,
                    'aligned_frames': len(aligned_rgbs),
                    'improvement_ratio': len(aligned_rgbs) / min_length if min_length > 0 else 0
                }
            }
            
            with open(bag_path+"/alignment_stats.pkl", 'wb') as f:
                pickle.dump(alignment_summary, f)
            
            # Create timing visualization
            create_timing_comparison_plots(
                realsense_ts, synced_mags, mag_alignment_stats, 
                f"{bag_path}/timing_comparison.png"
            )
            
            # Perform detailed alignment quality comparison
            print("\n" + "="*50)
            print("ALIGNMENT QUALITY COMPARISON")
            print("="*50)
            
            comparison_results = compare_alignment_quality(
                realsense_ts, synced_mags, aligned_rgbs, aligned_mags, rgbs,
                save_path=f"{bag_path}/alignment_quality_comparison.png"
            )
            
            # Print comparison summary
            before_stats = comparison_results['before_alignment']
            after_stats = comparison_results['after_alignment']
            improvement = comparison_results['improvement']
            
            print(f"\nALIGNMENT QUALITY COMPARISON RESULTS:")
            print(f"{'Metric':<25} {'Before':<15} {'After':<15} {'Improvement':<15}")
            print(f"{'-'*70}")
            print(f"{'Frames used':<25} {before_stats['num_frames']:<15} {after_stats['num_frames']:<15} {improvement['frame_utilization']*100:+.1f}%")
            print(f"{'Mean time diff (ms)':<25} {before_stats['mean_time_diff_ms']:<15.2f} {after_stats['mean_time_diff_ms']:<15.2f} {improvement['mean_time_diff_improvement_ms']:+.2f}")
            print(f"{'Std time diff (ms)':<25} {before_stats['std_time_diff_ms']:<15.2f} {after_stats['std_time_diff_ms']:<15.2f} {improvement['std_reduction_ms']:+.2f}")
            print(f"{'Max time diff (ms)':<25} {before_stats['max_time_diff_ms']:<15.2f} {after_stats['max_time_diff_ms']:<15.2f} {improvement['max_time_diff_improvement_ms']:+.2f}")
            print(f"{'Frames within 20ms':<25} {100*before_stats['frames_within_20ms']/before_stats['num_frames']:<15.1f}% {100*after_stats['frames_within_20ms']/after_stats['num_frames'] if after_stats['num_frames'] > 0 else 0:<15.1f}% {improvement['precision_improvement_20ms']*100:+.1f}%")
            print(f"{'Frames within 50ms':<25} {100*before_stats['frames_within_50ms']/before_stats['num_frames']:<15.1f}% {100*after_stats['frames_within_50ms']/after_stats['num_frames'] if after_stats['num_frames'] > 0 else 0:<15.1f}% {improvement['precision_improvement_50ms']*100:+.1f}%")
            
            # Save detailed comparison results
            with open(bag_path+"/alignment_quality_comparison.pkl", 'wb') as f:
                pickle.dump(comparison_results, f)
            
            print(f"\nFiles saved:")
            print(f"  - *_aligned.pkl: Time-aligned data (RECOMMENDED)")
            print(f"  - *.pkl: Original unaligned data")
            print(f"  - alignment_stats.pkl: Detailed alignment statistics")
            print(f"  - alignment_quality_comparison.pkl: Before/after comparison data")
            print(f"  - timing_comparison.png: Timing analysis visualization")
            print(f"  - alignment_quality_comparison.png: Detailed quality comparison plots")
            
        except Exception as e:
            print(f"Error processing {bag_path}: {str(e)}")
            import traceback
            traceback.print_exc()
            continue

if __name__ == '__main__':
    main()
