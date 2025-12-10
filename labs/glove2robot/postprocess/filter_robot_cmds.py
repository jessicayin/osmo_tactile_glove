import os
import hydra
import pickle
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
import os
from scipy.signal import savgol_filter
from omegaconf import DictConfig
import copy



@hydra.main(version_base=None, config_path="../config", config_name="config_extract_hamer")
def main(cfg: DictConfig):
    """Main function with options to test different IK methods."""
    
    
    # Get data paths from configuration
    data_paths = cfg.paths.data_path
    camera_config = cfg.camera_calibration
    start_idx = cfg.start_index
    end_idx = cfg.end_index
    
    # Handle both single path and list of paths
    if isinstance(data_paths, str):
        data_paths = [data_paths]
    
    parent_dir = "/home/gumdev/human2robot/"
    
    print(f"Processing {len(data_paths)} data directories from config:")
    for path in data_paths:
        print(f"  - {path}")
    
    for trial in data_paths:
        # Remove trailing slash if present
        trial = trial.rstrip('/')
        
        # Check if processed.pkl exists
        processed_file_path = parent_dir + trial + "/processed.pkl"
        robot_cmds_path = parent_dir + trial + "/robot_cmds.pkl"
        safety_violations_path = parent_dir + trial + "/violated_safety.pkl"

        if not os.path.exists(processed_file_path):
            print(f"Warning: {processed_file_path} does not exist, skipping {trial}")
            return
        if not os.path.exists(robot_cmds_path):
            print(f"Warning: {robot_cmds_path} does not exist, skipping {trial}")
            return
        if not os.path.exists(safety_violations_path):
            print(f"Warning: {safety_violations_path} does not exist, skipping {trial}")
            return

        # Step 1: Load data
        with open(processed_file_path, 'rb') as f:
            processed_data = pickle.load(f)
        with open(robot_cmds_path, 'rb') as f:
            robot_data = pickle.load(f)
        with open(safety_violations_path, 'rb') as f:
            safety_data = pickle.load(f)
       

        rgbs = np.asarray(processed_data["rs_color"])[start_idx:end_idx,:,:,:]
        glove = np.asarray(processed_data["glove"])[start_idx:end_idx,:,:]
        robot_cmds = np.asarray(robot_data)
        assert rgbs.shape[0] == robot_cmds.shape[0] == glove.shape[0]
        
        safe_glove = copy.deepcopy(glove)
        safe_rgbs = copy.deepcopy(rgbs)
        # Step 2: Repeat previous vals in processed.pkl
        safety_data = np.asarray(safety_data)
        for i in range(safety_data.shape[0]):
            safety_index = safety_data[i]
            safe_glove[safety_index] = safe_glove[safety_index-1]
            safe_rgbs[safety_index] = safe_rgbs[safety_index-1]
        
        print(safety_data)

        # Step 3: Filter franka and psyonic joints
        window_length = 51
        polyorder = 3
        robot_cmds_filtered = np.zeros_like(robot_cmds)
        for i in range(robot_cmds.shape[1]):
            robot_cmds_filtered[:, i] = savgol_filter(robot_cmds[:, i], window_length=window_length, polyorder=polyorder, mode='nearest')

        # Plot each robot_cmd before and after filter
        plt.figure(figsize=(12, 6))
        #13 subplots
        for i in range(robot_cmds.shape[1]):
            plt.subplot(3, 5, i+1)
            plt.plot(robot_cmds[:, i], label="original", alpha=0.7)
            plt.plot(robot_cmds_filtered[:, i], label="filtered")
            plt.title(f"Joint {i+1}")
            if i == 0:
                plt.legend()
        plt.tight_layout()
        plt.suptitle("Robot Commands Before and After Savitzky-Golay Filtering", y=1.02)
        plt.savefig(parent_dir + trial + "/robot_cmds_filter_comparison.png")
        # plt.show()
        plt.close()


        # Step 4: Save everything in new processed_train.pkl
        new_processed = {
            "rs_color": safe_rgbs,
            "glove": safe_glove,
            "robot_cmds": robot_cmds_filtered
        }
        new_processed_path = parent_dir + trial + "/processed_train.pkl"
        with open(new_processed_path, 'wb') as f:
            pickle.dump(new_processed, f)
        print(f"Saved filtered data to {new_processed_path}")

            
        print(f"\nProcessing: {trial}")


if __name__ == "__main__":
    
    main()
