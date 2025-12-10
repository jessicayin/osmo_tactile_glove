# basic plots of joints from psyonic_kinematics.py retargeting
import pickle
import matplotlib.pyplot as plt
import numpy as np

def causal_moving_average(data, window_size):
    """
    Apply a causal moving average filter to the data.
    Only uses data points before and including the current point.
    
    Args:
        data: 2D numpy array of shape (timesteps, joints)
        window_size: int, size of the moving average window
    
    Returns:
        filtered_data: 2D numpy array with same shape as input
    """
    if window_size <= 0:
        raise ValueError("Window size must be positive")
    
    timesteps, num_joints = data.shape
    filtered_data = np.zeros_like(data)
    
    for t in range(timesteps):
        # Define the window start (causal - only look backwards)
        start_idx = max(0, t - window_size + 1)
        end_idx = t + 1  # +1 because slicing is exclusive at the end
        
        # Calculate moving average for all joints at once
        window_data = data[start_idx:end_idx, :]
        filtered_data[t, :] = np.mean(window_data, axis=0)
    
    return filtered_data

def load_data(folder_path):
    robot_cmds = pickle.load(open(folder_path+"/robot_cmds_new.pkl", "rb"))
    robot_cmds = np.asarray(robot_cmds)
    psyonic = robot_cmds[:,7:]
    franka = robot_cmds[:,:7]
    return franka, psyonic


def plot_psyonic(folder_path, psyonic, psyonic_filtered=None):
    plt.figure(figsize=(12, 8))
    for i in range(psyonic.shape[1]):
        plt.plot(psyonic[:,i], label=f"joint{i}", alpha=0.7)
        if psyonic_filtered is not None:
            plt.plot(psyonic_filtered[:,i], label=f"joint{i}_filtered", linewidth=2, linestyle='--')
    plt.legend()
    title = "Psyonic Joints" + (" (with filter)" if psyonic_filtered is not None else "")
    plt.title(title)
    plt.xlabel("Time Step")
    plt.ylabel("Joint Value")
    plt.tight_layout()
    filename = f"{folder_path}/psyonic_joints_filtered.png"
    plt.savefig(filename)
    plt.close()
    print(f"Saved Psyonic plot to {filename}")

def plot_franka(folder_path, franka, franka_filtered=None):
    plt.figure(figsize=(12, 8))
    for i in range(franka.shape[1]):
        plt.plot(franka[:,i], label=f"joint{i}", alpha=0.7)
        if franka_filtered is not None:
            plt.plot(franka_filtered[:,i], label=f"joint{i}_filtered", linewidth=2, linestyle='--')
    plt.legend()
    title = "Franka Joints" + (" (with filter)" if franka_filtered is not None else "")
    plt.title(title)
    plt.xlabel("Time Step")
    plt.ylabel("Joint Value")
    plt.tight_layout()
    filename = f"{folder_path}/franka_joints_filtered.png"
    plt.savefig(filename)
    plt.close()
    print(f"Saved Franka plot to {filename}")

def save_filtered_data(folder_path, franka_filtered, psyonic_filtered):
    """
    Save the filtered robot commands to a pickle file.
    
    Args:
        folder_path: str, path to the folder
        franka_filtered: numpy array of filtered Franka data
        psyonic_filtered: numpy array of filtered Psyonic data
    """
    # Combine filtered data in the same format as original
    robot_cmds_filtered = np.concatenate([franka_filtered, psyonic_filtered], axis=1)
    
    # Save to pickle file
    output_path = folder_path + "/robot_cmds_filtered.pkl"
    with open(output_path, "wb") as f:
        pickle.dump(robot_cmds_filtered.tolist(), f)  # Convert to list to match original format
    print(f"Saved filtered robot commands to {output_path}")

def moving_average(data, window_size=5):
    """ Calculate moving average with only past data points"""
    for i in range(data.shape[1]):
        data[:, i] = np.convolve(data[:, i], np.ones(window_size)/window_size, mode='valid')
    cumsum = np.cumsum(data, dtype=float)
    cumsum[window_size:] = cumsum[window_size:] - cumsum[:-window_size]
    return cumsum[window_size - 1:] / window_size

def main():
    FRANKA_WINDOW_SIZE = 5  # Adjust as needed
    PSYONIC_WINDOW_SIZE = 5  # Adjust as needed
    APPLY_FILTER = True      # Set to False to skip filtering
    
    data_dirs = []
    for i in range(8, 9):
        data_dirs.append(f"/home/gumdev/human2robot/data/wipe1/grid_all{i}")

    for folder_path in data_dirs:
        print(f"Processing folder: {folder_path}")
        try:
            franka, psyonic = load_data(folder_path)
        except FileNotFoundError:
            print(f"Data not found in {folder_path}, skipping...")
            continue

        if APPLY_FILTER:
            # Apply causal moving average filter
            print(f"Applying filter - Franka window: {FRANKA_WINDOW_SIZE}, Psyonic window: {PSYONIC_WINDOW_SIZE}")
            franka_filtered = causal_moving_average(franka, FRANKA_WINDOW_SIZE)
            psyonic_filtered = causal_moving_average(psyonic, PSYONIC_WINDOW_SIZE)
            
            # Save filtered data
            save_filtered_data(folder_path, franka_filtered, psyonic_filtered)

            # Plot with filtered data
            plot_psyonic(folder_path, psyonic, psyonic_filtered)
            plot_franka(folder_path, franka, franka_filtered)
        else:
            # Plot without filtering
            plot_psyonic(folder_path, psyonic)
            plot_franka(folder_path, franka)


if __name__ == "__main__":
    main()
