import pickle
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import ConnectionPatch
import matplotlib.gridspec as gridspec
from tqdm import tqdm
import os
import hydra
from omegaconf import DictConfig

class PlotKeypointsWithBowie():
    def __init__(self, file_path, start_idx=25, end_idx=-1):
        self.file_path = file_path
        self.processed_path = file_path + "processed.pkl"
        self.bowie_aligned_path = file_path + "synced_mags_aligned.pkl"
        self.rgb_aligned_path = file_path + "rgbs_aligned.pkl"
        self.save_path = file_path + 'combined_visualization/'
        self.start_idx = start_idx
        self.end_idx = end_idx  
        
        # Finger order for Bowie data (matches the synced message format)
        self.fingers = ["index", "middle", "ring", "pinky", "thumb"]
        
        # Create output directory
        try:
            os.makedirs(self.save_path)
            print(f"Created directory: {self.save_path}")
        except FileExistsError:
            print(f"Directory {self.save_path} already exists. Cleaning up old files...")
            import glob
            old_files = glob.glob(os.path.join(self.save_path, "*.png"))
            for f in old_files:
                os.remove(f)
            print(f"Cleaned {len(old_files)} old PNG files")

        self.load_data()
        self.process_bowie_data()
        self.setup_plot_limits()
        self.create_combined_plots()
        self.run_ffmpeg()

    def load_data(self):
        """Load all required data from pickle files."""
        print("Loading processed hand data...")
        with open(self.processed_path, 'rb') as f:
            self.processed_data = pickle.load(f)
        
        # Load keypoints data 
        self.keypoints_2d = self.processed_data['pred_keypoints_2d']
        self.keypoints_3d = self.processed_data["pred_keypoints_3d"]
        
        # Try to load aligned data first, fall back to processed data
        try:
            print("Loading aligned RGB data...")
            with open(self.rgb_aligned_path, 'rb') as f:
                self.rgbs = pickle.load(f)
            print(f"Loaded {len(self.rgbs)} aligned RGB frames")
            self.rgbs = self.rgbs[self.start_idx:self.end_idx] 

        except FileNotFoundError:
            print("Aligned RGB data not found, using processed data...")
            self.rgbs = self.processed_data['rs_color'][self.start_idx:self.end_idx]  # Skip first 25 frames
            
        try:
            print("Loading aligned Bowie data...")
            with open(self.bowie_aligned_path, 'rb') as f:
                synced_mags_data = pickle.load(f)
            print(f"Loaded {len(synced_mags_data)} aligned Bowie frames")
            synced_mags_data = synced_mags_data[self.start_idx:self.end_idx]  
            synced_mags_data = np.asarray(synced_mags_data)
            synced_mags_data = synced_mags_data.reshape((synced_mags_data.shape[0],10,4))
            self.bowie_data = synced_mags_data[:,:,1:]
            print(f"Skipped first {self.start_idx} and last {self.end_idx} frames")
        except FileNotFoundError:
            print("Aligned Bowie data not found! Please run convert_bag_to_pkl.py first.")
            raise FileNotFoundError("Bowie aligned data is required for this visualization")
        
        print("processed_data keypoints_2d:", len(self.keypoints_2d))
        print("bowie_data:", len(self.bowie_data))
        print("rgbs:", len(self.rgbs))
        assert len(self.rgbs) == len(self.keypoints_2d) == len(self.bowie_data), "Data length mismatch after trimming"


        print(f"Data loaded successfully and lengths match: {len(self.rgbs)} frames")

    def process_bowie_data(self):
        """Process Bowie magnetometer data into finger-specific arrays."""
        print("Processing Bowie magnetometer data...")
        
        # Initialize arrays for each finger's mag data
        self.finger_data = {}
        
        for finger in self.fingers:
            self.finger_data[finger] = {
                'mag0': {'x': [], 'y': [], 'z': []},
                'mag1': {'x': [], 'y': [], 'z': []},
                'delta': {'x': [], 'y': [], 'z': []}  # mag1 - mag0
            }
        
        # Process each frame of Bowie data
        for frame_data in self.bowie_data:
            # Each frame_data is a flattened array of 30 values:
            # 5 fingers × (2 mags × 3 coords) = 30 values
            # Order: index_mag0_xyz, index_mag1_xyz, middle_mag0_xyz, middle_mag1_xyz, ...
            
            frame_array = np.array(frame_data).reshape(5, 6)  # 5 fingers, 6 values each (2 mags × 3 coords)
            
            for i, finger in enumerate(self.fingers):
                # Extract mag0 (first 3 values) and mag1 (last 3 values) for this finger
                mag0_xyz = frame_array[i, :3]
                mag1_xyz = frame_array[i, 3:]
                
                # Store individual magnetometer readings
                self.finger_data[finger]['mag0']['x'].append(mag0_xyz[0])
                self.finger_data[finger]['mag0']['y'].append(mag0_xyz[1])
                self.finger_data[finger]['mag0']['z'].append(mag0_xyz[2])
                
                self.finger_data[finger]['mag1']['x'].append(mag1_xyz[0])
                self.finger_data[finger]['mag1']['y'].append(mag1_xyz[1])
                self.finger_data[finger]['mag1']['z'].append(mag1_xyz[2])
                
                # Calculate delta (mag1 - mag0) for each axis
                self.finger_data[finger]['delta']['x'].append(mag1_xyz[0] - mag0_xyz[0])
                self.finger_data[finger]['delta']['y'].append(mag1_xyz[1] - mag0_xyz[1])
                self.finger_data[finger]['delta']['z'].append(mag1_xyz[2] - mag0_xyz[2])
        
        # Convert to numpy arrays for easier plotting
        for finger in self.fingers:
            for mag_type in ['mag0', 'mag1', 'delta']:
                for axis in ['x', 'y', 'z']:
                    self.finger_data[finger][mag_type][axis] = np.array(self.finger_data[finger][mag_type][axis])

    def setup_plot_limits(self):
        """Calculate appropriate plot limits for Bowie data visualization."""
        print("Setting up plot limits...")
        
        self.plot_limits = {}
        buffer = 50  # Add some padding to the limits
        
        for finger in self.fingers:
            self.plot_limits[finger] = {}
            
            # Calculate limits for delta values (most important for visualization)
            for axis in ['x', 'y', 'z']:
                delta_data = self.finger_data[finger]['delta'][axis]
                self.plot_limits[finger][axis] = {
                    'min': np.min(delta_data) - buffer,
                    'max': np.max(delta_data) + buffer
                }

    def create_combined_plots(self):
        """Create combined visualization with RGB+keypoints on left and Bowie plots on right."""
        print("Creating combined visualization frames...")
        
        for frame_idx in tqdm(range(len(self.rgbs))):
            # Create figure with custom layout
            fig = plt.figure(figsize=(20, 12))
            
            # Create grid: left half for RGB+keypoints, right half for Bowie plots
            gs = gridspec.GridSpec(5, 6, figure=fig, hspace=0.3, wspace=0.3)
            
            # Left side: RGB image with keypoints (spans full height, half width)
            ax_rgb = fig.add_subplot(gs[:, :3])
            
            # Display RGB image
            img = self.rgbs[frame_idx]
            img_rgb = img[..., ::-1]  # BGR to RGB conversion
            ax_rgb.imshow(img_rgb)
            
            # Overlay 2D keypoints
            if frame_idx < len(self.keypoints_2d):
                keypoints = self.keypoints_2d[frame_idx]
                ax_rgb.scatter(keypoints[:, 0], keypoints[:, 1], c='lime', s=15, alpha=0.8, edgecolors='darkgreen', linewidth=0.5)
                
                # Draw hand skeleton connections (optional - you can customize this)
                self.draw_hand_skeleton(ax_rgb, keypoints)
            
            ax_rgb.set_title(f'RGB + Hand Keypoints (Frame {frame_idx})', fontsize=14, fontweight='bold')
            ax_rgb.axis('off')
            
            # Right side: Bowie magnetometer plots (5 fingers × 3 axes)
            for finger_idx, finger in enumerate(self.fingers):
                for axis_idx, axis in enumerate(['x', 'y', 'z']):
                    ax_bowie = fig.add_subplot(gs[finger_idx, 3 + axis_idx])
                    
                    # Plot delta magnetometer data up to current frame
                    frames_so_far = np.arange(frame_idx + 1)
                    delta_data = self.finger_data[finger]['delta'][axis][:frame_idx + 1]
                    
                    # Plot the line
                    ax_bowie.plot(frames_so_far, delta_data, color=self.get_axis_color(axis), linewidth=2)
                    
                    # Highlight current point
                    if len(delta_data) > 0:
                        ax_bowie.scatter(frame_idx, delta_data[-1], color=self.get_axis_color(axis), s=50, zorder=5)
                    
                    # Set limits and labels
                    ax_bowie.set_xlim(0, len(self.rgbs))
                    ax_bowie.set_ylim(self.plot_limits[finger][axis]['min'], 
                                     self.plot_limits[finger][axis]['max'])
                    
                    # Add labels only for edge plots to avoid clutter
                    if finger_idx == 0:  # Top row
                        ax_bowie.set_title(f'{axis.upper()}', fontsize=10, fontweight='bold')
                    if axis_idx == 0:  # Left column
                        ax_bowie.set_ylabel(f'{finger.capitalize()}', fontsize=10, fontweight='bold')
                    if finger_idx == len(self.fingers) - 1:  # Bottom row
                        ax_bowie.set_xlabel('Frame', fontsize=8)
                    
                    # Style the plot
                    ax_bowie.grid(True, alpha=0.3)
                    ax_bowie.tick_params(labelsize=8)
            
            # Add overall title
            fig.suptitle('Synchronized Hand Pose and Magnetometer Data', fontsize=16, fontweight='bold', y=0.98)
            
            # Add legend for magnetometer data
            self.add_bowie_legend(fig)
            
            # Save frame
            plt.savefig(f'{self.save_path}/combined_{frame_idx:04d}.png', 
                       dpi=150, bbox_inches='tight', facecolor='white')
            plt.close()
    
    def draw_hand_skeleton(self, ax, keypoints):
        """Draw hand skeleton connections between keypoints."""
        # MANO hand model connections (simplified)
        connections = [
            # Thumb
            (0, 1), (1, 2), (2, 3), (3, 4),
            # Index
            (0, 5), (5, 6), (6, 7), (7, 8),
            # Middle  
            (0, 9), (9, 10), (10, 11), (11, 12),
            # Ring
            (0, 13), (13, 14), (14, 15), (15, 16),
            # Pinky
            (0, 17), (17, 18), (18, 19), (19, 20)
        ]
        
        for start_idx, end_idx in connections:
            if start_idx < len(keypoints) and end_idx < len(keypoints):
                start_point = keypoints[start_idx]
                end_point = keypoints[end_idx]
                ax.plot([start_point[0], end_point[0]], 
                       [start_point[1], end_point[1]], 
                       'lime', linewidth=1.5, alpha=0.7)

    def get_axis_color(self, axis):
        """Get color for each axis."""
        colors = {'x': 'red', 'y': 'green', 'z': 'blue'}
        return colors[axis]
    
    def add_bowie_legend(self, fig):
        """Add legend for Bowie magnetometer plots."""
        # Create a small legend in the top right
        legend_elements = [
            plt.Line2D([0], [0], color='red', lw=2, label='X-axis'),
            plt.Line2D([0], [0], color='green', lw=2, label='Y-axis'),
            plt.Line2D([0], [0], color='blue', lw=2, label='Z-axis')
        ]
        
        legend = fig.legend(handles=legend_elements, loc='upper right', 
                           bbox_to_anchor=(0.98, 0.95), fontsize=10)
        legend.set_title('Magnetometer Δ', prop={'weight': 'bold'})

    def run_ffmpeg(self):
        """Create video and GIF from the generated frames."""
        print("Creating video and GIF...")
        
        import os
        
        # Create MP4 video
        mp4_command = f"ffmpeg -y -framerate 25 -i {self.save_path}combined_%04d.png -c:v libx264 -pix_fmt yuv420p {self.file_path}output.mp4"
        os.system(mp4_command)
        
        # Create GIF
        gif_command = f"ffmpeg -y -i {self.file_path}output.mp4 -filter_complex '[0:v] fps=10,scale=1280:-1:flags=lanczos,split [a][b];[a] palettegen [p];[b][p] paletteuse' {self.file_path}output.gif"
        os.system(gif_command)
        
        # Clean up intermediate files
        cleanup_command = f"rm -rf {self.save_path}"
        os.system(cleanup_command)
        
        print(f"Visualization saved to:")
        print(f"  - Video: {self.file_path}bowie_kp_output.mp4")
        print(f"  - GIF: {self.file_path}bowie_kp_output.gif")


@hydra.main(version_base=None, config_path="../labs/glove2robot/config", config_name="config_extract_hamer")
def main(cfg: DictConfig):
    # Get data paths from configuration
    data_paths = cfg.paths.data_path
    start_idx = cfg.get('start_index', 25)
    end_idx = cfg.get('end_index', -1)
    
    # Handle both single path and list of paths
    if isinstance(data_paths, str):
        data_paths = [data_paths]
    
    print(f"Processing {len(data_paths)} data directories from config:")
    for path in data_paths:
        print(f"  - {path}")
    
    # Process each directory from the config
    for file_path in data_paths:
        # Remove trailing slash if present and add it back for consistency
        file_path = file_path.rstrip('/') + '/'
        
        # Check if required files exist
        processed_file = file_path + "processed.pkl"
        bowie_aligned_file = file_path + "synced_mags_aligned.pkl"
        
        if not os.path.exists(processed_file):
            print(f"Warning: {processed_file} does not exist, skipping {file_path}")
            continue
            
        if not os.path.exists(bowie_aligned_file):
            print(f"Warning: {bowie_aligned_file} does not exist, skipping {file_path}")
            print("Please run convert_bag_to_pkl.py first to generate aligned Bowie data")
            continue
            
        print(f"\nProcessing: {file_path}")
        try:
            visualizer = PlotKeypointsWithBowie(file_path, start_idx=start_idx, end_idx=end_idx)
            print(f"Successfully processed {file_path}")
        except Exception as e:
            print(f"Error processing {file_path}: {str(e)}")
            import traceback
            traceback.print_exc()
            continue


if __name__ == "__main__":
    main()
