"""


sensor IDs
index: 15, 35
middle: 11, 31
ring: 7, 27
pinky: 3, 23
thumb: 19, 39

ffmpeg commands
run from folder with images

mag video
ffmpeg -framerate 20 -i mag_%04d.png -c:v libx264 -pix_fmt yuv420p mag_video.mp4

rgb video
ffmpeg -framerate 20 -i frame_%04d.png -c:v libx264 -pix_fmt yuv420p frame_video.mp4

stack rgb + mag videos together
ffmpeg -i rgb_video.mp4 -i mag_video.mp4 -filter_complex "[0:v][1:v]hstack=inputs=2" -c:v libx264 -crf 23 -preset veryfast output.mp4

"""


import pickle
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm

class PlotSyncedData():
    def __init__(self, filename, save_path):
        self.filename = filename
        self.sensor_ids={ #0x1D - variant C - addresses
            "index": [15, 35],
            "middle": [11, 31],
            "ring": [7, 27],
            "pinky": [3, 23],
            "thumb": [19, 39]
        }
        self.fingers = ["index", "middle", "ring", "pinky", "thumb"]

        #for plotting
        self.axes_limits = { #x (min, max), y (min, max), z (min, max)
            "index":[[None, None], [None, None], [None, None]],
            "middle":[[None, None], [None, None], [None, None]],
            "ring":[[None, None], [None, None], [None, None]],
            "pinky":[[None, None], [None, None], [None, None]],
            "thumb":[[None, None], [None, None], [None, None]]
        }
        self.plot_min_buffer = self.plot_max_buffer = 50
        self.save_path = save_path
        self.load_data()
        self.process_bowie_mag()
        # self.plot_finger_mag()
        self.get_plot_axes()
        self.plot_synced_mags()
        self.plot_rgb()

    def load_data(self):
        data = pickle.load(open(self.filename, "rb"))
        bowie_data = np.asarray(data[1],dtype=object)
        self.bowie_data = bowie_data[:,1]
        realsense_data = np.asarray(data[0],dtype=object)
        self.realsense_data = realsense_data[:,1]

    def plot_finger_mag(self):
        """
        Plot synced delta magnetometer data only
        """
        fig, ax = plt.subplots(5, 3, figsize=(15, 15))
        
        for i in range(len(self.fingers)):
            finger=self.fingers[i]
            finger_data = np.asarray([entry[finger] for entry in self.indexed_delta_mag if finger in entry])

            ax[i, 0].plot(np.arange(finger_data.shape[0]),finger_data[:,0], label="x")
            ax[i, 1].plot(np.arange(finger_data.shape[0]),finger_data[:,1], label="y")
            ax[i, 2].plot(np.arange(finger_data.shape[0]),finger_data[:,2], label="z")
            ax[i, 0].set_title(f"{finger} - X")
            ax[i, 1].set_title(f"{finger} - Y")
            ax[i, 2].set_title(f"{finger} - Z")
        plt.title(f"Frame {i}")
        plt.tight_layout()
        plt.show()

    
    def plot_synced_mags(self):
        print("Plotting synced magnetometer data")
        finger_data = [self.index_delta_mags, self.middle_delta_mags, self.ring_delta_mags, self.pinky_delta_mags, self.thumb_delta_mags]
        for k in tqdm(range(self.bowie_data.shape[0])):
            fig, ax = plt.subplots(5, 3, figsize=(15, 15))
        
            for i in range(len(self.fingers)):
                finger=self.fingers[i]
                data = finger_data[i]

                ax[i, 0].plot(np.arange(data.shape[0])[:k],data[:k,0], label="x", color="red")
                ax[i, 1].plot(np.arange(data.shape[0])[:k],data[:k,1], label="y", color="green")
                ax[i, 2].plot(np.arange(data.shape[0])[:k],data[:k,2], label="z", color="blue")
                ax[i, 0].set_title(f"{finger} - X")
                ax[i, 1].set_title(f"{finger} - Y")
                ax[i, 2].set_title(f"{finger} - Z")
                ax[i, 0].set_ylim(self.axes_limits[finger][0][0], self.axes_limits[finger][0][1])
                ax[i, 1].set_ylim(self.axes_limits[finger][1][0], self.axes_limits[finger][1][1])
                ax[i, 2].set_ylim(self.axes_limits[finger][2][0], self.axes_limits[finger][2][1])
                ax[i, 0].set_xlim(0, data.shape[0])
                ax[i, 1].set_xlim(0, data.shape[0])
                ax[i, 2].set_xlim(0, data.shape[0])
            plt.title(f"Frame {i}")
            plt.tight_layout()
            plt.savefig(f"{self.save_path}/mag/mag_{k:04}.png")
            plt.close()


    def plot_rgb(self):
        """
        Plot realsense RGB data only
        """
        print("Plotting realsense")
        for i in tqdm(range(self.realsense_data.shape[0])):
            rgb_img = self.realsense_data[i][1]
            rgb_img = rgb_img[:, :, ::-1]
            plt.figure(figsize=(15, 15))
            plt.imshow(rgb_img)
            plt.title(f"Frame {i}")
            plt.axis('off')
            plt.tight_layout()
            plt.savefig(f"{self.save_path}/rgb/frame_{i:04}.png")
            plt.close()
        print("Saved all realsense images.")


    def delta_mag(self, mag_dict):
        """
        Calculate the difference between two magnetometer readings.
        Args:
            mag1: First magnetometer reading (x, y, z).
            mag2: Second magnetometer reading (x, y, z).
        Returns:
            A tuple containing the differences in x, y, and z.
        """
        for finger in mag_dict:
            if mag_dict[finger][0] is not None and mag_dict[finger][1] is not None:
                mag1 = mag_dict[finger][0]
                mag2 = mag_dict[finger][1]
                delta = (
                    mag2[0] - mag1[0],
                    mag2[1] - mag1[1],
                    mag2[2] - mag1[2]
                )
                mag_dict[finger] = delta
            else:
                print(f"WARNING: Not enough data for {finger} to calculate delta.")
        return mag_dict

    
    def sort_to_finger(self, data, finger, mag_dict):
        """
        Sorts the data into the appropriate finger category.
        Args:
            data: The magnetometer data to be sorted.
            finger: The finger category (e.g., "index", "middle").
            mag_dict: The dictionary to store the sorted data.
        """
        if mag_dict[finger][0] is None:
            mag_dict[finger][0] = [data["mag"]["x"], data["mag"]["y"], data["mag"]["z"]]
        elif mag_dict[finger][1] is None:
            mag_dict[finger][1] = [data["mag"]["x"], data["mag"]["y"], data["mag"]["z"]]
        else:
            print(f"Error: More than two magnetometer readings for {finger}.")
        return mag_dict
    
    def get_plot_axes(self):
        """
        Check the axes limits for the given finger.
        Args:
            finger: The finger category (e.g., "index", "middle").
        """
        finger_data = [self.index_delta_mags, self.middle_delta_mags, self.ring_delta_mags, self.pinky_delta_mags, self.thumb_delta_mags]
        for i in range(len(self.fingers)):
            finger = self.fingers[i]
            data = finger_data[i]
            for j in range(3):
                if self.axes_limits[finger][j] == [None, None]:
                    self.axes_limits[finger][j][0] = np.min(data[:,j]) - self.plot_min_buffer
                    self.axes_limits[finger][j][1] = np.max(data[:,j]) + self.plot_max_buffer
                else:
                    print(f"WARNING: Axes limits already set for {finger} axis {j}.")
        return self.axes_limits

    def process_bowie_mag(self):
        """
        Calculate delta magnetometer data, separate IMU data, preseve synced indices
        """
        self.indexed_delta_mag = []
        for frame in self.bowie_data:
            #frame [0] is mag data
            #frame [1] is imu data
            mag_dict = {
            "index": [None, None],
            "middle": [None, None],
            "ring": [None, None],
            "pinky": [None, None],
            "thumb": [None, None]
            }
            #process mag data
            for item in frame:
                # if item[0] is not None and item[1] is not None:
                import ipdb; ipdb.set_trace()
                mag_data = item[0]
                mag_dict = self.sort_to_finger(mag_data, mag_data["finger"], mag_dict)
            self.delta_mag(mag_dict)
            self.indexed_delta_mag.append(mag_dict)
        
        self.indexed_delta_mag = np.asarray(self.indexed_delta_mag)
        self.index_delta_mags = np.asarray([entry["index"] for entry in self.indexed_delta_mag if "index" in entry])
        self.middle_delta_mags = np.asarray([entry["middle"] for entry in self.indexed_delta_mag if "middle" in entry])
        self.ring_delta_mags = np.asarray([entry["ring"] for entry in self.indexed_delta_mag if "ring" in entry])
        self.pinky_delta_mags = np.asarray([entry["pinky"] for entry in self.indexed_delta_mag if "pinky" in entry])
        self.thumb_delta_mags = np.asarray([entry["thumb"] for entry in self.indexed_delta_mag if "thumb" in entry])

   
def main():
    filename="/home/gumdev/human2robot/hardware/wave_grasp1/wave_grasp1.pkl"
    save_path = "/home/gumdev/human2robot/hardware/wave_grasp1/"
    plotter = PlotSyncedData(filename, save_path)

if __name__ == "__main__":
    main()