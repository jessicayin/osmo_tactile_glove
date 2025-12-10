import concurrent.futures
import os
import pickle
import natsort
import numpy as np


def from_pickle(config, path, load_img=True):
    with open(path, "rb") as f:
        data = pickle.load(f)
    if "base_rgb" not in data and load_img:
        # note: base_rgb is only for HATO compatibility
        rgb_keys = config.data.im_key
        rgb = [data[k] for k in rgb_keys]
        data["base_rgb"] = np.stack(rgb, axis=0)
        for ik in config.data.im_key:
            del data[ik]
    return data


# Get the trajectory data from the given directory
def iterate(path, config, workers=32, load_img=True, num_cam=3):
    dir = os.listdir(path)
    dir = [d for d in dir if d.endswith(".pkl")]
    dir = natsort.natsorted(dir)
    dirname = os.path.basename(path)
    root_path = "./mask_cache"
    data = []

    for i, file in enumerate(dir):
        try:
            # Process each file sequentially
            d = from_pickle(config, os.path.join(path, file), load_img)
            # TODO: ask toru
            # Uncomment the following block if needed
            # if not d["activated"]["l"] and not d["activated"]["r"]:
            #     continue

            basedirfile = os.path.join(dirname, file)
            maskfile = os.path.join(root_path, basedirfile)

            # Check if mask file exists and load it
            if os.path.exists(maskfile):
                d["mask"] = from_pickle(maskfile)

            # Add paths to the data dictionary
            d["mask_path"] = maskfile
            d["file_path"] = os.path.join(path, file)

            # Append the processed data
            data.append(d)

        except Exception as e:
            # Print error message for failed files
            print(f"Failed to load {file}: {e}")
            pass

    return data


# Get all trajectory directories from the given path
def get_epi_dir(path, prefix=None):
    dir = natsort.natsorted(os.listdir(path))
    if prefix is not None:
        prefixs = prefix.split("-")

    new_dir = []

    for d in dir:
        if os.path.isdir(os.path.join(path, d)):
            matched = False
            if prefix is None:
                matched = True
            else:
                for prefix in prefixs:
                    if d.startswith(prefix):
                        matched = True
            if matched:
                new_dir.append(d)

    print("All Directories")
    print(new_dir)
    print("==========")
    dir = new_dir
    dir_list = [
        os.path.join(path, d) for d in dir if os.path.isdir(os.path.join(path, d))
    ]
    return dir_list