# assume run
# python glovedp/dataset/convert_vis_glove_data.py
import os
import cv2
import tyro
import pickle
import numpy as np
import imageio

from glob import glob
from dataclasses import dataclass
# from matplotlib import pyplot as plt


@dataclass
class DatasetConfig:
    data_dir: str = 'data/0912_wipe_sea_raw/'
    rst_dir: str = 'data/0912_wipe_sea/'
    clip_format: bool = False


def plot_keypoints_on_image(image, keypoints, radius=5, color=(0, 255, 0)):
    # image: (H, W, 3) in RGB or BGR format
    # keypoints: (21, 2) numpy array
    img_copy = image.copy()
    for (x, y) in keypoints:
        x, y = int(x), int(y)
        cv2.circle(img_copy, (x, y), radius, color, -1)
    img_copy = cv2.cvtColor(img_copy, cv2.COLOR_BGR2RGB)
    return img_copy


def save(cache_name, obs_dict):
    os.makedirs(cache_name, exist_ok=True)
    fn = obs_dict['t_name']
    fn = os.path.join(cache_name, fn)
    with open(fn + ".pkl", "wb") as f:
        pickle.dump(obs_dict, f)
    cv2.imwrite(fn + '.png', obs_dict['rgb'])
    print(f"Saving {fn}")


def main(config):
    data_dir = config.data_dir
    rst_dir = config.rst_dir
    dataset_date = int(''.join([c for c in data_dir if c.isdigit()]))
    if dataset_date <= 913:
        dataset_type = "v0"
    else: dataset_type = "v1"
    files = sorted(glob(data_dir + '*/'))
    for f_i, file_name in enumerate(files):
        print(file_name)
        rgb = os.path.join(file_name, 'processed.pkl')
        with open(rgb, 'rb') as f:
            data = pickle.load(f)
        
        if dataset_type == "v0":
            start_idx = 38
            end_idx = -50
            images = data['rs_color'][start_idx:end_idx]
            with open(os.path.join(file_name, 'robot_cmds.pkl'), 'rb') as f:
                robot_states = pickle.load(f)
                robot_states = np.array(robot_states)

        else:
            images = data["rs_color"]
            robot_states = data["robot_cmds"]

        # keypoints_2d = data['pred_keypoints_2d']
        # keypoints_3d = data['pred_keypoints_3d']
        glove_touch = data['glove']

        all_frames = []
        for i in range(robot_states.shape[0] - 1):
            image = images[i]
            touch = np.array(glove_touch[i])
            next_touch = np.array(glove_touch[i + 1])
            # # Convert matplotlib bar chart to image
            # touch_img = plot_touch_vector(touch)
            # touch_img = cv2.resize(touch_img, (640, 200))  # Match width
            # rgb = plot_keypoints_on_image(image, k2d)
            # rgb = cv2.resize(rgb, (640, 360))  # Match width
            # rgb = np.vstack([rgb, touch_img])
            #
            # rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)  # already RGB from matplotlib
            # all_frames.append(rgb_bgr)

            # cv2.imshow('image', rgb)
            # cv2.waitKey(1)
            # time.sleep(0.02)
            states = robot_states[i].reshape(-1)
            # (width, height)
            actions = robot_states[i + 1].reshape(-1)

            rgb = cv2.resize(images[i], (640, 360))
            rgb = rgb[30:320, 100:460]

            delta_index = touch[0] - touch[1]
            delta_middle = touch[2] - touch[3]
            delta_ring = touch[4] - touch[5]
            delta_pinky = touch[6] - touch[7]
            delta_thumb = touch[8] - touch[9]
            delta_touch = np.concatenate([delta_index, delta_middle, delta_ring, delta_pinky, delta_thumb])

            next_delta_index = next_touch[0] - next_touch[1]
            next_delta_middle = next_touch[2] - next_touch[3]
            next_delta_ring = next_touch[4] - next_touch[5]
            next_delta_pinky = next_touch[6] - next_touch[7]
            next_delta_thumb = next_touch[8] - next_touch[9]
            next_delta_touch = np.concatenate([next_delta_index, next_delta_middle, next_delta_ring, next_delta_pinky, next_delta_thumb])
            
            obs_dict = {
                'states': states,
                'actions': actions,
                'rgb': rgb,
                'touch': touch.reshape(-1),
                'delta_touch': delta_touch,
                'next_touch': next_delta_touch,
                't_name': f'{i:04d}',
            }
            save(os.path.join(rst_dir, f'{f_i:04d}'), obs_dict)

        # gif_path = 'output.gif'
        # imageio.mimsave(gif_path, all_frames, fps=20)  # You can change fps
        # print(f"Saved gif to {gif_path}")


if __name__ == '__main__':
    main(tyro.cli(DatasetConfig))
