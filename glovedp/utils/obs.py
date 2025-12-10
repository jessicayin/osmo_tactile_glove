import torch
import numpy as np


def minmax_norm_data(data, dmin, dmax):
    # normalize to [-1, 1]
    ndata = (data - dmin) / ((dmax - dmin) + 1e-8)
    ndata = ndata * 2 - 1
    return ndata


def minmax_unnorm_data(ndata, dmin, dmax):
    ndata = (ndata + 1) / 2
    data = ndata * (dmax - dmin + 1e-8) + dmin
    return data


def _get_image_observation(data, im_c, im_height, im_width, clip_far, threshold):
    im_h, im_w = data[0]['base_rgb'].shape[1], data[0]['base_rgb'].shape[2]
    if im_c == 4:
        img = [
            np.concatenate(
                [d["base_rgb"], d["base_depth"][..., None]], axis=-1
            ).reshape(-1, im_h, im_w, im_c)
            for d in data
        ]
        img = np.stack(img)
        if clip_far:
            depth = img[..., -1]
            back_view = depth[0][None, ...]
            wrist = depth[1:]
            clip_back_view = back_view > (threshold / 10)
            clip_wrist = wrist > threshold
            clip = np.concatenate([clip_back_view, clip_wrist], axis=0)
            clip = np.concatenate([clip[..., None]] * im_c,
                                  axis=-1)
            img = img * clip
    else:
        img = [
            d["base_rgb"].reshape(-1, im_h, im_w, im_c) for d
            in data
        ]
        img = np.stack(img)

    img = torch.tensor(img.astype(np.float32))
    img = img.permute(0, 1, 4, 2, 3)
    return img


def get_observation(data_key, data, im_c, im_h, im_w, clip_far, threshold, load_img=False):
    input_data = {}
    for rt in data_key:
        if rt == "img":
            if load_img:
                input_data[rt] = _get_image_observation(data, im_c, im_h, im_w, clip_far, threshold)
            else:
                input_data[rt] = np.stack([d["file_path"] for d in data])
        elif rt == "pos":
            input_data[rt] = np.stack([d["joint_positions"] for d in data])
        elif rt == "hand_qpos":
            input_data[rt] = np.stack([d["hand_states"] for d in data])
        elif rt == "joint_vel":
            input_data[rt] = np.stack([d["joint_vel"][:12] for d in data])
        else:
            input_data[rt] = np.stack([d[rt] for d in data])
    return input_data
