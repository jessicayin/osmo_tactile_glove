import os
import torch
import pickle
import warnings
import numpy as np
import torch.utils.data
from glob import glob
from typing import Tuple, Literal, Dict, Optional
from dataset import data_processing
from configs.base import GloveDPConfig
from utils.obs import get_observation
from utils.obs import minmax_norm_data, minmax_unnorm_data


RT_DIM = {
    "states": 13,
    "action": 13,
    "touch": 30,
    "delta_touch": 15,
}


def create_sample_indices(
    episode_ends: np.ndarray,
    sequence_length: int,
    pad_before: int = 0,
    pad_after: int = 0,
):
    indices = list()
    for i in range(len(episode_ends)):
        start_idx = 0
        if i > 0:
            start_idx = episode_ends[i - 1]
        end_idx = episode_ends[i]
        episode_length = end_idx - start_idx

        min_start = -pad_before
        max_start = episode_length - sequence_length + pad_after

        # range stops one idx before end
        for idx in range(min_start, max_start + 1):
            buffer_start_idx = max(idx, 0) + start_idx
            buffer_end_idx = min(idx + sequence_length, episode_length) + start_idx
            start_offset = buffer_start_idx - (idx + start_idx)
            end_offset = (idx + sequence_length + start_idx) - buffer_end_idx
            sample_start_idx = 0 + start_offset
            sample_end_idx = sequence_length - end_offset
            indices.append(
                [buffer_start_idx, buffer_end_idx, sample_start_idx, sample_end_idx]
            )
    indices = np.array(indices)
    return indices


def sample_sequence(
    train_data,
    sequence_length,
    buffer_start_idx,
    buffer_end_idx,
    sample_start_idx,
    sample_end_idx,
):
    result = dict()
    for key, input_arr in train_data.items():
        sample = input_arr[buffer_start_idx:buffer_end_idx]
        data = sample
        if (sample_start_idx > 0) or (sample_end_idx < sequence_length):
            data = np.zeros(
                shape=(sequence_length,) + input_arr.shape[1:], dtype=input_arr.dtype
            )
            if sample_start_idx > 0:
                data[:sample_start_idx] = sample[0]
            if sample_end_idx < sequence_length:
                data[sample_end_idx:] = sample[-1]
            data[sample_start_idx:sample_end_idx] = sample
        result[key] = data
    return result


# normalize data
def get_data_stats(data):
    data = data.reshape(-1, data.shape[-1])
    stats = {"min": np.min(data, axis=0), "max": np.max(data, axis=0)}
    if np.any(stats["max"] > 1e5) or np.any(stats["min"] < -1e5):
        raise ValueError("data out of range")
    return stats


def normalize_data(data, stats):
    # nomalize to [0,1]
    ndata = (data - stats["min"]) / (stats["max"] - stats["min"] + 1e-8)
    # normalize to [-1, 1]
    ndata = ndata * 2 - 1
    return ndata


def unnormalize_data(ndata, stats):
    ndata = (ndata + 1) / 2
    data = ndata * (stats["max"] - stats["min"] + 1e-8) + stats["min"]
    return data


class MemmapLoader:
    def __init__(self, path):
        with open(os.path.join(path, "metadata.pkl"), "rb") as f:
            meta_data = pickle.load(f)

        print("Meta Data:", meta_data)
        self.fps = {}

        self.length = None
        for key, (shape, dtype) in meta_data.items():
            self.fps[key] = np.memmap(
                os.path.join(path, key + ".dat"), dtype=dtype, shape=shape, mode="r"
            )
            if self.length is None:
                self.length = shape[0]
            else:
                assert self.length == shape[0]

    def __getitem__(self, index):
        rets = {}
        for key in self.fps.keys():
            # print(key)
            value = self.fps[key]
            value = value[index]
            value_cp = np.empty(dtype=value.dtype, shape=value.shape)
            # breakpoint()
            value_cp[:] = value
            rets[key] = value_cp
        # breakpoint()
        return rets

    def __length__(self):
        return self.length


# dataset
class Dataset(torch.utils.data.Dataset):
    def __init__(
        self,
        config: GloveDPConfig,
        data_path: str,
        data_key: Tuple[str, ...],
        stats: dict = None,
        transform=None,
        load_img: bool = False,
        binarize_touch: bool = False,
        split: Literal["train", "test"] = "train",
        percentiles: Optional[Dict[str, float]] = None
    ):
        self.data_key = data_key
        self.pre_horizon = config.dp.pre_horizon
        self.obs_horizon = config.dp.obs_horizon
        self.act_horizon = config.dp.act_horizon
        self.transform = transform

        self.load_img = load_img
        self.image_num = len(config.data.im_key)
        self.image_channel = config.data.im_channel
        self.image_height = config.data.im_height
        self.image_width = config.data.im_width
        # initialize training data buffer
        num_data_point = sum([
            len(glob(f"{d}/**/*.pkl", recursive=True)) for d in data_path
        ])
        train_data = {"data": {}, "meta": {}}
        for rt in self.data_key + ("action",):
            if rt == "img":
                train_data["data"][rt] = np.empty((
                    num_data_point, self.image_num,
                    self.image_channel, self.image_height, self.image_width
                ), dtype=np.uint16)                
            elif rt == "action":
                train_data["data"][rt] = np.zeros(
                    (num_data_point, config.data.action_dim), dtype=np.float32
                )
            else:
                train_data["data"][rt] = np.zeros(
                    (num_data_point, RT_DIM[rt]), dtype=np.float32
                )
        train_data["meta"] = {"episode_ends": []}

        data_index = 0

        for i, epi in enumerate(data_path):
            print("loading {}-th data from {}\r".format(i, epi), end="")
            load_img = "img" in config.data.data_key
            data = data_processing.iterate(epi, config, load_img=load_img)
            if len(data) == 0:
                continue

            data_length = len(data)

            # images - (N, num_cams, self.image_channel, 240, 320)
            obs = get_observation(
                self.data_key, data, im_c=self.image_channel,
                im_h=self.image_height, im_w=self.image_width,
                clip_far=False, threshold=10000, load_img=self.load_img
            )

            # obs space
            for rt in self.data_key:
                if rt == 'touch':
                    train_data["data"][rt][data_index:data_index+data_length] = obs[rt].reshape((-1, 30))
                else:
                    train_data["data"][rt][data_index:data_index+data_length] = obs[rt]

            # action space
            act = np.stack([d["actions"] for d in data])
            if config.data.pred_tactile:
                pred_tactile = np.stack([d["next_touch"] for d in data])
                act = np.concatenate([act, pred_tactile], axis=1)
            train_data["data"]["action"][data_index:data_index+data_length] = act

            if len(train_data["meta"]["episode_ends"]) == 0:
                train_data["meta"]["episode_ends"].append(data_length)
            else:
                train_data["meta"]["episode_ends"].append(
                    data_length + train_data["meta"]["episode_ends"][-1]
                )
            data_index += data_length

        print("data loaded")
        for k, v in train_data["data"].items():
            print(k, v.shape)

        data = train_data
        print("data type:", data_key)
        # float32, [0,1], (N, m, 96,96,3)
        
        # at this point data["data"]['action'] is raw joint positions
        train_data = {
            rt: data["data"][rt][:, :] for rt in data_key
        }
        train_data["action"] = data["data"]["action"][:]
        episode_ends = data["meta"]["episode_ends"][:]

        if split == "train":
            self.percentiles = {}
        else:
            self.percentiles = percentiles
        for data_type in data_key:
            if data_type == "img":
                continue  # image normalization is different
            d = train_data[data_type]

            # # Debugging: before normalization
            # from matplotlib import pyplot as plt
            # fig, axes = plt.subplots(5, 6, figsize=(18, 12), constrained_layout=True)
            # axes = axes.ravel()
            # for j in range(30):
            #     axes[j].hist(d[:, j], bins=40)
            #     axes[j].set_title(f"Dim {j}")
            # plt.show()

            if split == "train":
                p2 = np.percentile(d, 2, axis=0)
                p98 = np.percentile(d, 98, axis=0)
                self.percentiles[data_type] = {'lower': p2, 'upper': p98}
            else:
                p2 = self.percentiles[data_type]['lower']
                p98 = self.percentiles[data_type]['upper']
            print(data_type)
            print(p2, p98)
            mid = 0.5 * (p2 + p98)
            span = (p98 - p2)
            # Avoid divide-by-zero if a dim is (near) constant in the 2–98% range
            eps = 1e-12
            span_safe = np.where(span < eps, 1.0, span)
            y = 2.0 * (d - mid) / span_safe  # 2–98% -> [-1, 1]
            y = np.clip(y, -1.5, 1.5)  # cap to [-1.5, 1.5]
            train_data[data_type] = y

            # # Plot histogram grid (5x6 for 30 dims)
            # fig, axes = plt.subplots(5, 6, figsize=(18, 12), constrained_layout=True)
            # axes = axes.ravel()
            # for d in range(30):
            #     ax = axes[d]
            #     ax.hist(y[:, d], bins=40)
            #     ax.set_title(f"Dim {d}")
            #     ax.set_xlim([-1.5, 1.5])
            #     # Optional guides for mapped percentiles:
            #     ax.axvline(-1.0, linestyle="--", linewidth=1)
            #     ax.axvline(1.0, linestyle="--", linewidth=1)
            # plt.show()

        # compute start and end of each state-action sequence
        # also handles padding
        indices = create_sample_indices(
            episode_ends=episode_ends,
            sequence_length=self.pre_horizon,
            pad_before=self.obs_horizon - 1,
            pad_after=self.act_horizon - 1,
        )

        # compute statistics and normalized data to [-1,1]
        # if stats is None:
        #     stats = dict()
        #     for key, data in train_data.items():
        #         stats[key] = get_data_stats(data)
        if stats is None:
            stats = dict()
            stats["action"] = get_data_stats(train_data["action"])

        train_data["action"] = minmax_norm_data(
            train_data["action"], dmin=stats["action"]["min"], dmax=stats["action"]["max"]
        )
        self.indices = indices
        self.stats = stats
        self.train_data = train_data
        self.binarize_touch = binarize_touch

    def __len__(self):
        return len(self.indices)

    def __getitem__(self, idx):
        # get the start/end indices for this datapoint
        buffer_start_idx, buffer_end_idx, sample_start_idx, sample_end_idx = self.indices[idx]

        # get normalized data using these indices
        nsample = sample_sequence(
            train_data=self.train_data,
            sequence_length=self.pre_horizon,
            buffer_start_idx=buffer_start_idx,
            buffer_end_idx=buffer_end_idx,
            sample_start_idx=sample_start_idx,
            sample_end_idx=sample_end_idx,
        )
        for k in self.data_key:
            # discard unused observations
            nsample[k] = nsample[k][: self.obs_horizon]
            if k == "img":
                nsample["img"] = torch.tensor(
                    nsample["img"].astype(np.float32), dtype=torch.float32
                )
                nsample_shape = nsample["img"].shape
                # transform the img
                nsample["img"] = nsample["img"].reshape(
                    nsample_shape[0] * nsample_shape[1], *nsample_shape[2:]
                )
                nsample["img"] = self.transform(nsample["img"])
                nsample["img"] = nsample["img"].reshape(nsample_shape[:3] + (nsample['img'].shape[-2], nsample['img'].shape[-1]))
            else:
                nsample[k] = torch.tensor(nsample[k], dtype=torch.float32)
        nsample["action"] = torch.tensor(nsample["action"], dtype=torch.float32)
        return nsample
