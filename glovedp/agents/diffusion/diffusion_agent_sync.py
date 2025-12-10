import os
import json
import collections
import torch
import numpy as np
from typing import Any, Dict
from dataclasses import fields
from utils.obs import get_observation

from configs.base import GloveDPConfig
from dp.agent import Agent as DPAgent
import time


def merge_dataclass_and_dict(dataclass_obj: Any, update_dict: dict) -> Any:
    """
    Recursively merge a dataclass and a dictionary, returning a new dataclass instance.
    """
    if not hasattr(dataclass_obj, "__dataclass_fields__"):
        raise ValueError("Provided object is not a dataclass instance.")

    dataclass_fields = fields(dataclass_obj)
    dataclass_values = {field.name: getattr(dataclass_obj, field.name) for field in dataclass_fields}
    merged_values = {}

    for key, value in dataclass_values.items():
        if key in update_dict:
            if hasattr(value, "__dataclass_fields__"):  # If the field is a nested dataclass
                # Recursively merge the nested dataclass
                merged_values[key] = merge_dataclass_and_dict(value, update_dict[key])
            else:
                # Use the new value from the dictionary
                merged_values[key] = update_dict[key]
        else:
            # Keep the existing value if not in update_dict
            merged_values[key] = value

    # Create a new dataclass instance with merged values
    return dataclass_obj.__class__(**merged_values)


class DiffusionAgent:
    def __init__(self, ckpt_path):
        dp_args = os.path.join(os.path.dirname(ckpt_path), "config.json")
        with open(dp_args, "r") as f:
            dp_args = json.load(f)
        dp_args['multi_gpu'] = False
        dp_args['dp']['act_horizon'] = 16
        dp_args['dp']['diffusion_iters'] = 100

        config = merge_dataclass_and_dict(GloveDPConfig(), dp_args)
        torch.cuda.set_device(0)
        self.config = config
        self.data_key = config.data.data_key
        self.dp = DPAgent(
            config,
            clip_far=False,
            num_diffusion_iters=config.dp.diffusion_iters,
            load_img='img' in self.data_key,
            num_workers=8,
            device='cuda',  # Explicitly set GPU device
        )
        self.dp.load(ckpt_path)
        self.pre_horizon = config.dp.pre_horizon
        self.obs_horizon = config.dp.obs_horizon
        self.act_horizon = config.dp.act_horizon
        self.image_height = config.data.im_height
        self.image_width = config.data.im_width
        self.obsque = collections.deque(maxlen=self.obs_horizon)
        self.action_queue = collections.deque(maxlen=self.act_horizon)
        self.num_diffusion_iters = config.dp.diffusion_iters

    def act(self, obs: Dict[str, Any]) -> np.ndarray:
        load_img = 'img' in self.data_key
        obs = get_observation(
            self.data_key, [obs], im_c=3, im_h=self.image_height, im_w=self.image_width,
            clip_far=False, threshold=10000, load_img=load_img,
        )
        if "img" in obs:
            obs["img"] = self.dp.eval_transform(obs["img"].squeeze(0))

        # if obsque is empty, fill it with the current observation
        if len(self.obsque) == 0:
            self.obsque.extend([obs] * self.obs_horizon)
        else:
            self.obsque.append(obs)
        if len(self.action_queue) > 0:
            # if action queue is not empty, return the first action in the queue
            act = self.action_queue.popleft()
        else:
            # if action queue is empty, predict new actions
            start_time = time.time()
            pred = self.dp.predict(
                self.obsque, num_diffusion_iters=self.num_diffusion_iters
            )
            end_time = time.time()
            print(f'Inference Time: {end_time - start_time:.4f} sec')
            print("Action horizon:", self.act_horizon)
            print("Diffusion iters:", self.num_diffusion_iters)
            for i in range(self.act_horizon):
                act = pred[i]
                self.action_queue.append(act)
            act = self.action_queue.popleft()
        return act
