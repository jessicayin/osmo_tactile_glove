import os
import torch
import pickle
import collections
import torch.utils.data

from torch import nn
from torchvision import transforms
from dataset import data_processing
from dataset.dataset import Dataset
from dp.policy import DiffusionPolicy
from configs.base import GloveDPConfig


class Agent:
    def __init__(
        self,
        config: GloveDPConfig,
        clip_far=False,
        num_diffusion_iters=100,
        load_img=False,
        num_workers=8,
        binarize_touch=False,
        device='cpu',
    ):
        self.config = config
        self.device = device
        action_dim = config.data.action_dim
        # TODO: depth processing
        self.data_key = self.config.data.data_key
        # diffusion policy
        self.pre_horizon = config.dp.pre_horizon
        self.obs_horizon = config.dp.obs_horizon
        self.act_horizon = config.dp.act_horizon
        self.num_workers = 0 # num_workers
        self.binarize_touch = binarize_touch

        self.clip_far = clip_far
        self.load_img = load_img
        self.image_height = config.data.im_height
        self.image_width = config.data.im_width
        self.train_epi_dir, self.test_epi_dir = [], []

        if config.data.im_encoder == 'scratch':
            value_transform = transforms.Normalize(mean=[0.0, 0.0, 0.0], std=[255.0, 255.0, 255.0])
            train_crop_t = transforms.RandomCrop(
                (int(self.image_height * 0.9), int(self.image_width * 0.9)),
            )
            test_crop_t = transforms.CenterCrop(
                (int(self.image_height * 0.9), int(self.image_width * 0.9)),
            )
        elif config.data.im_encoder == 'DINO' or config.data.im_encoder == 'DINOv3':
            value_transform = transforms.Compose([
                transforms.Normalize(mean=(0.0, 0.0, 0.0), std=(255.0, 255.0, 255.0)),
                transforms.Normalize(mean=(0.485, 0.456, 0.406), std=(0.229, 0.224, 0.225))
            ])
            # from (270, 360) to (256, 320)
            train_crop_t = transforms.RandomCrop((224, 224))
            test_crop_t = transforms.CenterCrop((224, 224))
        elif config.data.im_encoder == 'CLIP':
            value_transform = transforms.Compose([
                transforms.Normalize(mean=(0.0, 0.0, 0.0), std=(255.0, 255.0, 255.0)),
                transforms.Normalize(mean=(0.4814, 0.4578, 0.4082), std=(0.2686, 0.2613, 0.2757))
            ])
            train_crop_t = transforms.RandomCrop((224, 224))
            test_crop_t = transforms.CenterCrop((224, 224))
        else:
            raise NotImplementedError

        self.transform = transforms.Compose([
            value_transform,
            train_crop_t,
        ])
        self.eval_transform = transforms.Compose([
            value_transform,
            test_crop_t,
        ])

        self.stats = None
        self.percentiles = None
        obs_dim = 0

        if config.policy_type == "dp":
            self.policy = DiffusionPolicy(
                config=config,
                obs_horizon=self.obs_horizon,
                obs_dim=obs_dim,
                pred_horizon=self.pre_horizon,
                action_horizon=self.act_horizon,
                action_dim=action_dim,
                num_diffusion_iters=num_diffusion_iters,
                weight_decay=config.optim.weight_decay,
                learning_rate=config.optim.learning_rate,
                binarize_touch=self.binarize_touch,
                device=self.device,
            )
        else:
            raise NotImplementedError

        self.policy.to(self.device)
        self.iter = 0
        self.obs_deque = None
        self.threshold = 8000

    def predict(
        self, obs_deque: collections.deque, num_diffusion_iters=15
    ):
        pred = self.policy.forward(
            self.stats, self.percentiles, obs_deque, num_diffusion_iters=num_diffusion_iters
        )
        return pred

    def train(
        self,
        batch_size=4,
        num_epoch=1,
        save_path=None,
        save_freq=10,
        eval_freq=10,
        wandb_logger=None,
    ):
        train_path = os.path.join(self.config.data_dir, self.config.train_data)
        self.train_epi_dir = data_processing.get_epi_dir(train_path)
        train_dataset = Dataset(
            config=self.config,
            data_path=self.train_epi_dir,
            data_key=self.data_key,
            stats=self.stats,
            load_img=self.load_img,
            transform=self.transform,
            binarize_touch=self.binarize_touch,
            split="train",
        )
        train_loader = torch.utils.data.DataLoader(
            train_dataset, batch_size=batch_size,
            num_workers=self.num_workers, shuffle=True,
            pin_memory=True, persistent_workers=False,
            drop_last=True,
        )
        self.policy.data_stat = train_dataset.stats

        test_paths = self.config.test_data.split("+")
        test_loaders = []
        for test_path in test_paths:
            test_path = os.path.join(self.config.data_dir, test_path)
            self.test_epi_dir = data_processing.get_epi_dir(test_path)
            test_dataset = Dataset(
                config=self.config,
                data_path=self.test_epi_dir,
                data_key=self.data_key,
                stats=self.policy.data_stat,
                load_img=self.load_img,
                transform=self.eval_transform,
                binarize_touch=self.binarize_touch,
                split="test",
                percentiles=train_dataset.percentiles,
            )
            test_loaders.append(torch.utils.data.DataLoader(
                test_dataset, batch_size=batch_size,
                num_workers=self.num_workers, shuffle=False,
                pin_memory=True, persistent_workers=False,
            ))

        self.policy.set_lr_scheduler(len(train_loader), len(train_loader) * num_epoch)
        if self.stats is None or self.percentiles is None:
            self.stats = train_dataset.stats
            self.percentiles = train_dataset.percentiles
            self.save_stats(save_path)

        self.policy.train(
            num_epoch,
            train_loader,
            test_loaders,
            save_path=save_path,
            eval_freq=eval_freq,
            save_freq=save_freq,
            wandb_logger=wandb_logger,
        )

    def load(self, path):
        model_path = os.path.join(path)
        dir_path = os.path.dirname(path)
        stat_path = os.path.join(dir_path, "stats.pkl")
        norm_path = os.path.join(dir_path, "norm.pkl")
        self.stats = pickle.load(open(stat_path, "rb"))
        self.percentiles = pickle.load(open(norm_path, "rb"))
        self.policy.data_stat = self.stats
        self.policy.load(model_path)
        print("model loaded")

    def save_stats(self, path):
        os.makedirs(path, exist_ok=True)
        stat_path = os.path.join(path, "stats.pkl")
        norm_path = os.path.join(path, "norm.pkl")
        if not os.path.exists(stat_path):
            with open(stat_path, "wb") as f:
                pickle.dump(self.stats, f)
        if not os.path.exists(norm_path):
            with open(norm_path, "wb") as f:
                pickle.dump(self.percentiles, f)
        print("stats saved")

    def save(self, path):
        os.makedirs(path, exist_ok=True)
        model_path = os.path.join(path, "model.ckpt")
        stat_path = os.path.join(path, "stats.pkl")
        norm_path = os.path.join(path, "norm.pkl")
        self.policy.save(model_path)

        # if stat not exist, create one
        if not os.path.exists(stat_path):
            with open(stat_path, "wb") as f:
                pickle.dump(self.stats, f)
        if not os.path.exists(norm_path):
            with open(norm_path, "wb") as f:
                pickle.dump(self.percentiles, f)
        print("model saved")
