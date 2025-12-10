from dataclasses import dataclass
from typing import Literal, Tuple


@dataclass(frozen=True)
class EncoderConfig:
    # each modality defines own encoder and output
    states_input_dim: int = 13
    states_net_dim: Tuple[int, ...] = (256, 256, 64)
    # image encoder and input
    im_net_output_dim: int = 32
    im_encoder_frozen: bool = False
    im_encoder_reduce_lr: bool = False
    # touch encoder dim
    touch_input_dim: int = 30
    touch_net_dim: Tuple[int, ...] = (256, 256, 64)
    touch_encoder_activation: Literal["ReLU", "GELU"] = "GELU"


@dataclass(frozen=True)
class DPConfig:
    encoder: EncoderConfig = EncoderConfig()
    obs_horizon: int = 1
    act_horizon: int = 4
    pre_horizon: int = 16
    diffusion_iters: int = 100
    diffusion_method: Literal["ddim", "ddpm"] = "ddim"
    action_decoder: Literal["mlp", "hourglass", "cond_hourglass"] = "hourglass"
    last_dropout: float = 0.0
    cond_dropout: float = 0.0


@dataclass(frozen=True)
class OptimConfig:
    batch_size: int = 128
    num_epoch: int = 500
    weight_decay: float = 0.0
    learning_rate: float = 0.0002


@dataclass(frozen=True)
class DataConfig:
    data_key: Tuple[str, ...] = ("states", "img", "touch")
    im_encoder: Literal["scratch", "DINO", "CLIP", "DINOv3"] = "scratch"
    im_key: Tuple[str, ...] = ("rgb",)
    im_channel: int = 3
    im_height: int = 290
    im_width: int = 360
    # output space
    base_action_dim: int = 13
    pred_tactile: bool = True
    @property
    def action_dim(self) -> int:
        dim = self.base_action_dim
        if self.pred_tactile:
            dim += 15
        return dim


@dataclass(frozen=True)
class GloveDPConfig:
    seed: int = 0
    gpu: str = '0'
    data_dir: str = 'data/'
    multi_gpu: bool = False
    output_dir: str = 'outputs/'
    train_data: str = '1024_wipe_sea/train/'
    test_data: str = '1024_wipe_sea/test/'
    output_name: str = 'debug'
    policy_type: Literal["dp", "bc"] = "dp"
    dp: DPConfig = DPConfig()
    optim: OptimConfig = OptimConfig()
    data: DataConfig = DataConfig()
