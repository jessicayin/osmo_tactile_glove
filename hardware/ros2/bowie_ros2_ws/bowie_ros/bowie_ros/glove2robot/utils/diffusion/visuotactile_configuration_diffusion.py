# Extension of original DiffusionConfig class from lerobot repo
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from gum.third_party.lerobot.common.policies.diffusion.configuration_diffusion import (
    DiffusionConfig,
)


@dataclass
class VisuoTactileDiffusionConfig(DiffusionConfig):
    """
    Extended diffusion configuration.

    It adds observation.image back in from the gum/third-party modifications
    It also adds in an option for observation.tactile


    """

    # Inputs / output structure.
    n_obs_steps: int = 2
    horizon: int = 8
    n_action_steps: int = 8

    # override input/oputput shapes
    input_shapes: Dict[str, List[int]] = field(
        default_factory=lambda: {
            "observation.image": [3, 640, 480],
            "observation.state": [3],
            "observation.tactile": [5],
        }
    )

    output_shapes: Dict[str, List[int]] = field(
        default_factory=lambda: {
            "action": [23],
        }
    )

    # Vision backbone.
    vision_backbone: str = "resnet18"
    crop_shape: tuple[int, int] | None = (84, 84)
    crop_is_random: bool = True
    pretrained_backbone_weights: str | None = None
    use_group_norm: bool = True
    spatial_softmax_num_keypoints: int = 32
    use_separate_rgb_encoder_per_camera: bool = False

    num_steps: int = 1000  # Number of diffusion steps.
    learning_rate: float = 1e-4  # Learning rate for the optimizer.
    batch_size: int = 32  # Batch size for training.
    image_size: int = 256  # Size of the input images.
    num_layers: int = 6  # Number of layers in the model.

    def __post_init__(self):
        """Input validation (not exhaustive)."""
        # super().__post_init__()  # for now, completely over-riding the initial checks with my own while in dev
        if not self.vision_backbone.startswith("resnet"):
            raise ValueError(
                f"`vision_backbone` must be one of the ResNet variants. Got {self.vision_backbone}."
            )

        image_keys = {k for k in self.input_shapes if k.startswith("observation.image")}

        if (
            len(image_keys) == 0
            and "observation.environment_state" not in self.input_shapes
        ):
            raise ValueError(
                "You must provide at least one image or the environment state among the inputs."
            )

        if len(image_keys) > 0:
            if self.crop_shape is not None:
                for image_key in image_keys:
                    if (
                        self.crop_shape[0] > self.input_shapes[image_key][1]
                        or self.crop_shape[1] > self.input_shapes[image_key][2]
                    ):
                        raise ValueError(
                            f"`crop_shape` should fit within `input_shapes[{image_key}]`. Got {self.crop_shape} "
                            f"for `crop_shape` and {self.input_shapes[image_key]} for "
                            "`input_shapes[{image_key}]`."
                        )
            # Check that all input images have the same shape.
            first_image_key = next(iter(image_keys))
            for image_key in image_keys:
                if self.input_shapes[image_key] != self.input_shapes[first_image_key]:
                    raise ValueError(
                        f"`input_shapes[{image_key}]` does not match `input_shapes[{first_image_key}]`, but we "
                        "expect all image shapes to match."
                    )
        supported_prediction_types = ["epsilon", "sample"]
        if self.prediction_type not in supported_prediction_types:
            raise ValueError(
                f"`prediction_type` must be one of {supported_prediction_types}. Got {self.prediction_type}."
            )
        supported_noise_schedulers = ["DDPM", "DDIM"]
        if self.noise_scheduler_type not in supported_noise_schedulers:
            raise ValueError(
                f"`noise_scheduler_type` must be one of {supported_noise_schedulers}. "
                f"Got {self.noise_scheduler_type}."
            )

        # Check that the horizon size and U-Net downsampling is compatible.
        # U-Net downsamples by 2 with each stage.
        downsampling_factor = 2 ** len(self.down_dims)
        if self.horizon % downsampling_factor != 0:
            raise ValueError(
                "The horizon should be an integer multiple of the downsampling factor (which is determined "
                f"by `len(down_dims)`). Got {self.horizon=} and {self.down_dims=}"
            )
