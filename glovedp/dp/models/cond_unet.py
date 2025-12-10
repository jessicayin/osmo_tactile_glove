import math
from typing import Union

import torch
from torch import nn
from dp.models.vision.resnet import ResnetEncoder


class MLP(nn.Module):
    def __init__(self, units, input_size):
        super(MLP, self).__init__()
        layers = []
        for output_size in units:
            layers.append(nn.Linear(input_size, output_size))
            layers.append(nn.ReLU())
            input_size = output_size
        self.mlp = nn.Sequential(*layers)

    def forward(self, x):
        return self.mlp(x)


class MLPGELU(nn.Module):
    def __init__(self, units, input_size):
        super(MLPGELU, self).__init__()
        layers = []
        for output_size in units:
            layers.append(nn.Linear(input_size, output_size))
            layers.append(nn.GELU())
            input_size = output_size
        self.mlp = nn.Sequential(*layers)

    def forward(self, x):
        return self.mlp(x)
    

class StateEncoder(nn.Module):
    def __init__(self, input_dim, net_dim, activation='ReLU'):
        super(StateEncoder, self).__init__()
        if activation == 'ReLU':
            self.mlp = MLP(net_dim, input_dim)
        else:
            self.mlp = MLPGELU(net_dim, input_dim)

    def forward(self, x):
        return self.mlp(x)


# Diffusion policy


class SinusoidalPosEmb(nn.Module):
    def __init__(self, dim):
        super().__init__()
        self.dim = dim

    def forward(self, x):
        device = x.device
        half_dim = self.dim // 2
        emb = math.log(10000) / (half_dim - 1)
        emb = torch.exp(torch.arange(half_dim, device=device) * -emb)
        emb = x[:, None] * emb[None, :]
        emb = torch.cat((emb.sin(), emb.cos()), dim=-1)
        return emb


class Downsample1d(nn.Module):
    def __init__(self, dim):
        super().__init__()
        self.conv = nn.Conv1d(dim, dim, 3, 2, 1)

    def forward(self, x):
        return self.conv(x)


class Upsample1d(nn.Module):
    def __init__(self, dim):
        super().__init__()
        self.conv = nn.ConvTranspose1d(dim, dim, 4, 2, 1)

    def forward(self, x):
        return self.conv(x)


class Conv1dBlock(nn.Module):
    """
    Conv1d --> GroupNorm --> Mish
    """

    def __init__(self, inp_channels, out_channels, kernel_size, n_groups=8):
        super().__init__()

        self.block = nn.Sequential(
            nn.Conv1d(
                inp_channels, out_channels, kernel_size, padding=kernel_size // 2
            ),
            nn.GroupNorm(n_groups, out_channels),
            nn.Mish(),
        )

    def forward(self, x):
        return self.block(x)


class ConditionalResidualBlock1D(nn.Module):
    def __init__(self, in_channels, out_channels, cond_dim, kernel_size=3, n_groups=8):
        super().__init__()

        self.blocks = nn.ModuleList(
            [
                Conv1dBlock(in_channels, out_channels, kernel_size, n_groups=n_groups),
                Conv1dBlock(out_channels, out_channels, kernel_size, n_groups=n_groups),
            ]
        )

        # FiLM modulation https://arxiv.org/abs/1709.07871
        # predicts per-channel scale and bias
        cond_channels = out_channels * 2
        self.out_channels = out_channels
        self.cond_encoder = nn.Sequential(
            nn.Mish(), nn.Linear(cond_dim, cond_channels), nn.Unflatten(-1, (-1, 1))
        )

        # make sure dimensions compatible
        self.residual_conv = (
            nn.Conv1d(in_channels, out_channels, 1)
            if in_channels != out_channels
            else nn.Identity()
        )

    def forward(self, x, cond):
        """
        x : [ batch_size x in_channels x horizon ]
        cond : [ batch_size x cond_dim]

        returns:
        out : [ batch_size x out_channels x horizon ]
        """
        out = self.blocks[0](x)
        embed = self.cond_encoder(cond)

        embed = embed.reshape(embed.shape[0], 2, self.out_channels, 1)
        scale = embed[:, 0, ...]
        bias = embed[:, 1, ...]
        out = scale * out + bias

        out = self.blocks[1](out)
        out = out + self.residual_conv(x)
        return out


class ConditionalUnet1D(nn.Module):
    def __init__(
        self,
        config,
        input_dim,
        diffusion_step_embed_dim=256,
        # down_dims=[256, 512, 1024],
        down_dims=[128, 256, 512],
        kernel_size=5,
        n_groups=8,
        device="cpu",
    ):
        """
        input_dim: Dim of actions.
        global_cond_dim: Dim of global conditioning applied with FiLM
          in addition to diffusion step embedding. This is usually obs_horizon * obs_dim
        diffusion_step_embed_dim: Size of positional encoding for diffusion iteration k
        down_dims: Channel size for each UNet level.
          The length of this array determines numebr of levels.
        kernel_size: Conv kernel size
        n_groups: Number of groups for GroupNorm
        """

        super().__init__()
        self.obs_horizon = config.dp.obs_horizon
        self.device = device
        self.data_key = config.data.data_key
        self.image_num = len(config.data.im_key)
        self.im_encoder = config.data.im_encoder
        possible_input_type = ["img", "states", "touch", "delta_touch"]
        self.policy_input_types = [
            rt for rt in possible_input_type if rt in self.data_key
        ]
        self.encoders = nn.ModuleDict({})
        encoder_config = config.dp.encoder
        obs_dim = 0
        if "img" in self.data_key:
            if config.data.im_encoder == 'scratch':
                image_encoder = nn.ModuleList([
                    ResnetEncoder(encoder_config.im_net_output_dim, config.data.im_channel)
                    for i in range(self.image_num)
                ])
            elif config.data.im_encoder == 'DINO':
                model = torch.hub.load('facebookresearch/dinov2', 'dinov2_vits14')
                proj = nn.ModuleList([
                    nn.Linear(384, encoder_config.im_net_output_dim) for i in range(self.image_num)
                ])
                image_encoder = nn.ModuleList([
                    model for i in range(self.image_num)
                ])
                self.encoders["img_encoder_proj"] = proj
            elif config.data.im_encoder == 'DINOv3':
                model = torch.hub.load(
                    '/home/gumdev/dinov3/', 'dinov3_vits16',
                    source='local', weights='/home/gumdev/dinov3/dinov3.ckpt')
                proj = nn.ModuleList([
                    nn.Linear(384, encoder_config.im_net_output_dim) for i in range(self.image_num)
                ])
                image_encoder = nn.ModuleList([
                    model for i in range(self.image_num)
                ])
                self.encoders["img_encoder_proj"] = proj
            elif config.data.im_encoder == 'CLIP':
                import clip
                model, _ = clip.load("ViT-B/32", device=device)
                model = model.float()
                proj = nn.ModuleList([
                    nn.Linear(512, encoder_config.im_net_output_dim) for i in range(self.image_num)
                ])
                image_encoder = nn.ModuleList([
                    model for i in range(self.image_num)
                ])
                self.encoders["img_encoder_proj"] = proj
            else:
                raise NotImplementedError
            image_dim = encoder_config.im_net_output_dim * self.image_num
            self.encoders["img_encoder"] = image_encoder
            obs_dim += image_dim
        if "states" in self.data_key:
            self.encoders["states_encoder"] = StateEncoder(
                encoder_config.states_input_dim,
                encoder_config.states_net_dim,
            )
            obs_dim += encoder_config.states_net_dim[-1]
        if "touch" in self.data_key:
            self.encoders["touch_encoder"] = StateEncoder(
                encoder_config.touch_input_dim,
                encoder_config.touch_net_dim,
            )
            obs_dim += encoder_config.touch_net_dim[-1]
        if "delta_touch" in self.data_key:
            self.encoders["delta_touch_encoder"] = StateEncoder(
                15,
                encoder_config.touch_net_dim,
                activation=encoder_config.touch_encoder_activation
            )
            obs_dim += encoder_config.touch_net_dim[-1]

        global_cond_dim = obs_dim * self.obs_horizon
        all_dims = [input_dim] + list(down_dims)
        start_dim = down_dims[0]

        dsed = diffusion_step_embed_dim
        diffusion_step_encoder = nn.Sequential(
            SinusoidalPosEmb(dsed),
            nn.Linear(dsed, dsed * 4),
            nn.Mish(),
            nn.Linear(dsed * 4, dsed),
        )
        cond_dim = dsed + global_cond_dim

        in_out = list(zip(all_dims[:-1], all_dims[1:]))
        mid_dim = all_dims[-1]
        self.mid_modules = nn.ModuleList(
            [
                ConditionalResidualBlock1D(
                    mid_dim,
                    mid_dim,
                    cond_dim=cond_dim,
                    kernel_size=kernel_size,
                    n_groups=n_groups,
                ),
                ConditionalResidualBlock1D(
                    mid_dim,
                    mid_dim,
                    cond_dim=cond_dim,
                    kernel_size=kernel_size,
                    n_groups=n_groups,
                ),
            ]
        )

        down_modules = nn.ModuleList([])
        for ind, (dim_in, dim_out) in enumerate(in_out):
            is_last = ind >= (len(in_out) - 1)
            down_modules.append(
                nn.ModuleList(
                    [
                        ConditionalResidualBlock1D(
                            dim_in,
                            dim_out,
                            cond_dim=cond_dim,
                            kernel_size=kernel_size,
                            n_groups=n_groups,
                        ),
                        ConditionalResidualBlock1D(
                            dim_out,
                            dim_out,
                            cond_dim=cond_dim,
                            kernel_size=kernel_size,
                            n_groups=n_groups,
                        ),
                        Downsample1d(dim_out) if not is_last else nn.Identity(),
                    ]
                )
            )

        up_modules = nn.ModuleList([])
        for ind, (dim_in, dim_out) in enumerate(reversed(in_out[1:])):
            is_last = ind >= (len(in_out) - 1)
            up_modules.append(
                nn.ModuleList(
                    [
                        ConditionalResidualBlock1D(
                            dim_out * 2,
                            dim_in,
                            cond_dim=cond_dim,
                            kernel_size=kernel_size,
                            n_groups=n_groups,
                        ),
                        ConditionalResidualBlock1D(
                            dim_in,
                            dim_in,
                            cond_dim=cond_dim,
                            kernel_size=kernel_size,
                            n_groups=n_groups,
                        ),
                        Upsample1d(dim_in) if not is_last else nn.Identity(),
                    ]
                )
            )

        final_conv = nn.Sequential(
            Conv1dBlock(start_dim, start_dim, kernel_size=kernel_size),
            nn.Conv1d(start_dim, input_dim, 1),
        )

        self.diffusion_step_encoder = diffusion_step_encoder
        self.up_modules = up_modules
        self.down_modules = down_modules
        self.final_conv = final_conv

        all_params = sum(p.numel() for p in self.parameters())
        print(f"Model Parameters: {all_params // 1e6:.1f}M")

    def forward_encoder(self, data):
        features = []
        for data_key in self.policy_input_types:
            nsample = data[data_key][:, :self.obs_horizon].to(self.device)
            if data_key == "img":
                # nsample: (batch, horizon, view, c, h, w)
                # images: list of num_views
                # - each element is (batch, horizon, c, h, w)
                images = [
                    nsample[:, :, i] for i in range(nsample.shape[2])
                ]
                # flatten view to batch
                # encoder input: (batch x horizon, c, h, w)
                if self.im_encoder == 'CLIP':
                    image_features = [
                        self.encoders[f"{data_key}_encoder"][i].encode_image(
                            image.flatten(end_dim=1)
                        )
                        for i, image in enumerate(images)
                    ]
                    image_features = [
                        self.encoders["img_encoder_proj"][i](im_feat.float()) for i, im_feat in enumerate(image_features)
                    ]
                elif self.im_encoder == 'DINO' or self.im_encoder == 'DINOv3':
                    image_features = [
                        self.encoders[f"{data_key}_encoder"][i](
                            image.flatten(end_dim=1)
                        )
                        for i, image in enumerate(images)
                    ]
                    image_features = [
                        self.encoders["img_encoder_proj"][i](im_feat.float()) for i, im_feat in enumerate(image_features)
                    ]
                else:
                    image_features = [
                        self.encoders[f"{data_key}_encoder"][i](
                            image.flatten(end_dim=1)
                        )
                        for i, image in enumerate(images)
                    ]
                # (batch, dim, num_image)
                image_features = torch.stack(image_features, dim=2)
                image_features = image_features.reshape(
                    *nsample.shape[:2], -1
                )
                features.append(image_features)
            else:
                nfeat = self.encoders[f"{data_key}_encoder"](
                    nsample.flatten(end_dim=1)
                )
                nfeat = nfeat.reshape(*nsample.shape[:2], -1)
                features.append(nfeat)
                # (B, obs_horizon, obs_dim)
        obs_features = torch.cat(features, dim=-1)
        # (B, obs_horizon * obs_dim)
        obs_cond = obs_features.flatten(start_dim=1)
        return obs_cond

    def forward_denoise(self, obs_cond, sample, timestep):
        # (B,T,C)
        sample = sample.moveaxis(-1, -2)
        # (B,C,T)

        # 1. time
        timesteps = timestep
        if not torch.is_tensor(timesteps):
            # TODO: this requires sync between CPU and GPU. So try to pass timesteps as tensors if you can
            timesteps = torch.tensor(
                [timesteps], dtype=torch.long, device=sample.device
            )
        elif torch.is_tensor(timesteps) and len(timesteps.shape) == 0:
            timesteps = timesteps[None].to(sample.device)
        # broadcast to batch dimension in a way that's compatible with ONNX/Core ML
        timesteps = timesteps.expand(sample.shape[0])

        global_feature = self.diffusion_step_encoder(timesteps)

        if obs_cond is not None:
            global_feature = torch.cat([global_feature, obs_cond], axis=-1)

        x = sample
        h = []
        for idx, (resnet, resnet2, downsample) in enumerate(self.down_modules):
            x = resnet(x, global_feature)
            x = resnet2(x, global_feature)
            h.append(x)
            x = downsample(x)

        for mid_module in self.mid_modules:
            x = mid_module(x, global_feature)

        for idx, (resnet, resnet2, upsample) in enumerate(self.up_modules):
            x = torch.cat((x, h.pop()), dim=1)
            x = resnet(x, global_feature)
            x = resnet2(x, global_feature)
            x = upsample(x)

        x = self.final_conv(x)

        # (B,C,T)
        x = x.moveaxis(-1, -2)

        # (B,T,C)
        return x

    def forward(
        self,
        data,
        sample: torch.Tensor,
        timestep: Union[torch.Tensor, float, int],
    ):
        """
        x: (B,T,input_dim)
        timestep: (B,) or int, diffusion step
        global_cond: (B,global_cond_dim)
        output: (B,T,input_dim)
        """
        obs_cond = self.forward_encoder(data)  # (B, 64)
        x = self.forward_denoise(obs_cond, sample, timestep)
        return x
