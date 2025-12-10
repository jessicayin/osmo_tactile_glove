import math
import torch
from torch import nn


class TemporalTransformer(nn.Module):
    def __init__(self, embedding_dim, n_head, depth, output_dim, dim_feedforward=2048, use_pe=False, pre_ffn=False, input_dim=0,
                 dense_output: bool = False, dropout=0.1):
        """
        dense_output: whether transformer output per-step output or single classification
        """
        super(TemporalTransformer, self).__init__()
        self.use_pe = use_pe
        self.pre_ffn = pre_ffn
        self.dense_output = dense_output
        if self.use_pe:
            self.position_encoder = PositionalEncoding(embedding_dim, 31)
        if self.pre_ffn:
            self.ffn = nn.Sequential(
                nn.Linear(input_dim, embedding_dim),
                nn.ReLU(inplace=True),
            )
        encoder_layer = nn.TransformerEncoderLayer(
            d_model=embedding_dim,
            dim_feedforward=dim_feedforward,
            nhead=n_head,
            dropout=dropout,
            batch_first=True
        )
        self.transformer_encoder = nn.TransformerEncoder(
            encoder_layer,
            num_layers=depth
        )
        self.low_dim_proj = nn.Linear(embedding_dim, output_dim)
        self.cls_token = torch.nn.Parameter(
            torch.randn(1, 1, embedding_dim)
        )  # "global information"
        torch.nn.init.normal_(self.cls_token, std=0.02)

    def forward(self, x, extra_embedding=None):
        # assume x of shape [B, T, F]
        if self.pre_ffn:
            x = self.ffn(x)
        # cls_token = torch.randn((x.shape[0], 1, x.shape[-1]), device=x.device)
        # x = torch.column_stack((cls_token, x))  # tokens is of shape [B, 1+T, F]
        x = torch.column_stack((self.cls_token.repeat(x.shape[0], 1, 1), x))  # tokens is of shape [B, 1+T, F]
        if self.use_pe:
            x = self.position_encoder(x)
        if extra_embedding is not None:
            x = torch.cat([x, extra_embedding[:, None]], dim=1)
        x = self.transformer_encoder(x)
        if self.dense_output:
            x = self.low_dim_proj(x)
        else:
            x = x[:, 0, :]
            x = self.low_dim_proj(x.flatten(1))
        return x


class PositionalEncoding(nn.Module):

    def __init__(self, d_model: int, max_len: int = 5000):
        super().__init__()
        position = torch.arange(max_len).unsqueeze(1)
        div_term = torch.exp(torch.arange(0, d_model, 2) * (-math.log(10000.0) / d_model))
        pe = torch.zeros(max_len, d_model)
        pe[:, 0::2] = torch.sin(position * div_term)
        pe[:, 1::2] = torch.cos(position * div_term)
        pe = pe[None]
        self.register_buffer('pe', pe)

    def forward(self, x):
        """
        Arguments:
            x: Tensor, shape ``[seq_len, batch_size, embedding_dim]``
        """
        x = x + self.pe[:, :x.size(1)]
        return x
