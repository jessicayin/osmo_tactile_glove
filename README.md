# OSMO: Open-Source Tactile Glove for Human-to-Robot Skill Transfer
Jessica Yin, Haozhi Qi*, Youngsun Wi*, Sayantan Kundu, Mike Lambeta, William Yang, Changhao Wang, Tingfan Wu, Jitendra Malik, and Tess Hellebrekers

*equal contribution 

Project website: [jessicayin.github.io/osmo_tactile_glove](https://jessicayin.github.io/osmo_tactile_glove/)

ArXiv: [https://arxiv.org/abs/2512.08920](https://arxiv.org/abs/2512.08920)

# OSMO Hardware

Hardware guides and PCB files are on the `website` branch of this repo.

# OSMO Glove Data Pipeline

### Code has references to "bowie", which was the internal name of OSMO. Bowie and the OSMO glove may be treated as interchangeable when understanding the code.



1. Extract the hamer keypoints: (in osmo env)
 ```
 conda activate osmo
 python data_collect/glove/labs/glove2robot/postprocess/extract_hamer.py <rel_path_to_data> 
 ```
 - sample data collect for paper can be downloaded via `data/download_data.sh` (TODO: upload data and update script)
2. **Optional but recommended** Inspect the extracted keypoints: (in osmo env) 
```
conda activate osmo
python data_collect/glove/scripts/plot_keypoints_with_osmo.py
```
3. Retarget the hamer keypoints to Psyonic to construct the dataset: (in osmo_kinematics env). 
   
   Requires camera extrinsics represented as a (4,4) rigid transformation matrix to be saved as a .npy file and the path of the file to be provided in `labs/glove2robot/config/config_extract_hamer.yaml` under `camera_calibration` 
```
conda activate osmo_kinematics
python data_collect/glove/kinematics/construct_retarget_dataset.py
```

## Setting up environments

Osmo Env
```
conda env create -f conda/osmo.yml
```
Osmo Kinematics Env
```
conda env create -f conda/osmo_kinematics.yml
```


## Extract Hamer
- The most computation-intensive step of the pipeline, we automate processing of multiple experiments using `data_collect/glove/labs/glove2robot/config/run_hardcoded_batches.sh`

## Plot Keypoints Osmo
- Used for visually validating the extract hamer keypoints and generating the combined visualization (including the magnetometer readings)

## Construct Retargeted Dataset
- Retargets the extracted hamer keypoints to Psyonic + Franka kinematics to construct the training dataset


# Train and Deploy Instructions
[Policy Training and Deployment](glovedp/README.md)

# BibTeX
If you find this work helpful, please consider citing:

```
@article{yin2025osmo,
    title={OSMO: Open-Source Tactile Glove for Human-to-Robot Skill Transfer},
    author={Jessica Yin and Haozhi Qi and Youngsun Wi and Sayantan Kundu and Mike Lambeta and William Yang and Changhao Wang and Tingfan Wu and Jitendra Malik and Tess Hellebrekers},
    journal={arXiv:2512.08920},
    year={2025}
}
```

---

# OSMO 项目详细分析报告

> **分析人**: Robusr  
> **日期**: 2026年6月16日  
> **仓库状态**: `main` 分支, commit `bfc7328`  
> **分析范围**: 完整代码库走查，覆盖硬件固件、数据管线、策略训练与机器人部署四大模块

---

## 目录

1. [项目概述](#1-项目概述)
2. [系统架构总览](#2-系统架构总览)
3. [模块深度分析](#3-模块深度分析)
   - [3.1 硬件层 — 触觉手套](#31-硬件层--触觉手套)
   - [3.2 固件层 — STM32 嵌入式系统](#32-固件层--stm32-嵌入式系统)
   - [3.3 数据采集与通信层](#33-数据采集与通信层)
   - [3.4 数据处理管线](#34-数据处理管线)
   - [3.5 运动学重定向](#35-运动学重定向)
   - [3.6 策略学习 — Diffusion Policy](#36-策略学习--diffusion-policy)
   - [3.7 机器人部署](#37-机器人部署)
4. [技术栈全景图](#4-技术栈全景图)
5. [数据流分析](#5-数据流分析)
6. [环境与依赖](#6-环境与依赖)
7. [代码质量与架构评价](#7-代码质量与架构评价)
8. [潜在改进方向](#8-潜在改进方向)

---

## 1. 项目概述

OSMO (Open-Source Tactile Glove) 是一个**完整的开源触觉手套系统**，旨在实现从人类操作演示到机器人灵巧操作策略的技能迁移。该系统由 UC Berkeley 和 Meta FAIR 联合开发，覆盖了从硬件设计、嵌入式固件、多模态数据采集与处理、手部姿态估计、运动学重定向、到扩散策略行为克隆训练与部署的**端到端全栈管线**。

**核心创新点**:
- 低成本、开源的可穿戴触觉手套（基于 MLX90393 三轴磁力计）
- 将触觉感知（30 维磁力计读数）与视觉（立体红外/RGB）和机器人状态融合
- 使用 Diffusion Policy 进行多模态模仿学习，同步预测机器人动作与未来触觉信号
- 支持 Franka FR3 机械臂 + Psyonic Ability Hand 灵巧手的完整部署

**内部代号**: 代码中大量引用 "Bowie" 作为 OSMO 手套的内部项目名，两者可互换理解。

---

## 2. 系统架构总览

```
┌─────────────────────────────────────────────────────────────────┐
│                      OSMO 系统架构                              │
├───────────────┬─────────────────────┬───────────────────────────┤
│   硬件/固件    │     数据管线         │     策略学习与部署         │
├───────────────┼─────────────────────┼───────────────────────────┤
│               │                     │                           │
│  PCB 设计     │  ROS2 数据采集      │  Diffusion Policy 训练    │
│  (传感器板    │  (bowie_ros,        │  (Conditional U-Net 1D)  │
│   + MCU 板)   │   realsense 节点)   │                           │
│               │        ↓            │        ↓                  │
│  STM32 固件   │  HaMeR 手部姿态估计 │  行为克隆推理             │
│  (I²C 多路    │  + SAM 手部分割     │  (DDIM/DDPM 调度器)       │
│   复用器,      │  + FoundationStereo │                           │
│   MLX90393)   │        ↓            │        ↓                  │
│               │  运动学重定向       │  Franka + Psyonic         │
│               │  (MuJoCo 仿真,      │  开环回放部署             │
│               │   Psyonic + Franka) │                           │
└───────────────┴─────────────────────┴───────────────────────────┘
```

---

## 3. 模块深度分析

### 3.1 硬件层 — 触觉手套

**目录**: `pcb/`, `hardware/`

手套的触觉感知基于 **10 个 MLX90393 三轴磁力计**，分布于五指指尖（每指 2 个传感器 × 3 轴 = 30 维信号）。传感器数据与手指形变产生的磁场变化相关，从而提供接触力/形变的代理感知。

| 硬件组件 | 位置 | 说明 |
|----------|------|------|
| [pcb/sensor_boards/](pcb/sensor_boards/) | 指尖 | 每个指尖 2 块传感器 PCB |
| [pcb/mcu_board/](pcb/mcu_board/) | 手背 | STM32 主控板，含 I²C 多路复用器 |
| [hardware/ros2/](hardware/ros2/) | 主机端 | ROS2 节点，接收串口数据并发布 |

**传感器排列**（`data/README.md` 确认）:
```
手指     传感器           坐标轴
食指     index_mag0     X, Y, Z
食指     index_mag1     X, Y, Z
中指     middle_mag0    X, Y, Z
中指     middle_mag1    X, Y, Z
无名指   ring_mag0      X, Y, Z
无名指   ring_mag1      X, Y, Z
小指     pinky_mag0     X, Y, Z
小指     pinky_mag1     X, Y, Z
拇指     thumb_mag0     X, Y, Z
拇指     thumb_mag1     X, Y, Z
```

**视觉硬件**: Intel RealSense D455/D405 立体红外相机，分辨率为 1280×720，基线约 5 cm。左右红外图像用于 FoundationStereo 深度估计，RGB 图像用于 HaMeR 手部姿态估计。

### 3.2 固件层 — STM32 嵌入式系统

**目录**: `firmware/`

| 固件项目 | 功能 |
|----------|------|
| [BowieGlove/](firmware/BowieGlove/) | 主固件 — STM32CubeIDE 工程，实现完整手套功能 |
| [BowieGlove_rtos/](firmware/BowieGlove_rtos/) | RTOS 变体（实时操作系统版本） |
| [mlx90393_GloveUSB_binary_burst/](firmware/mlx90393_GloveUSB_binary_burst/) | MLX90393 传感器驱动，二进制突发模式读取 |
| [i2c_scanner_mux/](firmware/i2c_scanner_mux/) | I²C 多路复用器扫描器 |
| [Glove2.2_SD_and_USB/](firmware/Glove2.2_SD_and_USB/) | SD 卡 + USB 双输出变体 |

**关键技术细节**:
- **通信协议**: 使用 Protobuf 进行数据序列化（`bowie.pb.h`），通过 COBS (Consistent Overhead Byte Stuffing) 编码后经 USB 串口传输
- **传感器读数**: MLX90393 通过 I²C 多路复用器轮询读取，每只手套共 10 个传感器
- **IMU**: BHI360 惯性测量单元集成（`bhi360.h`）
- **硬件平台**: STM32 微控制器，使用 STM32CubeIDE 开发环境

### 3.3 数据采集与通信层

**目录**: `hardware/ros2/`

数据采集基于 **ROS2 (Robot Operating System 2)** 分布式节点架构：

| ROS2 包 | 节点 | 功能 |
|---------|------|------|
| `bowie_ros` | `bowie_node_synced.py` | 手套传感器数据流（同步采集） |
| `bowie_ros` | `bowie_node.py` | 基础手套数据采集 |
| `realsense` | `realsense_node.py` | RealSense 摄像头流（RGB + 立体红外） |

**数据同步**: 传感器和摄像头数据通过 ROS2 时间戳同步。`bowie_node_synced.py` 负责协调多数据流的时间对齐。同步后的数据保存为 `.pkl` 格式，包含对应的 `rgbs`, `left_ir`, `right_ir`, `mags` 四个数据流。

**依赖**: `pyserial`（串口通信）、`cobs`（COBS 编解码）、`betterproto`（Protobuf 解析）

### 3.4 数据处理管线

**目录**: `labs/glove2robot/`

这是整个系统中最复杂的模块，负责将原始传感器数据转化为可用于策略训练的结构化数据集。

#### 管线步骤

```
┌──────────────┐     ┌──────────────────┐     ┌─────────────────────┐
│  原始数据     │ ──→ │  Step 1:         │ ──→ │  Step 2:            │
│  (.pkl)      │     │  extract_hamer   │     │  plot_keypoints     │
│              │     │  (HaMeR 关键点)  │     │  (可视化验证)        │
└──────────────┘     └──────────────────┘     └─────────────────────┘
                                                       ↓
┌──────────────┐     ┌─────────────────────┐
│  训练数据集   │ ←── │  Step 3:            │
│  (Diffusion  │     │  construct_retarget │
│   Policy)    │     │  (运动学重定向)      │
└──────────────┘     └─────────────────────┘
```

#### Step 1: HaMeR 关键点提取（[extract_hamer.py](labs/glove2robot/postprocess/extract_hamer.py)）

这是计算量最大的步骤。核心流程：

1. **手部检测**: 使用 **SAM (Segment Anything Model)** 或 **ViT** 进行手部检测与分割
   - 裁剪策略: 检测手部边界框后，以 `crop_padding=100px` 进行扩展裁剪，加速后续处理
2. **HaMeR 姿态估计**: 在 RGB 图像上运行 HaMeR 模型，提取 21 个手部 3D 关键点和 MANO 手部网格参数
3. **立体深度处理**: 使用 **FoundationStereo** 对左右红外图像进行视差估计
4. **深度对齐**: 支持两种模式：
   - `simplified`（推荐）: 直接使用 RealSense 的深度查找
   - `complex`: 完整的 RGB → IR 坐标变换
5. **滤波**: 使用 **Savitzky-Golay 滤波器**（`window_length=50/25`, `polyorder=3`）对关键点轨迹进行平滑
6. **ICP 配准**: 使用 Open3D 的 ICP 将手部 mesh 与点云进行配准优化

**关键配置** ([config_extract_hamer.yaml](labs/glove2robot/config/config_extract_hamer.yaml)):
- 支持三种数据源模式: `async` / `sync` / `osmo`
- 批量处理: `run_hardcoded_batches.sh` 自动化多实验处理
- GPU 优化: 支持多 GPU、`batch_size_multiplier=2`、`max_workers=8`

#### Step 2: 可视化验证（[plot_keypoints_with_osmo.py](scripts/plot_keypoints_with_osmo.py)）

用于视觉验证提取的 HaMeR 关键点质量，并生成包含磁力计读数的综合可视化。这是一个可选但推荐的 QA 步骤。

#### 工具库（[labs/glove2robot/utils/](labs/glove2robot/utils/)）

| 文件 | 功能 |
|------|------|
| [bowie_data.py](labs/glove2robot/utils/bowie_data.py) | 手套数据结构定义与 IO |
| [bowie_node.py](labs/glove2robot/utils/bowie_node.py) | 手套 ROS2 节点通用实现 |
| [constants.py](labs/glove2robot/utils/constants.py) | 系统常量（传感器 ID、关节索引等） |
| [glove_utils.py](labs/glove2robot/utils/glove_utils.py) | 手套传感器处理工具 |
| [hamer_utils.py](labs/glove2robot/utils/hamer_utils.py) | HaMeR 模型加载与推理工具 |
| [realsense_utils.py](labs/glove2robot/utils/realsense_utils.py) | RealSense 相机配置与数据读取 |
| [file_utils.py](labs/glove2robot/utils/file_utils.py) | 文件 IO 与路径管理 |
| [enumerate_cams.py](labs/glove2robot/utils/enumerate_cams.py) | 相机枚举与检测 |
| [dash_plot_bowie.py](labs/glove2robot/utils/dash_plot_bowie.py) | Dash Web 交互式可视化 |
| [simple_plot_bowie.py](labs/glove2robot/utils/simple_plot_bowie.py) | Matplotlib 静态可视化 |
| [bowiepb/](labs/glove2robot/utils/bowiepb/) | Protobuf Python 绑定 |

### 3.5 运动学重定向

**目录**: `kinematics/`, `models/`

这一步将提取的手部关键点映射到机器人执行空间。

#### [construct_retarget_dataset.py](kinematics/construct_retarget_dataset.py)

核心流程：
1. 加载 HaMeR 提取的手部关键点（21 个 3D 点）
2. 使用 **MuJoCo** 物理引擎加载 Psyonic Ability Hand + Franka FR3 模型
3. 通过 **Mink** 库（基于 MuJoCo 的逆运动学求解器）进行运动学优化
4. 将人手关键点重定向到机器人关节空间

**Psyonic 手部关节映射**:
```
ctrl[0]  → fr3_joint1          (Franka 关节 1)
ctrl[1]  → fr3_joint2          (Franka 关节 2)
ctrl[2]  → fr3_joint3          (Franka 关节 3)
ctrl[3]  → fr3_joint4          (Franka 关节 4)
ctrl[4]  → fr3_joint5          (Franka 关节 5)
ctrl[5]  → fr3_joint6          (Franka 关节 6)
ctrl[6]  → fr3_joint7          (Franka 关节 7)
ctrl[7]  → index_mcp           (食指 MCP)
ctrl[8]  → middle_mcp          (中指 MCP)
ctrl[9]  → ring_mcp            (无名指 MCP)
ctrl[10] → pinky_mcp           (小指 MCP)
ctrl[11] → thumb_flexor        (拇指屈曲)
ctrl[12] → thumb_rotator       (拇指旋转)
```

**MuJoCo 模型文件** ([models/](models/)):
- `psyonic_right_franka.xml` — Psyonic 右手 + Franka 机械臂组合模型
- `psyonic_right_franka_scene.xml` — 完整仿真场景
- `psyonic_right_franka_keyframes.xml` — 关键帧参考姿态
- `franka_fr3/` — Franka FR3 机械臂模型文件

**坐标变换**: 需要相机标定矩阵（`camera_calibration`）作为 `(4,4)` 刚体变换 `.npy` 文件，将相机空间坐标映射到机器人基座坐标系。

#### [plot_extracted_cmds.py](kinematics/plot_extracted_cmds.py)
用于可视化重定向后的机器人指令序列。

### 3.6 策略学习 — Diffusion Policy

**目录**: `glovedp/`

这是训练模块的核心，实现了基于扩散模型的行为克隆策略。

#### 架构设计

```
多模态观测                 编码器              融合            扩散策略
┌──────────┐          ┌──────────────┐
│ states   │ ──────→  │ MLP Encoder  │ ──→ (64d)
│ (13维)   │          │ 256→256→64   │
└──────────┘          └──────────────┘      ┌─────────────────┐     ┌──────────┐
                          ↓                 │ Conditional     │     │ action   │
┌──────────┐          ┌──────────────┐      │ Unet 1D         │ ──→ │ (28维)   │
│ img      │ ──────→  │ ViT/CNN      │ ──→  │ (噪声预测网络)   │     │ 13 基础  │
│ (RGB)    │          │ Encoder      │ (32d)│                 │     │ +15 触觉 │
└──────────┘          └──────────────┘      │ + DDIM/DDPM     │     └──────────┘
                          ↓                 │   调度器         │
┌──────────┐          ┌──────────────┐      │ + EMA 平滑      │
│ touch    │ ──────→  │ MLP Encoder  │ ──→  └─────────────────┘
│ (30维)   │          │ 256→256→64   │ (64d)
└──────────┘          └──────────────┘
```

#### 关键文件分析

| 文件 | 行数 | 核心功能 |
|------|------|----------|
| [configs/base.py](glovedp/configs/base.py) | 74 | 配置数据类：`EncoderConfig`, `DPConfig`, `OptimConfig`, `DataConfig`, `GloveDPConfig` |
| [dp/policy.py](glovedp/dp/policy.py) | 436 | `DiffusionPolicy` 类 — 噪声预测、DDIM/DDPM 调度、训练/评估/推理循环 |
| [dp/agent.py](glovedp/dp/agent.py) | 218 | `Agent` 类 — 训练编排、模型持久化、预测接口 |
| [dataset/dataset.py](glovedp/dataset/dataset.py) | 348 | PyTorch Dataset — 多模态数据加载、百分位归一化、序列采样 |
| [dataset/data_processing.py](glovedp/dataset/data_processing.py) | - | Episode 目录遍历与数据聚合 |
| [utils/obs.py](glovedp/utils/obs.py) | - | 观测空间处理（裁剪远距离点、图像加载） |
| [utils/misc.py](glovedp/utils/misc.py) | - | 辅助工具（种子设置、Git hash、日志） |
| [utils/utils.py](glovedp/utils/utils.py) | - | 通用工具函数 |
| [train.py](glovedp/train.py) | 65 | 训练入口 — tyro CLI、多 GPU 支持、自动保存配置 |

#### 关键超参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `obs_horizon` | 1 | 观察历史帧数 |
| `act_horizon` | 4 | 预测动作帧数 |
| `pre_horizon` | 16 | 扩散预测总长度 |
| `diffusion_iters` | 100 | 扩散去噪步数（训练） |
| `diffusion_method` | `ddim` | 扩散方法（DDIM 比 DDPM 推理更快） |
| `batch_size` | 128 | 训练批次大小 |
| `num_epoch` | 500 | 训练轮数 |
| `learning_rate` | 0.0002 | 学习率 |
| `action_dim` | 13 或 28 | 动作维度（基础 13 + 可选触觉预测 15） |

#### 数据归一化策略

采用**百分位归一化**而非简单的 min-max：
```python
p2 = np.percentile(data, 2, axis=0)   # 第 2 百分位
p98 = np.percentile(data, 98, axis=0) # 第 98 百分位
mid = 0.5 * (p2 + p98)
span = (p98 - p2)
normalized = 2.0 * (data - mid) / span  # 映射到 [-1, 1]
```
这种策略对异常值更鲁棒，避免极端值压缩有效数据范围。

#### 视觉编码器选项

| 编码器 | 说明 | 图像尺寸 |
|--------|------|----------|
| `scratch` | 从头训练的 CNN | RandomCrop (261, 324) |
| `DINO` | 预训练 ViT（自监督） | CenterCrop (224, 224) |
| `DINOv3` | DINO v3（更强自监督） | CenterCrop (224, 224) |
| `CLIP` | 预训练 ViT（多模态） | CenterCrop (224, 224) |

对于冻结的预训练编码器，支持降低图像编码器学习率（`im_encoder_reduce_lr`）和完全冻结（`im_encoder_frozen`）。

#### 训练流程

1. **数据加载**: 从 `data/` 目录读取 episode（每个 episode 为 `.pkl` 文件的目录）
2. **统计计算**: 在训练集上计算百分位数和 min/max 统计
3. **多 GPU 支持**: 通过 `torchrun` 支持分布式训练（`--nproc_per_node=N`），使用 NCCL 后端同步梯度
4. **评估**: 每个 `eval_freq` epoch 在测试集上评估 MSE，保存最佳模型
5. **EMA**: 使用指数移动平均（power=0.75）平滑模型权重，提高推理稳定性

#### 推理流程（`DiffusionPolicy.forward()`）

```python
# 1. 编码多模态观测 → obs_cond
# 2. 初始化随机噪声动作
noisy_action = torch.randn(1, pred_horizon, action_dim)
# 3. DDIM/DDPM 迭代去噪 (默认 100 步)
for k in timesteps:
    noise_pred = model(obs_cond, noisy_action, k)
    noisy_action = scheduler.step(noise_pred, k, noisy_action)
# 4. 反归一化 + 取 action_horizon 个动作
```

### 3.7 机器人部署

**目录**: `hardware/deploy/`

[open_loop_replay.py](hardware/deploy/open_loop_replay.py) — 在真实机器人系统上执行训练好的 Diffusion Policy。采用**开环回放**模式，即一次推理输出整个动作序列并执行，不依赖闭环反馈。

---

## 4. 技术栈全景图

### 核心深度学习框架
| 技术 | 用途 |
|------|------|
| **PyTorch 2.4.1** | 深度学习主干 |
| **diffusers 0.35.2** | HuggingFace 扩散模型库（DDIM/DDPM 调度器） |
| **HaMeR** | 手部姿态估计（MANO 参数化手部模型） |
| **SAM / SAM-2** | 手部分割与检测 |
| **FoundationStereo** | 立体深度估计 |
| **DINO / DINOv3 / CLIP** | 预训练视觉编码器 |
| **flash-attn 2.7.4** | 高效注意力机制 |

### 机器人生态
| 技术 | 用途 |
|------|------|
| **ROS2 (Humble)** | 数据采集分布式通信 |
| **MuJoCo 3.4.0** | 物理仿真（运动学重定向） |
| **Mink** | MuJoCo 逆运动学求解器 |
| **Franka FR3** | 7-DoF 机械臂 |
| **Psyonic Ability Hand** | 灵巧手（6 个执行器） |
| **RealSense SDK** | D455/D405 相机驱动 |

### 数据处理与可视化
| 技术 | 用途 |
|------|------|
| **Hydra 1.3.2** | 配置管理 |
| **tyro** | CLI 参数解析 |
| **Open3D 0.19.0** | 点云处理与 ICP |
| **scipy** | 信号滤波（Savitzky-Golay） |
| **OpenCV** | 图像处理 |
| **Plotly / Dash** | 交互式可视化 |
| **TensorBoard** | 训练监控 |

### 固件工具链
| 技术 | 用途 |
|------|------|
| **STM32CubeIDE** | STM32 开发环境 |
| **Protobuf (nanopb)** | 嵌入式数据序列化 |
| **COBS** | 数据帧编码 |
| **I²C** | 传感器总线通信 |

---

## 5. 数据流分析

### 完整数据流

```
演示者佩戴手套操作
        │
        ▼
┌─────────────────────────────────────────┐
│  硬件层采集                              │
│  - 10× MLX90393 磁力计 (I²C → STM32)   │
│  - RealSense RGB + 立体 IR (USB → PC)   │
│  - BHI360 IMU                           │
└────────────┬────────────────────────────┘
             │ Protobuf + COBS (串口)
             │ ROS2 Topic (USB)
             ▼
┌─────────────────────────────────────────┐
│  ROS2 数据同步                           │
│  bowie_node_synced.py + realsense_node  │
│  输出: .pkl 文件 (时间对齐)              │
│    - rgbs.pkl (RGB 图像序列)            │
│    - left_ir.pkl / right_ir.pkl        │
│    - mags.pkl (30维磁力计)              │
└────────────┬────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────┐
│  extract_hamer.py                       │
│  - SAM/ViT 手部检测 + 裁剪              │
│  - HaMeR 姿态估计 → 21 个 3D 关键点     │
│  - FoundationStereo 深度 → 3D 点云      │
│  - ICP 配准优化                         │
│  - Savitzky-Golay 滤波                  │
│  输出: hamer_keypoints.pkl + 可视化     │
└────────────┬────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────┐
│  construct_retarget_dataset.py          │
│  - MuJoCo 加载 Psyonic + Franka 模型    │
│  - Mink IK 求解器 → 关节角度            │
│  - 相机→机器人基座坐标变换               │
│  输出: retarget_dataset/ (训练就绪)      │
└────────────┬────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────┐
│  Diffusion Policy 训练                   │
│  - Dataset: 多模态 (states+img+touch)   │
│  - 百分位归一化                         │
│  - Conditional U-Net 1D 噪声预测        │
│  - DDIM/DDPM 训练 + EMA 平滑            │
│  输出: model.ckpt + stats.pkl          │
└────────────┬────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────┐
│  open_loop_replay.py (部署)             │
│  - 加载 model.ckpt                      │
│  - 实时推理 → Franka + Psyonic 指令     │
│  - 开环执行动作序列                      │
└─────────────────────────────────────────┘
```

### 数据维度演化

```
原始传感器数据:
  mags:  (N, 30)         → 30 维触觉
  rgbs:  (N, 720, 1280)  → RGB 图像
  left_ir / right_ir: (N, 720, 1280) → 立体红外

HaMeR 提取后:
  hand_keypoints: (N, 21, 3)  → 21 个手部 3D 点
  wrist_pose: (N, 4, 4)       → 手腕刚体变换

运动学重定向后:
  robot_states: (N, 13)       → Franka(7) + Psyonic(6)
  actions: (N, 13)            → 关节目标位置

策略训练数据:
  states:  (obs_horizon, 13)  → 机器人状态
  img:     (obs_horizon, C, H, W) → RGB 图像
  touch:   (obs_horizon, 30)  → 触觉信号
  action:  (pred_horizon, 28) → 动作 + 触觉预测
```

---

## 6. 环境与依赖

### Conda 环境对比

| 特性 | `osmo` 环境 | `osmo_kinematics` 环境 |
|------|-------------|------------------------|
| Python | 3.10 | 3.9 |
| CUDA | 12.1 (完整工具包) | 无 CUDA |
| PyTorch | 2.4.1 + torchvision 0.19.1 | 无 |
| 核心用途 | 数据管线 + 策略训练 | 运动学重定向 |
| 关键包数 | ~460+ (含 pip) | ~180+ (含 pip) |

### 硬件需求

| 组件 | 最低要求 | 推荐配置 |
|------|----------|----------|
| GPU | NVIDIA GPU (CUDA 12.1+) | RTX 4090 / A100 |
| 相机 | RealSense D455/D405 | D455（更优深度） |
| 机械臂 | Franka FR3 | 需标定 |
| 灵巧手 | Psyonic Ability Hand | 右手版本 |

---

## 7. 代码质量与架构评价

### 优点

1. **端到端完整性**: 从硬件 PCB 到策略部署的全栈开源，在机器人操作领域极为罕见
2. **模块化设计**: 硬件/固件/数据处理/训练/部署分离清晰，各模块可独立使用
3. **配置驱动**: 使用 Hydra + dataclass 进行配置管理，避免硬编码
4. **多 GPU 支持**: 训练脚本原生支持分布式训练（torchrun + NCCL）
5. **数据归一化鲁棒**: 采用 2-98 百分位归一化替代简单 min-max，提高对异常值的鲁棒性
6. **丰富的可视化**: 提供 Dash 交互式、Matplotlib 静态、Plotly 等多种可视化工具
7. **详细文档**: 数据格式、传感器布局、配置参数均有详细文档说明
8. **隐私保护**: 发布的数据中将 IR 图像非手部区域做了白化处理

### 待改进点

1. **硬编码路径**: 大量绝对路径（如 `/home/gumdev/...`）存在于配置和脚本中，降低了可移植性
2. **代码复用**: `labs/glove2robot/utils/` 和 `hardware/ros2/.../utils/` 存在代码重复（如 `bowie.py`, `bowie_data.py`, `constants.py` 等文件完全一致）
3. **类型注解不完整**: 部分函数缺少类型注解，降低了可维护性
4. **测试覆盖**: 仅 ROS2 包有基础的 `test_copyright.py`/`test_flake8.py`/`test_pep257.py`，核心管线缺少单元测试
5. **错误处理**: 部分脚本缺少完善的异常处理，GPU 资源管理（如 `gc.collect()`, `torch.cuda.empty_cache()`）需要手动调用
6. **配置分散**: 存在多个 YAML 配置文件（Hydra 配置、模型配置、训练配置），部分参数需跨文件同步
7. **数据下载**: `download_data.sh` 标记为 TODO，示例数据托管方案待完善

---

## 8. 潜在改进方向

### 短期改进（代码质量）
1. **统一工具库**: 将重复的 `labs/glove2robot/utils/` 代码提取为共享包
2. **路径抽象**: 使用环境变量或 Hydra 变量替代硬编码路径
3. **CI/CD**: 添加 GitHub Actions 进行代码格式检查（ruff/flake8）和基本测试
4. **Docker 支持**: 提供 Dockerfile 简化环境配置

### 中期改进（功能增强）
1. **闭环控制**: 当前仅支持开环回放，可添加基于触觉反馈的闭环调整
2. **数据增强**: 在 Diffusion Policy 训练中引入数据增强（随机裁剪、颜色抖动）
3. **实时推理优化**: 将 DDIM 推理步数从 100 降至 ~15（推理时），已在 `Agent.predict()` 中预留了接口
4. **多任务学习**: 扩展数据集支持多种操作技能的统一训练
5. **在线微调**: 支持在机器人执行过程中在线收集数据并微调策略

### 长期改进（架构演进）
1. **Sim-to-Real**: 利用 MuJoCo 仿真生成合成数据辅助训练
2. **多传感器融合**: 集成 IMU 数据（BHI360 已焊接但未在管线中使用）
3. **跨机器人泛化**: 支持不同机械臂/灵巧手平台的快速适配
4. **触觉自监督学习**: 利用触觉预测作为自监督信号，减少对标注的依赖

---

## 附录：关键文件索引

### 入口脚本
| 脚本 | 环境 | 功能 |
|------|------|------|
| [labs/glove2robot/postprocess/extract_hamer.py](labs/glove2robot/postprocess/extract_hamer.py) | `osmo` | HaMeR 关键点提取 |
| [scripts/plot_keypoints_with_osmo.py](scripts/plot_keypoints_with_osmo.py) | `osmo` | 关键点可视化 |
| [kinematics/construct_retarget_dataset.py](kinematics/construct_retarget_dataset.py) | `osmo_kinematics` | 运动学重定向 |
| [glovedp/train.py](glovedp/train.py) | `osmo` | Diffusion Policy 训练 |
| [hardware/deploy/open_loop_replay.py](hardware/deploy/open_loop_replay.py) | `osmo` | 机器人部署 |

### 配置文件
| 文件 | 说明 |
|------|------|
| [labs/glove2robot/config/config_extract_hamer.yaml](labs/glove2robot/config/config_extract_hamer.yaml) | HaMeR 管线配置 |
| [glovedp/configs/base.py](glovedp/configs/base.py) | 训练超参数（Python dataclass） |
| [conda/osmo.yml](conda/osmo.yml) | 主环境依赖 |
| [conda/osmo_kinematics.yml](conda/osmo_kinematics.yml) | 运动学环境依赖 |

### 模型文件
| 文件 | 说明 |
|------|------|
| [models/psyonic_right_franka.xml](models/psyonic_right_franka.xml) | MuJoCo 机器人模型 |
| [models/psyonic_right_franka_scene.xml](models/psyonic_right_franka_scene.xml) | 仿真场景 |
| [models/psyonic_right_franka_keyframes.xml](models/psyonic_right_franka_keyframes.xml) | 关键帧参考 |

---

