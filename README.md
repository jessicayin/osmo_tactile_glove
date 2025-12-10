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
