# Behavior Cloning for Glove Data

## Download Dataset

The following should be run from `glovedp/`

```
cd data/raw_data/
sftp hqi@em1.ist.berkeley.edu
get glove_data/some_name.zip
unzip ...
```

Process the data to be diffusion policy format (in `glovedp` folder):

`python dataset/convert_vis_glove_data.py --data_dir data/jessica_data/wipe1/ --rst_dir data/0811_wipe/`

After that, also manually create `train` and `test` folder, then you should have file structure:

```
data/
  0811_wipe/
    test/
      0004/
      0013/
    train/
      0000/
      0001/
      ...
      0015/
```

Train diffusion policy training

```
python train.py train --output_name im_state_touch_dp
```

The configs are in `glovedp/configs/base.py`.