## Image + state only

python tools/deploy/closeloop_with_realsense.py --ckpt-path outputs/0913_state_dinov2_image_small/model_best.ckpt
python tools/deploy/closeloop_with_realsense.py --ckpt-path outputs/0913_state_image_small/model_best.ckpt

## All modalities

python tools/deploy/closeloop_all.py --ckpt-path outputs/0913_state_dinov2_image_delta_touch_small/model_last.ckpt
python tools/deploy/closeloop_all.py --ckpt-path outputs/0913_state_image_delta_touch_small/model_last.ckpt

## Need to modify unet
python tools/deploy/closeloop_all.py --ckpt-path outputs/0913_state_image_delta_touch/model_best.ckpt
python tools/deploy/closeloop_all.py --ckpt-path outputs/0913_state_dinov2_image_delta_touch/model_best.ckpt

## Need to modify tactile