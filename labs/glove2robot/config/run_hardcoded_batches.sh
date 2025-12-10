#!/bin/bash

# Hardcoded script to run extract_hamer.py for specific batches of data folders
# Each batch is executed sequentially with a 5-second sleep in between

set -e  # Exit on any error

# Configuration
SCRIPT_PATH="data_collect/glove/labs/glove2robot/postprocess/extract_hamer.py"

# Hardcoded Python commands for each batch
COMMANDS=(
    # "python $SCRIPT_PATH 'data/wipe_sea3/rosbag2_2025_09_13-23_34_24/,data/wipe_sea3/rosbag2_2025_09_13-23_34_53/'"
    # "python $SCRIPT_PATH 'data/wipe_sea3/rosbag2_2025_09_13-23_54_42/,data/wipe_sea3/rosbag2_2025_09_13-23_55_29/'"
    # "python $SCRIPT_PATH 'data/wipe_sea3/rosbag2_2025_09_13-23_56_28/,data/wipe_sea3/rosbag2_2025_09_13-23_57_22/'"
    # "python $SCRIPT_PATH 'data/wipe_sea3/rosbag2_2025_09_13-23_58_27/,data/wipe_sea3/rosbag2_2025_09_13-23_59_44/'" 
    "python $SCRIPT_PATH 'data/wipe_sea3/rosbag2_2025_09_14-00_00_41/,data/wipe_sea3/rosbag2_2025_09_14-00_01_38/'" 
    "python $SCRIPT_PATH 'data/wipe_sea3/rosbag2_2025_09_14-00_02_33/,data/wipe_sea3/rosbag2_2025_09_14-00_44_55/'"
    "python $SCRIPT_PATH 'data/wipe_sea3/rosbag2_2025_09_14-00_45_48/,data/wipe_sea3/rosbag2_2025_09_14-00_46_41/'" 
    "python $SCRIPT_PATH 'data/wipe_sea3/rosbag2_2025_09_14-00_47_26/,data/wipe_sea3/rosbag2_2025_09_14-00_48_13/'"
    "python $SCRIPT_PATH 'data/wipe_sea3/rosbag2_2025_09_14-00_48_51/,data/wipe_sea3/rosbag2_2025_09_14-00_49_22/'"
    "python $SCRIPT_PATH 'data/wipe_sea3/rosbag2_2025_09_14-00_49_52/,data/wipe_sea3/rosbag2_2025_09_14-00_50_56/'"
    "python $SCRIPT_PATH 'data/wipe_sea3/rosbag2_2025_09_14-00_51_34/,data/wipe_sea3/rosbag2_2025_09_14-00_52_29/'"
    "python $SCRIPT_PATH 'data/wipe_sea3/rosbag2_2025_09_14-00_53_32/,data/wipe_sea3/rosbag2_2025_09_14-00_54_09/'"
    "python $SCRIPT_PATH 'data/wipe_sea3/rosbag2_2025_09_14-00_54_41/,data/wipe_sea3/rosbag2_2025_09_14-00_55_12/'"
    "python $SCRIPT_PATH 'data/wipe_sea3/rosbag2_2025_09_14-00_55_45/,data/wipe_sea3/rosbag2_2025_09_14-00_56_15/'"
    "python $SCRIPT_PATH 'data/wipe_sea3/rosbag2_2025_09_14-00_57_11/,data/wipe_sea3/rosbag2_2025_09_14-00_58_10/'"
    "python $SCRIPT_PATH 'data/wipe_sea3/rosbag2_2025_09_14-00_58_59/,data/wipe_sea3/rosbag2_2025_09_14-00_59_23/'"
    "python $SCRIPT_PATH 'data/wipe_sea3/rosbag2_2025_09_14-00_59_44/,data/wipe_sea3/rosbag2_2025_09_14-01_00_02/'"
    "python $SCRIPT_PATH 'data/wipe_sea3/rosbag2_2025_09_14-01_06_17/,data/wipe_sea3/rosbag2_2025_09_14-01_06_56/'"
    "python $SCRIPT_PATH 'data/wipe_sea3/rosbag2_2025_09_14-01_07_40/,data/wipe_sea3/rosbag2_2025_09_14-01_08_10/'"
    "python $SCRIPT_PATH 'data/wipe_sea3/rosbag2_2025_09_14-01_08_44/,data/wipe_sea3/rosbag2_2025_09_14-01_09_18/'"
    "python $SCRIPT_PATH 'data/wipe_sea3/rosbag2_2025_09_14-01_09_43/,data/wipe_sea3/rosbag2_2025_09_14-01_10_23/'" #here now
    "python $SCRIPT_PATH 'data/wipe_sea3/rosbag2_2025_09_14-01_21_48/,data/wipe_sea3/rosbag2_2025_09_14-01_28_29/'"
    "python $SCRIPT_PATH 'data/wipe_sea3/rosbag2_2025_09_14-01_29_19/,data/wipe_sea3/rosbag2_2025_09_13-22_34_49/'"
    "python $SCRIPT_PATH 'data/wipe_sea3/rosbag2_2025_09_13-23_27_45/,data/wipe_sea3/rosbag2_2025_09_13-23_31_59/'" 
)

# Execute each command sequentially
for cmd in "${COMMANDS[@]}"; do
    echo "Executing: $cmd"
    eval "$cmd"
    echo "Sleeping for 5 seconds..."
    sleep 5
    echo
done

echo "All batches completed successfully."
