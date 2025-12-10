# Collect Glove Demonstrations

1. 4 terminal tabs:
    - Bowie ROS node
    - Realsense ROS node
    - ROS topic inspection
    - Rosbag collection
2. Activate `_metahand` conda env in all tabs
3. Source the ROS2 workspaces

    ```bash
    #Bowie tab
    cd ~/human2robot/hardware/ros2/bowie_ros2_ws
    source ./install/setup.bash

    #Realsense tab
    cd ~/human2robot/hardware/ros2/realsense_ros2_ws
    source ./install/setup.bash

    #Inspection tab
    cd ~/human2robot/hardware/ros2/bowie_ros2_ws
    source ./install/setup.bash

    #Collection tab
    cd ~/human2robot/hardware/ros2/bowie_ros2_ws
    source ./install/setup.bash
    ```
4. Run the ROS2 nodes

    ```bash
    #Bowie node
    ros2 run bowie_ros bowie_node

    #Realsense node
    ros2 run realsense realsense_node

    #Inspection
    ros2 topic list
    ros2 topic hz /bowie/synced
    ros2 topic hz /realsense/rgb
    ```

5. Collect data in ROS2 bag

    ```bash
    cd ~/human2robot/data/{folder}

    ros2 bag record -a -o {filename}
    ```

# Postprocess Glove Demonstrations for Training
1. Update folder paths in `config_extract_hamer.yaml` for processing pipeline
    
    ```bash
    paths:
    	data_path: ['home/gumdev/human2robot/data/wipe_fre/wipe0/',]
    ```
    
2. Extract data from ROS2 bags
    
    ```bash
    cd ~/human2robot/data_collect/glove/scripts/bowie
    conda activate _metahand #needed for ros2 bag read
    python convert_bag_to_pkl.py
    
    ```
    
3. [Per Lab Setup] Update camera configs in `config_extract_hamer.yaml` 
    
    ```bash
    #on computer with realsense, we need color intrinsics (1280 x 720) and Color to Infrared 1 extrinsics
    rs-enumerate-devices -c
    
    #convert rotation matrix to quaternion with online quaternion calculator
    #use translation vector directly
    
    #update in config file
    camera:
    	color_intrinsic:
    	- [fx, 0, ppx]
    	- [0, fy, ppy]
    	- [0, 0, 1]
    	extrinsics: 
    	- translation x
    	- translation y
    	- translation z
    	- qx
    	- qy
    	- qz
    	- qw
    ```
    
4. Run pipeline

    ```bash
    conda activate bowie_
    cd human2robot
    python data_collect/glove/labs/glove2robot/postprocess/extract_hamer.py
    ```
    NOTE: Lambda tends to run out of memory for CUDA after processing about 3 trials. Supervise and restart as needed.

5. Visualize hamer keypoints and tactile glove data with a gif for inspection

    ```bash
    conda activate bowie_
    #legacy, keypoints on rgb only:
    #python data_collect/glove/scripts/overlay_2d_keypoints.py
    python data_collect/glove/scripts/plot_keypoints_with_bowie.py
    ```

6. Retarget to Psyonic + Franka joints with Mujoco

    ``` bash
    conda activate kinematics
    cd human2robot
    #update data_dirs here too
    python data_collect/glove/kinematics/psyonic_kinematics.py
    #key functions: visualize_ik_trajectory - perform ik
    # play_filtered_commands - visualize ik trajectories

    #output: robot_cmds.pkl
    #convert to robot_cmds.npy before using trajdex/tools/replay
    ```