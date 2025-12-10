import matplotlib.pyplot as plt
import numpy as np
import os
import pickle




def load_data(file_path, start_idx, end_idx):
    with open(file_path, 'rb') as f:
        data = pickle.load(f)
    bowie_data = data["glove"]
    rgb_data = data["rs_color"][start_idx:end_idx]                           
    left_ir = data["rs_ir1"][start_idx:end_idx]    
    right_ir = data["rs_ir2"][start_idx:end_idx]    
    pred_keypoints_2d = data["pred_keypoints_2d"]
    pred_keypoints_3d = data["pred_keypoints_3d"]
    return bowie_data, rgb_data, left_ir, right_ir, pred_keypoints_2d, pred_keypoints_3d  

def load_pipeline_data(parent_dir):
    with open(f"{parent_dir}/cropped_aligned_depth.pkl", 'rb') as f:
        cropped_aligned_depth_data = pickle.load(f)
    with open(f"{parent_dir}/cropped_fs_depth.pkl", 'rb') as f:
        cropped_fs_depth_data = pickle.load(f)
    with open(f"{parent_dir}/cropped_left.pkl", 'rb') as f:
        cropped_left_ir_data = pickle.load(f)
    with open(f"{parent_dir}/cropped_right.pkl", 'rb') as f:
        cropped_right_ir_data = pickle.load(f)
    with open(f"{parent_dir}/cropped_rgb.pkl", 'rb') as f:
        cropped_rgb_data = pickle.load(f)
    with open(f"{parent_dir}/hand_bbox.pkl", 'rb') as f:
        hand_bbox = pickle.load(f)      

    return cropped_aligned_depth_data, cropped_fs_depth_data, cropped_left_ir_data, cropped_right_ir_data, cropped_rgb_data, hand_bbox


def draw_hand_skeleton(self, ax, keypoints):
    """Draw hand skeleton connections between keypoints."""
    # MANO hand model connections (simplified)
    connections = [
        # Thumb
        (0, 1), (1, 2), (2, 3), (3, 4),
        # Index
        (0, 5), (5, 6), (6, 7), (7, 8),
        # Middle  
        (0, 9), (9, 10), (10, 11), (11, 12),
        # Ring
        (0, 13), (13, 14), (14, 15), (15, 16),
        # Pinky
        (0, 17), (17, 18), (18, 19), (19, 20)
    ]
    
    for start_idx, end_idx in connections:
        if start_idx < len(keypoints) and end_idx < len(keypoints):
            start_point = keypoints[start_idx]
            end_point = keypoints[end_idx]
            ax.plot([start_point[0], end_point[0]], 
                    [start_point[1], end_point[1]], 
                    'lime', linewidth=1.5, alpha=0.7)



def plot_img(img, img_name):
    #if 3 channels, convert from BGR to RGB
    if img.ndim==3:
        img = img[:,:,::-1]
        plt.imshow(img)
    if img.ndim==2:
        #set cmap to gray for ir images
        plt.imshow(img, cmap='gray')
    plt.axis('off')
    # plt.show()
    plt.savefig(f'/home/gumdev/human2robot/data/figures/{img_name}.png', bbox_inches='tight', pad_inches=0)
    plt.close()

def plot_depth_img(depth_img, img_name):
    plt.imshow(depth_img, cmap='plasma') #use plasma colormap for depth images
    plt.colorbar(label='Depth (m)')
    plt.axis('off')
    # plt.show()
    plt.savefig(f'/home/gumdev/human2robot/data/figures/{img_name}.png', bbox_inches='tight', pad_inches=0)
    plt.close()



def plot_keypoints_2d(img, keypoints):
    plt.imshow(img[:,:,::-1]) #convert from BGR to RGB
    # plt.imshow(img, cmap='plasma') #for depth
    # plt.scatter(keypoints_2d[:,0], keypoints_2d[:,1], c='r', s=10)
    plt.scatter(keypoints[:, 0], keypoints[:, 1], c='lime', s=15, alpha=0.8, edgecolors='darkgreen', linewidth=0.5)

    connections = [
    # Thumb
    (0, 1), (1, 2), (2, 3), (3, 4),
    # Index
    (0, 5), (5, 6), (6, 7), (7, 8),
    # Middle  
    (0, 9), (9, 10), (10, 11), (11, 12),
    # Ring
    (0, 13), (13, 14), (14, 15), (15, 16),
    # Pinky
    (0, 17), (17, 18), (18, 19), (19, 20)
    ]

    for start_idx, end_idx in connections:
        if start_idx < len(keypoints) and end_idx < len(keypoints):
            start_point = keypoints[start_idx]
            end_point = keypoints[end_idx]
            plt.plot([start_point[0], end_point[0]], 
                    [start_point[1], end_point[1]], 
                    'lime', linewidth=1.5, alpha=0.7)

    plt.axis('off')
    # plt.show()
    plt.savefig(f'/home/gumdev/human2robot/data/figures/keypoints_2d.png', bbox_inches='tight', pad_inches=0)
    plt.close()


def plot_glove_data(bowie_data):

    thumb_mag0 = bowie_data[:,8,:]
    thumb_mag1 = bowie_data[:,9,:]
    delta_mag = thumb_mag1 - thumb_mag0
    fig, axs = plt.subplots(1,3, figsize=(10, 4))
    linewidth_param = 4
    #add overall title for all 3 subplots
    axs[0].plot(delta_mag[:,0], color='red', linewidth=linewidth_param)
    axs[0].set_xlabel('Frame')
    axs[0].set_ylabel('Magnetometer Delta X')
    axs[0].grid()    
    axs[1].plot(delta_mag[:,1], color='green', linewidth=linewidth_param)
    axs[1].set_xlabel('Frame')
    axs[1].set_ylabel('Magnetometer Delta Y')
    axs[1].grid()
    axs[2].plot(delta_mag[:,2], color='blue', linewidth=linewidth_param)
    axs[2].set_xlabel('Frame')
    axs[2].set_ylabel('Magnetometer Delta Z')
    axs[2].grid()
    plt.tight_layout()
    # plt.show()
    plt.savefig('/home/gumdev/human2robot/data/figures/thumb_magnetometer_deltas.png')
    plt.close()

    # pass
    #use wipe0 thumb
#pipeline figure: raw IR images, raw rgb data, glove data

#lang sam - bounding box
#foundationstereo depth map
#wrist shift
#mujoco IK
#policy??

def plot_robot_cmds():
    robot_cmds = np.load("/home/gumdev/human2robot/data/wipe_sea1/wipe0/robot_cmds.npy")
    franka_cmds = robot_cmds[:,:7]
    psyonic_cmds = robot_cmds[:,7:]
    fig, axs = plt.subplots(2,1, figsize=(9,10))
    linewidth_param = 4
    fontsize_param = 30
    for i in range(6):
        axs[0].plot(franka_cmds[:,i], label=f'Joint {i+1}', linewidth=linewidth_param)
    axs[0].set_title('Franka Joint Commands', fontsize=fontsize_param)
    axs[0].set_xlabel('Frame', fontsize=fontsize_param)
    axs[0].set_ylabel('Joint Position (rad)', fontsize=fontsize_param)
    # axs[0].legend(loc='lower right', fontsize='small', ncol=4)
    axs[0].grid()

    for i in range(6):
        axs[1].plot(psyonic_cmds[:,i], label=f'Joint {i+1}', linewidth=linewidth_param)
    axs[1].set_title('Ability Hand Joint Commands', fontsize=fontsize_param)
    axs[1].set_xlabel('Frame', fontsize=fontsize_param)
    axs[1].set_ylabel('Joint Position (rad)', fontsize=fontsize_param)
    # axs[1].legend(loc='lower right', fontsize='small', ncol=4)
    axs[1].grid()

    plt.tight_layout()
    # plt.show()
    plt.savefig('/home/gumdev/human2robot/data/figures/robot_cmds.png')
    plt.close()


def main():
    data_path = "/home/gumdev/human2robot/data/wipe_sea1/wipe0/processed_fixed.pkl"
    parent_dir = "/home/gumdev/human2robot/data/figures/wipe0_frame65"
    # robot_cmds = "/home/gumdev/human2robot/data/wipe_sea1/wipe0/robot_cmds.npy"
    plot_robot_cmds()
    start_idx = 38
    end_idx = -50
    bowie_data, rgb_data, left_ir, right_ir, pred_keypoints_2d, pred_keypoints_3d = load_data(data_path, start_idx, end_idx)
    print(f"CHECK START INDEX:{start_idx} AND END INDEX {end_idx}!")

    img_idx = 65
    # plot_img(rgb_data[img_idx], "rgb_image")
    # plot_img(left_ir[img_idx], "left_ir_image")
    # plot_img(right_ir[img_idx], "right_ir_image")

    # plot_keypoints_2d(rgb_data[img_idx], pred_keypoints_2d[img_idx])

    #intermediate data
    cropped_aligned_depth_data, cropped_fs_depth_data, cropped_left_ir_data, cropped_right_ir_data, cropped_rgb_data, hand_bbox = load_pipeline_data(parent_dir)
    # plot_img(cropped_rgb_data, "cropped_rgb_image")
    # plot_img(cropped_left_ir_data, "cropped_left_ir_image")
    # plot_img(cropped_right_ir_data, "cropped_right_ir_image")
    
    # plot_depth_img(cropped_aligned_depth_data, "cropped_aligned_depth_image")
    # plot_depth_img(cropped_fs_depth_data, "cropped_fs_depth_image")
    # plot_keypoints_2d(cropped_fs_depth_data, pred_keypoints_2d[img_idx])

    #bowie data
    # plot_glove_data(bowie_data)


    







if __name__ == "__main__":
    main()