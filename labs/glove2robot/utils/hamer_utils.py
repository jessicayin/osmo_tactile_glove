import open3d as o3d
import numpy as np
from scipy.optimize import minimize

def ransac(sim_pcd, real_pcd, threshold=0.04, iterations=1000):

    # If sim_pcd and real_pcd have different number of points, randomly sample the smaller one
    if sim_pcd.shape[0] != real_pcd.shape[0]:   
        N= min(sim_pcd.shape[0], real_pcd.shape[0])
        sim_pcd = sim_pcd[:N]
        real_pcd = real_pcd[:N]
    best_inlier_count = 0
    best_transformation = None
    best_inliers = None
    
    for _ in range(iterations):
        sample_indices = np.random.choice(N, 3, replace=False)
        sim_sample = sim_pcd[sample_indices]
        real_sample = real_pcd[sample_indices]
        
        centroid_sim = np.mean(sim_sample, axis=0)
        centroid_real = np.mean(real_sample, axis=0)
        
        sim_centered = sim_sample - centroid_sim
        real_centered = real_sample - centroid_real
        
        H = sim_centered.T @ real_centered
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        
        if np.linalg.det(R) < 0:
            Vt[2, :] *= -1
            R = Vt.T @ U.T
            
        t = centroid_real - R @ centroid_sim
        
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        
        sim_homo = np.hstack((sim_pcd, np.ones((N, 1))))
        sim_transformed = (T @ sim_homo.T).T[:, :3]
        
        errors = np.linalg.norm(sim_transformed - real_pcd, axis=1)
        inliers = errors < threshold
        inlier_count = np.sum(inliers)
        
        if inlier_count > best_inlier_count:
            best_inlier_count = inlier_count
            best_transformation = T
            best_inliers = inliers
            
    if best_inlier_count > 3:
        sim_inliers = sim_pcd[best_inliers]
        real_inliers = real_pcd[best_inliers]
        centroid_sim = np.mean(sim_inliers, axis=0)
        centroid_real = np.mean(real_inliers, axis=0)
        sim_centered = sim_inliers - centroid_sim
        real_centered = real_inliers - centroid_real
        
        H = sim_centered.T @ real_centered
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[2, :] *= -1
            R = Vt.T @ U.T
        t = centroid_real - R @ centroid_sim
        
        best_transformation = np.eye(4)
        best_transformation[:3, :3] = R
        best_transformation[:3, 3] = t
        
    return real_pcd[best_inliers]

def compute_nearest_neighbors(source_points, target_points):
    # Create a KD-tree from the source points
    source_pcd = o3d.geometry.PointCloud()
    source_pcd.points = o3d.utility.Vector3dVector(source_points)
    source_kd_tree = o3d.geometry.KDTreeFlann(source_pcd)
    distances = []
    for target_point in target_points:
        [_, idx, _] = source_kd_tree.search_knn_vector_3d(target_point, 1)
        nearest_source_point = source_points[idx[0]]
        distances.append(nearest_source_point - target_point)
    return np.array(distances)
def objective_function(translation, source_points, target_points):
    translated_target_points = target_points + translation
    distances = compute_nearest_neighbors(source_points, translated_target_points)
    return np.sum(np.linalg.norm(distances, axis=1) ** 2)
def optimize_translation(source, target):
    # Load point clouds
    source_points = np.asarray(source.points)
    target_points = np.asarray(target.points)
    # Initial guess for translation
    initial_translation = np.zeros(3)
    # Optimize translation
    result = minimize(objective_function, initial_translation, args=(source_points, target_points))
    return result.x

def perform_icp_registration(hand_mesh, hand_pcd, threshold=0.06, nb_neighbors=30, std_ratio=0.02):
    """
    Perform ICP registration between a hand mesh and point cloud.
    
    Args:
        hand_mesh (trimesh.Trimesh): The source hand mesh
        hand_pcd (np.ndarray): Target point cloud of the hand
        threshold (float): Maximum correspondence distance for ICP
        nb_neighbors (int): Number of neighbors for statistical outlier removal
        std_ratio (float): Standard deviation ratio for outlier removal
        
    Returns:
        o3d.pipelines.registration.RegistrationResult: The ICP registration result
    """

    # Sample points from mesh and convert to Open3D PointCloud
    points_source = hand_mesh.sample(5000)
    pcd_source = o3d.geometry.PointCloud()
    pcd_source.points = o3d.utility.Vector3dVector(points_source)
    
    # Convert target points to Open3D PointCloud and remove outliers
    pcd_target = o3d.geometry.PointCloud()
    pcd_target.points = o3d.utility.Vector3dVector(hand_pcd)

    # Apply RANSAC-based outlier removal to target point cloud
    # if len(pcd_target.points) > 0:
        # Estimate a plane using RANSAC
    #     plane_model, inliers = pcd_target.segment_plane(distance_threshold=0.01,
    #                                                     ransac_n=3,
    #                                                     num_iterations=1000)
    #     # Get outliers (non-plane points which are likely hand points)
    #     outlier_cloud = pcd_target.select_by_index(inliers, invert=True)
        
    #     # If we have valid points after RANSAC, use the outlier cloud (hand points)
    #     if len(outlier_cloud.points) > 0:
    #         pcd_target = outlier_cloud
        
    #     print(f"RANSAC removed {len(inliers)} plane points, {len(pcd_target.points)} hand points remain")

    # # remove outliers
    # ransac_pcd_target, ind = pcd_target.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    # pcd_target_arr = ransac(np.asarray(pcd_source.points), np.asarray(pcd_target.points), threshold=0.01, iterations=1000)
    pcd_target_arr = np.asarray(pcd_target.points)
    # pcd_target = o3d.geometry.PointCloud()
    # pcd_target.points = o3d.utility.Vector3dVector(pcd_target_arr)
    
    # Perform ICP registration
    initial_tf = np.eye(4)
    print("human2robot repo utils")

    init_trans = optimize_translation(pcd_source, pcd_target)
    # init_trans = np.zeros_like(init_trans)
    print(init_trans)
    pcd_source.points = o3d.utility.Vector3dVector(np.array(pcd_source.points) - init_trans)
    
    reg_icp = o3d.pipelines.registration.registration_icp(
        pcd_source, pcd_target, threshold, initial_tf,
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
        
    print("Estimated transformation from ICP:")
    print(reg_icp.transformation, init_trans)
    
    return reg_icp, init_trans, pcd_target_arr