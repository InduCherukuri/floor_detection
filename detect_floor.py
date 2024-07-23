import numpy as np
import open3d as o3d
import glob

# Function to compute the rotation matrix using Rodrigues formula
def rodrigues_rotation_matrix(axis, angle):
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0]
    ])
    I = np.eye(3)
    R = I + np.sin(angle) * K + (1 - np.cos(angle)) * np.dot(K, K)
    return R

def detect_floor_and_re_orient_pcd(original_pcd, voxel_size=0.01, nb_neighbors=20, std_ratio=2.0, radius=0.1, max_nn=30, distance_threshold=0.02, ransac_n=3, num_iterations=1000):

    # Downsample the point cloud
    pcd = original_pcd.voxel_down_sample(voxel_size=voxel_size)

    # Remove outliers
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    clean_pcd = pcd.select_by_index(ind)

    o3d.visualization.draw_geometries([clean_pcd], window_name="Cleaned Point Cloud")

    # Estimate normals
    clean_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn))

    # Segment the largest plane using RANSAC
    plane_model, inliers = clean_pcd.segment_plane(distance_threshold=distance_threshold, ransac_n=ransac_n, num_iterations=num_iterations)
    [a, b, c, d] = plane_model

    print(f"Plane model: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    # Extract the inlier points (the detected plane)
    inlier_cloud = clean_pcd.select_by_index(inliers)

    # Visualize the point cloud and the detected plane
    # clean_pcd.paint_uniform_color([0.5, 0.5, 0.5])  # Gray for original point cloud
    inlier_cloud.paint_uniform_color([1, 0, 0])     # Red for the detected plane

    o3d.visualization.draw_geometries([clean_pcd, inlier_cloud], window_name="Floor Detection")

    # Calculate the centroid of the floor plane
    centroid = np.mean(np.asarray(inlier_cloud.points), axis=0)

    # Translate the point cloud to bring the centroid to the origin
    translation = -centroid
    clean_pcd.translate(translation)
    inlier_cloud.translate(translation)

    o3d.visualization.draw_geometries([clean_pcd], window_name="Cleaned and Centered Point Cloud")

    # Calculate the normal vector and target vector
    normal = np.array([a, b, c])
    normal /= np.linalg.norm(normal)
    target = np.array([0, -1, 0])
    print("Normal Vector ", normal, "Target Vector ", target)

    # Calculate the axis and angle for rotation
    axis = np.cross(normal, target)
    axis /= np.linalg.norm(axis)
    angle = np.arccos(np.dot(normal, target))

    print("Axis ", axis, "Angle ", angle)

    # Compute the rotation matrix
    R = rodrigues_rotation_matrix(axis, angle)

    # Apply the rotation to the point cloud
    clean_pcd.rotate(R, center=(0, 0, 0))

    # Save and visualize the rotated and translated point cloud
    o3d.io.write_point_cloud("rotated_and_translated_point_cloud.ply", clean_pcd)
    o3d.visualization.draw_geometries([clean_pcd], window_name="Rotated and Translated Point Cloud")

    # Segment the largest plane using RANSAC
    plane_model_updated, _ = clean_pcd.segment_plane(distance_threshold=distance_threshold, ransac_n=ransac_n, num_iterations=num_iterations)
    

    return clean_pcd, plane_model_updated

def surface_reconstruction(updated_pcd, depth=9, density_threshold=0.01):

    # Perform Poisson surface reconstruction
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(updated_pcd, depth=depth)

    # Remove low-density vertices to clean up the mesh
    vertices_to_remove = densities < np.quantile(densities, density_threshold)
    mesh.remove_vertices_by_mask(vertices_to_remove)

    # Visualize the mesh
    o3d.visualization.draw_geometries([mesh], window_name="Smooth Surface Reconstruction")

    # Optionally, save the mesh to a file
    o3d.io.write_triangle_mesh("smooth_surface.ply", mesh)



if __name__=="__main__":

    
    point_cloud_paths = glob.glob("./pcs/*.ply")
    for pcd_path in point_cloud_paths:

        # Load the point cloud
        original_pcd = o3d.io.read_point_cloud(pcd_path)
        # Vsualizing the original point cloud
        o3d.visualization.draw_geometries([original_pcd], window_name="Original Point Cloud")

        # Detecting the floor and re-orient the floor to YZ plane
        # Experimental Params: voxel_size=0.01, nb_neighbors=20, std_ratio=2.0, radius=0.1, max_nn=30, distance_threshold=0.02, ransac_n=3, num_iterations=1000
        updated_pcd, plane_model = detect_floor_and_re_orient_pcd(original_pcd)

        # Smoothen the scene representation using poisson surface reconstruction
        surface_reconstruction(updated_pcd)