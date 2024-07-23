import unittest
import numpy as np
import numpy.testing as npt
import open3d as o3d

from detect_floor import detect_floor_and_re_orient_pcd


# Generate a random rotation matrix
def random_rotation_matrix():
    theta_x = np.random.uniform(0, 2 * np.pi)
    theta_y = np.random.uniform(0, 2 * np.pi)
    theta_z = np.random.uniform(0, 2 * np.pi)

    R_x = np.array([[1, 0, 0],
                    [0, np.cos(theta_x), -np.sin(theta_x)],
                    [0, np.sin(theta_x), np.cos(theta_x)]])
    
    R_y = np.array([[np.cos(theta_y), 0, np.sin(theta_y)],
                    [0, 1, 0],
                    [-np.sin(theta_y), 0, np.cos(theta_y)]])
    
    R_z = np.array([[np.cos(theta_z), -np.sin(theta_z), 0],
                    [np.sin(theta_z), np.cos(theta_z), 0],
                    [0, 0, 1]])
    
    R = R_z @ R_y @ R_x  # Combined rotation matrix
    return R

# Generate a random translation vector
def random_translation_vector():
    t = np.random.uniform(-1, 1, 3)  # Random translation in x, y, z directions
    return t

# Apply the random transformation to the point cloud
def apply_random_transformation(pcd):
    R = random_rotation_matrix()
    t = random_translation_vector()
    
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = R
    transformation_matrix[:3, 3] = t
    
    pcd.transform(transformation_matrix)

    return pcd

# Function to generate a synthetic point cloud
def create_synthetic_point_cloud():
    y = np.linspace(-5, 5, 100)
    z = np.linspace(-5, 5, 100)
    Y, Z = np.meshgrid(y, z)
    X = np.zeros(Y.shape)
    points = np.vstack((X.flatten(), Y.flatten(), Z.flatten())).T
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd

class TestPointCloudTransformation(unittest.TestCase):

    def test_random_transformation(self):

        original_pt_cloud_path = "/Users/indu.cherukuri/Desktop/PC_reorient/floor_detection/pcs/chair_pc.ply"
        original_pcd = o3d.io.read_point_cloud(original_pt_cloud_path)
        # calling detect floor and re-orient pcd function
        re_oriented_pcd, plane_model1 = detect_floor_and_re_orient_pcd(original_pcd) #original point cloud
        re_oriented_pcd_points = np.asarray(re_oriented_pcd.points)

        # performing a random transformation on original point cloud
        rand_transformed_pcd = apply_random_transformation(original_pcd)

        # calling detect floor and re-orient pcd function on original point clould which is randomly transformed
        re_oriented_rand_transformed_pcd, plane_model2 = detect_floor_and_re_orient_pcd(original_pcd)
        re_oriented_rand_transformed_pcd_points = np.asarray(re_oriented_rand_transformed_pcd.points)

        # find normals of the floor in 2 pointclouds
        print(plane_model1, plane_model2)
        npt.assert_array_almost_equal(plane_model1, plane_model2, decimal=2)




if __name__=="__main__":

    unittest.main()