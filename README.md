# floor_detection
Detect floor in the point cloud and re-orient it and perform surface reconstruction on the transformed point cloud

## Installation

pip install open3d numpy

## Usage
##### To detect floor and re-orient the floor and perform surface reconstruction on each of 10 point clouds
    python detect_floor.py

##### To run the unit testcase:
    python -m unittest unit_testcase.py


## Detailed Aproach

I broke down the problem statement into following parts:

1. Detect the floor in any given point cloud
2. Translate the center of the floor to lie on the origin
3. Re-orient the floor to YZ plane such that normal is [1, 0, 0] (will also result in point cloud rotation)
4. Run surface reconstruction algorithm for a smoother representation of the scene compared to point clould
5. Run unit test to check the correctness of the above method.


###### Preprocessing the point cloud:
1. Downsample the point cloud for faster algorithmic excecution.
2. Now remove the outliers so our next steps are more accurate.

###### Detecting floor by plane segmentation:
1. We can find the floor/planar region by randomly sampling points from the point clould and trying to fit a plane and repeat this process for n_iteraions. (segment_plane function is used for this)
2. We also calculate normal for each point in the point cloud to aid in determiing the orientation of the plane. (internally used in segment_plane function)
3. Visually verify the detected floor plane

###### Translate the point cloud such that center of the floor lies on the origin:
1. Compute the centroid of the detected floor plane
2. Translate the point cloud to bring centroid to origin

###### Re-orient the floor such that the equation of floor is y=0
1. Calculate rotation from Rodrigues formula by figuring out the angle and axis of roation
2. we get normal from the detected plane and the target normal is [0, -1, 0] based on our requirement that equation of floor has to be y=0 and objects have to be in commonsense pose.
3. Calculate angle from dot product of both vectors and axis from cross product.
4. From the above we get the rotation matrix and we rotate the point cloud and center it at (0, 0, 0)

###### Surface reconstruction
1. For a more smoother and continous representation of the scene we perform Poisson Surface Reconstruction.

###### Unit Testcase
1. Create a random transformation matrix and apply that transformation on the original point cloud.
2. Verify if the normals after applying detect_floor_and_re_orient_pcd function on original point cloud are same as that of randomly transformed point cloud.

## Results
* Shoe2 original pcd
  
![Alt text](images/shoe2_pcd_after_cleaning.png)

* Shoe2 pcd with floor detected and represented in red
  
![Alt text](images/Shoe2_floor_detection.png)

* Shoe2 re-oriented and centered at origin pcd

![Alt text](images/shoe2_rotated_and_translated.png)

* Shoe2 poisson surface reconstruction

![Alt text](images/Shoe2_pcd.png)



