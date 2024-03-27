## Early sensor fusion of camera and LIDAR

### Description
The code is implemented in ROS2. This project projects a point cloud data obtained by Lidar sensor on an image from RGB camera.

### Acknowledgement
A large part of this project was inspired by the repository:
https://github.com/EPVelasco/lidar-camera-fusion/tree/main



### Theory
Point projection from 3D world to image
![[Pasted image 20231018134725.png]]![[Pasted image 20231018134742.png]]
Extrinsic matrix tell us how to transform a point from world coordinate to 3d camera coordinate. This is equivalent to knowing the position of world coordinate wrt 3D camera coordinate. 

In this problem, we consider the transformation from LiDAR coordinate to 3D camera coordinate. This means **we need the position and orientation of LiDAR wrt to camera**.

Mathematical formular:
$$
	Y = P\times (LiDAR|Cam \times X)
$$
in which 
- $Y$ is the pixel coordinate
- $X$ is the 3D coordinate of point in pointcloud wrt to LiDAR,
- $LiDAR|Cam$ is the matrix transforming points from LiDAR coordinate frame to 3D camera coordinate frame. Or in other words the position of Lidar coordinate wrt camera coordinate
- $P$ is the intrinsic matrix of the camera.

$P$ is a 3x4 matrix to be homogenous with the last column being all 0.

### Dependencies
- pcl
- pcl_conversion
- message_filter
- opencv
- nlohmann json library for reading camera matrix in json file
- ament_index_cpp: Get the path to the shared directory of a package after installed. 