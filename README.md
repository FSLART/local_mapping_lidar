# local_mapping_lidar

Local Mapping algorithm for LiDAR sensor and monocular camera.

The algorithm is as follows:
- The ground plane is removed from the LiDAR point cloud using RANSAC.
- The outlier points (not on the ground plane) are clustered using DBSCAN.
- On the camera image, a bounding box is cropped for each detected cone of the point cloud,, by knowing the LiDAR resolution and the camera model.
- The cone is classified by its class (or as a "not cone") by a simple convolutional neural network.
- For each cone classified, associate its class to the previous detected position.

### Inputs
- `/pointcloud` (`sensor_msgs/PointCloud2`)
- `/image` (`sensor_msgs/Image`)
- `/camera_info` (`sensor_msgs/CameraInfo`)

### Outputs
- `/cones` (`lart_msgs/ConeArray`)

### Required transforms
- `base_link` -> `lidar_link`
- `base_link` -> `camera_link`

## Dependencies
- ROS Humble
- CMake
- C++ compiler
- Open3D
- LibTorch
- [lart_msgs](https://github.com/FSLART/lart_msgs)
