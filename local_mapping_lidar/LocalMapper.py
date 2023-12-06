import rclpy 
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from lart_msgs.msg import Cone, ConeArray

import open3d as o3d

class LocalMapper(Node):

    def __init__(self):
        super().__init__('local_mapper')

        # point cloud subscriber
        self.pcd_sub = self.create_subscription(PointCloud2, '/pointcloud', self.pcd_callback, 10)

        # cone publisher
        self.cone_pub = self.create_publisher(ConeArray, '/cones', 10)

        # marker publisher
        self.marker_pub = self.create_publisher(MarkerArray, '/markers', 10)


    def pcd_callback(self, msg):
        # TODO: convert ros point cloud to open3d point cloud
        # TODO: transform the point cloud to the base_link frame
        # TODO: remove ground plane using RANSAC
        # TODO: create clusters using DBSCAN
        # TODO: publish the center of each cluster as a marker and as a cone

        print("TODO: received a point cloud message")

    def remove_ground_plane(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:

        # remove the ground plane using RANSAC
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                                ransac_n=3,
                                                num_iterations=1000)
        
        # get the outlier points (without the ground plane)
        outlier_cloud = pcd.select_by_index(inliers, invert=True)





def main(args=None):
    rclpy.init(args=args)

    local_mapper = LocalMapper()

    rclpy.spin(local_mapper)

    local_mapper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()