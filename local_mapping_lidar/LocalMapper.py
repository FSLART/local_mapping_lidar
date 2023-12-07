import rclpy 
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from lart_msgs.msg import Cone, ConeArray

from tf2_ros import TransformListener, Buffer, TransformStamped, TransformException

import open3d as o3d
import numpy as np

import threading

class LocalMapper(Node):

    def __init__(self):
        super().__init__('local_mapper')

        # point cloud subscriber
        self.pcd_sub = self.create_subscription(PointCloud2, '/pointcloud', self.pcd_callback, 10)

        # cone publisher
        self.cone_pub = self.create_publisher(ConeArray, '/cones', 10)

        # marker publisher
        self.marker_pub = self.create_publisher(MarkerArray, '/markers', 10)

        # create the transform buffer
        self.tf_buffer = Buffer()

        # create the transform listener
        self.tf_listener = TransformListener(self.tf_buffer, self)


    def pcd_callback(self, msg):

        # convert ros point cloud to open3d point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(msg.data, dtype=np.float32).reshape(-1, 3))

        # transform the point cloud to the base_link frame
        # lookup the transform
        try:
            transform = self.tf_buffer.lookup_transform('base_link', msg.header.frame_id, msg.header.stamp)
        except TransformException as e:
            self.get_logger().warn(e)
            return
        
        # convert the transform to a 4x4 homogeneous matrix
        transform_matrix = np.array([[transform.transform.rotation.w, -transform.transform.rotation.z, transform.transform.rotation.y, transform.transform.translation.x],
                                        [transform.transform.rotation.z, transform.transform.rotation.w, -transform.transform.rotation.x, transform.transform.translation.y],
                                        [-transform.transform.rotation.y, transform.transform.rotation.x, transform.transform.rotation.w, transform.transform.translation.z],
                                        [0.0, 0.0, 0.0, 1.0]])
        # invert the transform (sensor to base_link instead)
        transform_matrix = np.linalg.inv(transform_matrix)
        
        # transform the point cloud
        pcd.transform(transform_matrix)

        # remove ground plane using RANSAC
        pcd = self.remove_ground_plane(pcd)

        # cluster the cones using DBSCAN
        cone_coords = self.cluster_cones(pcd)

        # create the ConeArray message
        cone_array = self.to_cone_array(cone_coords)

        # create the MarkerArray message
        marker_array = self.to_marker_array(cone_coords)

        # publish the cone array message
        self.cone_pub.publish(cone_array)

        # publish the marker array message
        self.marker_pub.publish(marker_array)


    def remove_ground_plane(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:

        # remove the ground plane using RANSAC
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                                ransac_n=3,
                                                num_iterations=1000)
        
        # get the outlier points (without the ground plane)
        outlier_cloud = pcd.select_by_index(inliers, invert=True)

        return outlier_cloud
    
    def cluster_cones(self, pcd: o3d.geometry.PointCloud) -> list:

        # get the clusters using DBSCAN
        labels = np.array(pcd.cluster_dbscan(eps=0.02, min_points=10))

        # get the cluster centers
        cluster_centers = []

        # list of threads
        threads = []

        # mutex for the cluster centers list
        mutex = threading.Lock()

        # for each cluster
        # use multi-threading to speed up the process
        for label in np.unique(labels):
            if label == -1:
                continue

            t = threading.Thread(target=cluster_center_routine, args=(pcd, label, cluster_centers, mutex))
            threads.append(t)
            t.start()

        # wait for all threads to finish
        for t in threads:
            t.join()

        return cluster_centers
    
    def to_cone_array(self, cluster_centers: list) -> ConeArray:

        # create the ConeArray message
        cone_array = ConeArray()

        # for each cluster center
        for cluster_center in cluster_centers:
            # create a cone message
            cone = Cone()
            # set the cone message fields
            cone.x = cluster_center[0]
            cone.y = cluster_center[1]
            cone.z = cluster_center[2]
            # append the cone message to the cone array message
            cone_array.cones.append(cone)

        return cone_array
    
    def to_marker_array(self, cluster_centers: list) -> MarkerArray:
            
        # create the MarkerArray message
        marker_array = MarkerArray()

        # for each cluster center
        for cluster_center in cluster_centers:
            # create a marker message
            marker = Marker()
            # set the marker message fields
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = cluster_center[0]
            marker.pose.position.y = cluster_center[1]
            marker.pose.position.z = cluster_center[2]
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            # append the marker message to the marker array message
            marker_array.markers.append(marker)

        return marker_array

def cluster_center_routine(pcd: o3d.geometry.PointCloud, label: o3d.utility.IntVector, centers: list, mutex: threading.Lock) -> None:

    # get cluster using the label
    cluster = pcd.select_by_index(np.where(label == label)[0])

    # get the geometric center of the cluster
    cluster_center = cluster.get_center()

    # lock the mutex
    mutex.acquire()

    # append the cluster center to the list of centers
    centers.append(cluster_center)

    # release the mutex
    mutex.release()


def main(args=None):
    rclpy.init(args=args)

    local_mapper = LocalMapper()

    rclpy.spin(local_mapper)

    local_mapper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()