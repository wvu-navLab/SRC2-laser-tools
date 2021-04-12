#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Range
from sensor_msgs.msg import PointCloud2
#import laser_geometry.laser_geometry as lg
import math
from scipy import optimize
import numpy as np
import pcl
from numpy import linalg as LA

from range_to_rover.srv import RangeToRover, RangeToRoverResponse
from laser_tools_src2.srv import ScanToPointCloud2

####### circle fit code ##########
# https://scipy-cookbook.readthedocs.io/items/Least_Squares_Circle.html
# define x,y globally
x = np.empty(0)
y = np.empty(0)
z = np.empty(0)

max_height_thresh = 0.5
min_height_thresh = 0.3


def range(mess):
    global x, y, z, returnVal

    cloud = pcl.PointCloud()

    # call rotate in place service

    # call the point cloud
    rospy.wait_for_service("scan_to_cloud")
    clouder = rospy.ServiceProxy("scan_to_cloud", ScanToPointCloud2)
    resp_cloud = clouder(mess.angle, mess.angle, 1, 10)

    # convert it to a generator of the individual points
    point_generator = pc2.read_points(resp_cloud.cloud)
    rangeMeas = Range()

    # we can access a generator in a loop

    x = np.delete(x, np.arange(len(x)))
    y = np.delete(y, np.arange(len(y)))
    z = np.delete(z, np.arange(len(z)))
    for point in point_generator:
        if not (math.isnan(point[0]) & math.isnan(point[1])):
            x = np.append(x, point[0])
            y = np.append(y, point[1])
            z = np.append(y, point[2])

    mat = np.stack(x, y, z)

    cloud.from_array(points)

    # seg = cloud.make_segmenter_normals(ksearch=50)
    # seg.set_optimize_coefficients(True)
    # seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
    # seg.set_method_type(pcl.SAC_RANSAC)
    # seg.set_distance_threshold(0.01)
    # seg.set_normal_distance_weight(0.01)
    # seg.set_max_iterations(100)
    # indices, coefficients = seg.segment()

    i = 0
    nr_points = cloud.size

    tree = cloud.make_kdtree()
    ec = cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(25000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    print('cluster_indices : ' + str(cluster_indices.count) + " count.")

    for j, indices in enumerate(cluster_indices):
        # cloudsize = indices
        print('indices = ' + str(len(indices)))
        # cloudsize = len(indices)
        points = np.zeros((len(indices), 3), dtype=np.float32)
        # points = np.zeros((cloudsize, 3), dtype=np.float32)

        # for indice in range(len(indices)):
        for i, indice in enumerate(indices):
            # print('dataNum = ' + str(i) + ', data point[x y z]: ' + str(cloud_filtered[indice][0]) + ' ' + str(cloud_filtered[indice][1]) + ' ' + str(cloud_filtered[indice][2]))
            # print('PointCloud representing the Cluster: ' + str(cloud_cluster.size) + " data points.")
            points[i][0] = cloud_filtered[indice][0]
            points[i][1] = cloud_filtered[indice][1]
            points[i][2] = cloud_filtered[indice][2]

        centroid = np.mean(points, axis=1)

        if (centroid[2] > min_height_thresh) and (centroid[2] < max_height_thresh):
            rangeMeas.range = LA.norm(centroid)
            rangeMeas.header.stamp = rospy.Time.now()

    returnVal = rangeMeas
    return returnVal


rospy.init_node('rover_lidar_localization')

service = rospy.Service('range_to_rover_service', RangeToRover, range)

rospy.spin()
