#! /usr/bin/env python

import rospy
import std_msgs.msg
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import numpy as np

def get_quadrant(point):
    res = 0
    x = point[0]
    y = point[1]
    if x > 0 and y >= 0:
        res = 1
    elif x <= 0 and y > 0:
        res = 2
    elif x < 0 and y <= 0:
        res = 3
    elif x >= 0 and y < 0:
        res = 4
    return res

def add_ring_info(scan_points):
    num_of_points = scan_points.shape[0]
    scan_points = np.hstack([scan_points,
                                np.zeros((num_of_points, 1))])
    velodyne_rings_count = 64
    previous_quadrant = 0
    ring = 0
    for num in range(num_of_points-1, -1, -1):
        quadrant = get_quadrant(scan_points[num])
        if quadrant == 4 and previous_quadrant == 1 and ring < velodyne_rings_count-1:
            ring += 1
        scan_points[num, 4] = ring
        previous_quadrant = quadrant
    return scan_points

def cal_ring_64(scan):
    # get ring channel
    depth = np.linalg.norm(scan, 2, axis=1)
    pitch = np.arcsin(scan[:, 2] / (depth + 0.00001)) # arcsin(z, depth)
    fov_down = -24.8 / 180.0 * np.pi
    fov = (abs(-24.8) + abs(2.0)) / 180.0 * np.pi
    proj_y = (pitch + abs(fov_down)) / fov  # in [0.0, 1.0]
    proj_y *= 63  # in [0.0, H]
    proj_y = np.round(proj_y)
    proj_y = np.minimum(63, proj_y)
    proj_y = proj_y.reshape(-1, 1)
    return(proj_y)

def cal_ring_32(scan):
    # get ring channel
    depth = np.linalg.norm(scan, 2, axis=1)
    pitch = np.arcsin(scan[:, 2] / (depth + 0.00001)) # arcsin(z, depth)
    fov_down = -30.67 / 180.0 * np.pi
    fov = (abs(-30.67) + abs(10.67)) / 180.0 * np.pi
    proj_y = (pitch + abs(fov_down)) / fov  # in [0.0, 1.0]
    proj_y *= 31  # in [0.0, H]
    proj_y = np.round(proj_y)
    proj_y = np.minimum(63, proj_y)
    proj_y = proj_y.reshape(-1, 1)
    return(proj_y)

def cal_ring_16(scan):
    # get ring channel
    depth = np.linalg.norm(scan, 2, axis=1)
    pitch = np.arcsin(scan[:, 2] / (depth + 0.00001)) # arcsin(z, depth)
    fov_down = -15 / 180.0 * np.pi
    fov = (abs(-15) + abs(15)) / 180.0 * np.pi
    proj_y = (pitch + abs(fov_down)) / fov  # in [0.0, 1.0]
    proj_y *= 15  # in [0.0, H]
    proj_y = np.round(proj_y)
    proj_y = np.minimum(15, proj_y)
    proj_y = proj_y.reshape(-1, 1)
    return(proj_y)

def callback(msg):
    global frame
    pt_x = []
    pt_y = []
    pt_z = []
    pt_i = []

    points = pc2.read_points(msg, field_names = ("x", "y", "z", "intensity"), skip_nans=False)

    for point in points:
        pt_x.append(point[0])
        pt_y.append(point[1])
        pt_z.append(point[2])
        # pt_i.append(point[3])
        pt_i.append(1.0)
    points_cloud = np.transpose(np.vstack((pt_x, pt_y, pt_z, pt_i)))
    # points_cloud = points_cloud[:-1, :]

    # points_ring = add_ring_info(points_cloud)
    if num_rings == 64:
        pt_r = cal_ring_64(points_cloud[:, 0:3])
    elif num_rings == 32:
        pt_r = cal_ring_32(points_cloud[:, 0:3])
    elif num_rings == 16:
        pt_r = cal_ring_16(points_cloud[:, 0:3])
    points_ring = np.hstack((points_cloud, pt_r))

    # print(pt_r)
    # print(msg.header.stamp)
    print(frame)
    frame+=1

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          PointField('intensity', 16, PointField.FLOAT32, 1),
          PointField('ring', 20, PointField.UINT16, 1),
          ]

    #header
    header = std_msgs.msg.Header()
    header.stamp = msg.header.stamp
    header.frame_id = 'velodyne'
    #create pcl from points
    points_new = pc2.create_cloud(header, fields, points_ring)
    pub.publish(points_new)
    # rospy.loginfo(str(header.stamp) + ": point cloud recorded")

if __name__ == '__main__':
    frame = 1
    num_rings = 16
    rospy.init_node('points_save', anonymous=True)
    sub = rospy.Subscriber('/rslidar_points_no_ground', PointCloud2, callback)
    pub = rospy.Publisher('/velodyne_points', PointCloud2, queue_size=10)
    rospy.spin()
