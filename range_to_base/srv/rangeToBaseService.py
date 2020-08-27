#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Range
from sensor_msgs.msg import PointCloud2
#import laser_geometry.laser_geometry as lg
import math
from scipy import optimize
import numpy as np

from range_to_base.srv import RangeToBase, RangeToBaseResponse
from laser_tools_src2.srv import ScanToPointCloud2

####### circle fit code ##########
# https://scipy-cookbook.readthedocs.io/items/Least_Squares_Circle.html
# define x,y globally
x=np.empty(0)
y=np.empty(0)
def calc_R(xc, yc):
    """ calculate the distance of each data points from the center (xc, yc) """

    return np.sqrt((x-xc)**2 + (y-yc)**2)

def f_2b(c):
    """ calculate the algebraic distance between the 2D points and the mean circle centered at c=(xc, yc) """
    Ri = calc_R(*c)
    return Ri - Ri.mean()

def Df_2b(c):
    """ Jacobian of f_2b
    The axis corresponding to derivatives must be coherent with the col_deriv option of leastsq"""
    xc, yc     = c
    df2b_dc    = np.empty((len(c), x.size))

    Ri = calc_R(xc, yc)
    df2b_dc[0] = (xc - x)/Ri                   # dR/dxc
    df2b_dc[1] = (yc - y)/Ri                   # dR/dyc
    df2b_dc    = df2b_dc - df2b_dc.mean(axis=1)[:, np.newaxis]

    return df2b_dc


def range(mess):
    global x,y,returnVal


    # call rotate in place service


    # call the point cloud
    rospy.wait_for_service("scan_to_cloud")
    clouder = rospy.ServiceProxy("scan_to_cloud", ScanToPointCloud2);
    resp_cloud = clouder(mess.angle,mess.angle, 1, 10);


    # convert it to a generator of the individual points
    point_generator = pc2.read_points(resp_cloud.cloud)
    rangeMeas = Range()

    # we can access a generator in a loop

    x=np.delete(x,np.arange(len(x)))
    y=np.delete(y,np.arange(len(y)))
    for point in point_generator:
        if not (math.isnan(point[0]) & math.isnan(point[1])):
            x=np.append(x,point[0])
            y=np.append(y,point[1])

    x_m = np.mean(x)
    y_m = np.mean(y)

    center_estimate = x_m, y_m
    ## try the least squares fit to a circle
    try:
        center_final_est, ier = optimize.leastsq(f_2b, center_estimate, Dfun=Df_2b, col_deriv=True)
 	R_est = calc_R(*center_final_est)
    	R_avg= R_est.mean()
    	xc_2, yc_2 = center_final_est

    	residu_2   = sum((R_est - R_avg)**2)
    # if least squares fit circle fit fails, assume robot is outside the crater
    except TypeError:
        ier=-1
    # if least squares circle fit succeeds, test further
    if ier>0:
	xx=np.mean(xc_2)
	yy=np.mean(yc_2)
        print 'Time: %5.2f Rad: %5.2f cen: %5.2f %5.2f Res: %5.2f'%(resp_cloud.cloud.header.stamp.to_sec(),R_avg, np.mean(xc_2), np.mean(yc_2),residu_2)
        rangeMeas.range =  np.sqrt(np.mean(xc_2)**2+np.mean(yc_2)**2)
        rangeMeas.header.stamp = rospy.Time.now()
	# check for 'goodness of fit' by looking at size of radius, residuals, x, y
	if residu_2<1000:
		returnVal=rangeMeas
                return returnVal
        else:

                rangeMeas.range = 999.99

                returnVal=rangeMeas
                return returnVal




rospy.init_node('range_to_base')

service=rospy.Service('range_to_base_service',RangeToBase,range)

rospy.spin()
