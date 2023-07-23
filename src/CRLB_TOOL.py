import random
import numpy as np

from numpy import dot, sum, tile, linalg
from numpy.linalg import inv 
from numpy import *
import matplotlib.pyplot as plt
from math import *
import random

from TOOLBOX import *

# need modified
def Static_CRLB_toa(ref_x, ref_y, x ,y, R):

    dist = distance(ref_x, ref_y, x, y)
    c = 3.*pow(10,8)
    H = array( [[ ( x - ref_x ) / dist / c],[ (y- ref_y  ) / dist / c]])
    gamma = 10
    v = gamma * pow(( x - ref_x ),2) / pow(dist, 4)

    return dot(dot(H, inv(R)),H.T), v

def Static_CRLB_tdoa(ref_x_1, ref_y_1, ref_x_2, ref_y_2, x ,y, R):

    dist_1 = distance(ref_x_1, ref_y_1, x, y)
    dist_2 = distance(ref_x_2, ref_y_2, x, y)
    c = 3.*pow(10,8)
    H = array( [[ (( x - ref_x_1 ) / dist_1 / c) -  (( x - ref_x_2 ) / dist_2 / c)] , [ ((y - ref_y_1  ) / dist_1 / c )-  ((y - ref_y_2  ) / dist_2 / c)]])
    gamma = 10
    v = gamma * pow(( x - ref_x_1 ),2) / pow(dist_1, 4)

    return dot(dot(H, inv(R)),H.T), v

def Static_CRLB_rssi(ref_x, ref_y, x ,y, PLE, R):

    dist = distance(ref_x, ref_y, x, y)
    H = array( [[ -(10*PLE* ( x - ref_x ))/ (pow( dist, 2)*log(10) ) ],[ -(10*PLE* (y - ref_y  )  )/(pow(dist, 2) *log(10)) ] ])
    gamma = pow(10*PLE/ (sqrt(R[0][0])*log(10)) ,2)
    v = gamma * pow(( x - ref_x ),2) / pow(dist, 4)

    return dot(dot(H, inv(R)),H.T), v

def Static_CRLB(ref_x, ref_y, x ,y,dist, PLE, R):

    H = array( [[ -(10*PLE* ( x - ref_x ))/ (pow( dist, 2)*log(10) ) ],[ -(10*PLE* (y - ref_y  )  )/(pow(dist, 2) *log(10)) ] ])
    gamma = pow(10*PLE/ (sqrt(R[0][0])*log(10)) ,2)
    v = gamma * pow(( x - ref_x ),2) / pow(dist, 4)

    return dot(dot(H, inv(R)),H.T), v


def rss_crlb(real_loc, anchors, PLE, shadow, KF_P):

    data_J = []
    num_anchors = len(anchors)

    J = np.zeros((2, 2))

    for i in range(num_anchors):

        dist = distance(real_loc[0], real_loc[1], anchors[i][0], anchors[i][1] )
        #tmp_J, v = Static_CRLB(anchors[i][0], anchors[i][1], real_loc[0], real_loc[1], dist, PLE, array([[shadow**2 + ((sqrt(KF_P) *10*PLE)/log(10)/dist)**2 ]]))
        tmp_J, v = Static_CRLB(anchors[i][0], anchors[i][1], real_loc[0], real_loc[1], dist, PLE, array([[shadow**2]]))

        J += tmp_J

    inv_J = inv(J)
    data_J.append( sqrt(inv_J[0][0]+ inv_J[1][1]))

    return data_J

def rss_crlb2(real_loc, anchors, PLE, shadow):



    data_J = []
    num_anchors = len(anchors)
    if num_anchors <= 1: return array([1.])
    J = np.zeros((2, 2))

    for i in range(num_anchors):
        dist = distance(real_loc[0], real_loc[1], anchors[i][0], anchors[i][1]  )
        #tmp_J, v = Static_CRLB(anchors[i][0], anchors[i][1], real_loc[0], real_loc[1], dist, PLE, array([[shadow**2 + ((sqrt(KF_P) *10*PLE)/log(10)/dist)**2 ]]))
        tmp_J, v = Static_CRLB(anchors[i][0], anchors[i][1], real_loc[0], real_loc[1], dist, PLE, array([[shadow**2]]))

        J += tmp_J
    inv_J = inv(J)
    data_J.append( sqrt(inv_J[0][0]+ inv_J[1][1]))

    return data_J

def rss_crlb3(real_loc, anchors, PLE, shadow):

    data_J = []
    num_anchors = len(anchors)

    J = np.zeros((2, 2))

    for i in range(num_anchors):

        dist = distance(real_loc[0], real_loc[1], anchors[i][0], anchors[i][1]  )
        #tmp_J, v = Static_CRLB(anchors[i][0], anchors[i][1], real_loc[0], real_loc[1], dist, PLE, array([[shadow**2 + ((sqrt(KF_P) *10*PLE)/log(10)/dist)**2 ]]))
        tmp_J, v = Static_CRLB(anchors[i][0], anchors[i][1], real_loc[0], real_loc[1], dist, PLE, array([[shadow**2]]))

        J += tmp_J

    inv_J = inv(J)
    data_J.append( sqrt(inv_J[0][0]+ inv_J[1][1]))

    return data_J


def rss_crlb(real_loc, anchors, PLE, shadow, KF_P):

    data_J = []
    num_anchors = len(anchors)

    J = np.zeros((2, 2))

    for i in range(num_anchors):

        dist = distance(real_loc[0], real_loc[1], anchors[i][0], anchors[i][1]  )
        tmp_J, v = Static_CRLB(anchors[i][0], anchors[i][1], real_loc[0], real_loc[1], dist, PLE, array([[shadow**2 ]]))
        J += tmp_J

    inv_J = inv(J)
    data_J.append( sqrt(inv_J[0][0]+ inv_J[1][1]))

    return data_J


def gps_crlb(GPS_std, GPS_bias):

    data_J = []

    bias_tmp = GPS_bias[0]**2+GPS_bias[1]**2

    J = np.zeros((2, 2))

    J = array([[GPS_std**2,0],[0,GPS_std**2]])

    inv_J = J
    data_J.append( sqrt(inv_J[0][0] + inv_J[1][1]+bias_tmp    ))

    return data_J
    