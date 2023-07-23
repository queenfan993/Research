import numpy as np
from math import *
from numpy import array
from numpy import dot
from numpy.linalg import inv 

EnergyDetectionThreshold = -96

def communicating_LogNormalModel(dist, PLE, PL0, tx_power):

    return tx_power-PL0-10*PLE*np.log10(dist)

def compute_dist(x,y,x_t,y_t):

    return sqrt((x-x_t)**2 + (y-y_t)**2)

def objective_gps(x, gps_std, gps_pos):
    summ = 0.
    summ += dot(dot(array([gps_pos[0][0]-x[0], gps_pos[1][0]-x[1]]), inv(array([ [gps_std**2,0], [0,gps_std**2]]))),array([gps_pos[0][0]-x[0] , gps_pos[1][0]-x[1]]).T)
    return summ 

def objective_gps_magic(x, gps_std, gps_pos):
    summ = 0.
    summ += dot(dot(array([gps_pos[0][0]-x[0]-x[2], gps_pos[1][0]-x[1]-x[2]]), inv(array([ [gps_std**2,0], [0,gps_std**2]]))),array([gps_pos[0][0]-x[0]-x[2] , gps_pos[1][0]-x[1]-x[2]]).T)
    return summ 

def Gaussian( mu, sigma, x):
    
    # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
    return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))

def objective_rssi_gps_proposed_magic_neighbor(x,shadow,measurement_rssi, tx_power,PL0,PLE,anchor,neighbor_uncertainty, gps_std, gps_pos):
    summ = 0.
    summ += dot(dot(array([gps_pos[0][0]-x[0]-x[2], gps_pos[1][0]-x[1]-x[2]]), inv(array([ [gps_std**2,0], [0,gps_std**2]]))),array([gps_pos[0][0]-x[0]-x[2] , gps_pos[1][0]-x[1]-x[2]]).T)
    for i in range(len(measurement_rssi)):
        if measurement_rssi[i] < EnergyDetectionThreshold: continue 

        dist = compute_dist(x[0]-x[2],x[1]-x[2],anchor[i][0][0]-x[3+i],anchor[i][1][0]-x[3+i])
        R = array([[shadow**2]])
        IS = R + ((neighbor_uncertainty[i][0][0]*10*PLE)/log(10)/dist)**2
        tmp_rssi = communicating_LogNormalModel(dist,PLE, PL0, tx_power)
        summ += (pow(IS[0][0], -1)*pow(measurement_rssi[i] -tmp_rssi , 2))
        
    return summ 

def objective_rssi_gps_proposed_magic_neighbor2(x,shadow,measurement_rssi, tx_power,PL0,PLE,anchor,neighbor_uncertainty, gps_std, gps_pos):
    summ = 0.
    summ += dot(dot(array([gps_pos[0][0]-x[0]-x[2], gps_pos[1][0]-x[1]-x[2]]), inv(array([ [gps_std**2,0], [0,gps_std**2]]))),array([gps_pos[0][0]-x[0]-x[2] , gps_pos[1][0]-x[1]-x[2]]).T)
    for i in range(len(measurement_rssi)):
        if measurement_rssi[i] < EnergyDetectionThreshold: continue 

        dist = compute_dist(x[0]-x[2],x[1]-x[2],anchor[i][0][0]-x[3+i],anchor[i][1][0]-x[3+i])
        R = array([[shadow**2]])
        IS = R 
        tmp_rssi = communicating_LogNormalModel(dist,PLE, PL0, tx_power)
        summ += (pow(IS[0][0], -1)*pow(measurement_rssi[i] -tmp_rssi , 2))
        
    return summ 

def objective_rssi_gps_proposed_magic(x,shadow,measurement_rssi, tx_power,PL0,PLE,anchor,neighbor_uncertainty, gps_std, gps_pos):
    summ = 0.
    summ += dot(dot(array([gps_pos[0][0]-x[0]-x[2], gps_pos[1][0]-x[1]-x[2]]), inv(array([ [gps_std**2,0], [0,gps_std**2]]))),array([gps_pos[0][0]-x[0]-x[2] , gps_pos[1][0]-x[1]-x[2]]).T)
    for i in range(len(measurement_rssi)):
        if measurement_rssi[i] < EnergyDetectionThreshold: continue 

        dist = compute_dist(x[0]-x[2],x[1]-x[2],anchor[i][0][0],anchor[i][1][0])
        R = array([[shadow**2]])
        IS = R + ((neighbor_uncertainty[i][0][0]*10*PLE)/log(10)/dist)**2
        tmp_rssi = communicating_LogNormalModel(dist,PLE, PL0, tx_power)
        summ += (pow(IS[0][0], -1)*pow(measurement_rssi[i] - tmp_rssi , 2))
        
    return summ 


def objective_rssi_gps_proposed(x,shadow,measurement_rssi, tx_power,PL0,PLE,anchor,neighbor_uncertainty, gps_std, gps_pos):
    summ = 0.
    summ += dot(dot(array([gps_pos[0][0]-x[0], gps_pos[1][0]-x[1]]), inv(array([ [gps_std**2,0], [0,gps_std**2]]))),array([gps_pos[0][0]-x[0] , gps_pos[1][0]-x[1]]).T)
    for i in range(len(measurement_rssi)):
    	if measurement_rssi[i] < EnergyDetectionThreshold: continue 

        dist = compute_dist(x[0],x[1],anchor[i][0][0],anchor[i][1][0])
        R = array([[shadow**2]])
        IS = R + ((neighbor_uncertainty[i][0][0]*10*PLE)/log(10)/dist)**2
        tmp_rssi = communicating_LogNormalModel(dist,PLE, PL0, tx_power)
        summ += (pow(IS[0][0], -1)*pow(measurement_rssi[i] -tmp_rssi , 2))
        
    return summ 

def objective_rssi_gps(x,shadow,measurement_rssi, tx_power,PL0,PLE,anchor,neighbor_uncertainty, gps_std, gps_pos):
    summ = 0.
    summ += dot(dot(array([gps_pos[0][0]-x[0], gps_pos[1][0]-x[1]]), inv(array([ [gps_std**2,0], [0,gps_std**2]]))),array([gps_pos[0][0]-x[0] , gps_pos[1][0]-x[1]]).T)
    for i in range(len(measurement_rssi)):
    	if measurement_rssi[i] < EnergyDetectionThreshold: continue 

        dist = compute_dist(x[0],x[1],anchor[i][0][0],anchor[i][1][0])
        R = array([[shadow**2]])
        IS = R 
        tmp_rssi = communicating_LogNormalModel(dist,PLE, PL0, tx_power)
        summ += (pow(IS[0][0], -1)*pow(measurement_rssi[i] -tmp_rssi , 2))
        
    return summ 

def objective_rssi_proposed(x,shadow,measurement_rssi,tx_power,PL0,PLE,anchor,neighbor_uncertainty):
    summ = 0.
    for i in range(len(measurement_rssi)):

    	if measurement_rssi[i] < EnergyDetectionThreshold: continue 
        dist = compute_dist(x[0],x[1],anchor[i][0][0],anchor[i][1][0])
        R = array([[shadow**2]])
        IS = R + ((neighbor_uncertainty[i][0][0]*10*PLE)/log(10)/dist)**2
        tmp_rssi = communicating_LogNormalModel(dist,PLE, PL0, tx_power)
        summ += (pow(IS[0][0], -1)*pow(measurement_rssi[i] -tmp_rssi , 2))
        
    return summ 


def objective_rssi_MC(x,shadow, measurement_rssi,tx_power,PL0,PLE,anchor, num_particles, neighbor_uncertainty):
    summ = 0.

    print "x:", x

    for i in range(len(measurement_rssi)):

    	if measurement_rssi[i] < EnergyDetectionThreshold: continue 

    	sample_GPS = np.random.multivariate_normal((anchor[i][0][0], anchor[i][1][0]), neighbor_uncertainty[i], num_particles).T
        total = 0.
        for n in range(num_particles):
            p_x = sample_GPS[0][n]
            p_y = sample_GPS[1][n]

            dist = compute_dist(x[0], x[1], p_x, p_y)
            tmp_rssi = communicating_LogNormalModel(dist, PLE, PL0, tx_power)
            #total += (pow(shadow, -2)*pow(measurement_rssi[i] - tmp_rssi, 2))
            #print i, measurement_rssi[i], tmp_rssi
            #print Gaussian( measurement_rssi[i], shadow, tmp_rssi)
            #print sqrt(2.0 * pi * (shadow ** 2))
            #print Gaussian( measurement_rssi[i], shadow, tmp_rssi) * sqrt(2.0 * pi * (shadow ** 2))
            #print pow(Gaussian( measurement_rssi[i], shadow, tmp_rssi) * sqrt(2.0 * pi * (shadow ** 2)),2)
            total += pow(Gaussian( measurement_rssi[i], shadow, tmp_rssi) * sqrt(2.0 * pi * (shadow ** 2)),2)
            #print Gaussian( measurement_rssi[i], shadow, tmp_rssi) * sqrt(2.0 * pi * (shadow ** 2))
            #print pow(Gaussian( measurement_rssi[i], shadow, tmp_rssi) * sqrt(2.0 * pi * (shadow ** 2)),2)
            #total += Gaussian( measurement_rssi[i], shadow, tmp_rssi)
        
        total = log(total)

        total /= num_particles
        total = total
        print "value2:",total
        #total = total * sqrt(2.0 * pi * (shadow ** 2))
        #total = log(total)
        
        #total = -total
        summ += total
    print "summ:", summ

    return summ 

def objective_rssi(x,shadow,measurement_rssi,tx_power,PL0,PLE,anchor):
    summ = 0.
    for i in range(len(measurement_rssi)):
    	if measurement_rssi[i] < EnergyDetectionThreshold: continue 
        dist = compute_dist(x[0], x[1], anchor[i][0][0], anchor[i][1][0])
        tmp_rssi = communicating_LogNormalModel(dist, PLE, PL0, tx_power)
        summ += (pow(shadow, -2)*pow(measurement_rssi[i] -tmp_rssi, 2))
        
    return summ 


