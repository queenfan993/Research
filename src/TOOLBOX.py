import random
import numpy as np

from numpy import dot, sum
from numpy.linalg import inv 
from numpy import *
from math import *
import random

def distance(x,y,x_t,y_t):

    return sqrt((x-x_t)**2 + (y-y_t)**2)

def distance2(x,y,x_t,y_t):

    return (x-x_t)**2 + (y-y_t)**2



def compute_entropy(weight):
    sum_entropy = 0.
    for i in range(len(weight)):
        sum_entropy += (- weight[i] * log(weight[i]))
    return sum_entropy/len(weight)

def Gaussian( mu, sigma, x):
    
    # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
    return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))

def multivariate_gaussian(pos, mu, Sigma):

    n = mu.shape[0]
    Sigma_det = np.linalg.det(Sigma)

    Sigma_inv = np.linalg.inv(Sigma)
    N = np.sqrt((2*np.pi)**n * Sigma_det)
    # This einsum call calculates (x-mu)T.Sigma-1.(x-mu) in a vectorized
    # way across all the input variables.
    fac = np.einsum('...k,kl,...l->...', pos-mu, Sigma_inv, pos-mu)

    return np.exp(-fac / 2) / N


def odometry(x_t, y_t, x_t_1, y_t_1, motion_noise, direction_noise):

    true_direction = math.atan2(y_t - y_t_1, x_t - x_t_1) 
    true_forward = distance(x_t, y_t, x_t_1, y_t_1)
    if true_direction == 0 and true_forward ==0:
        return 0 , 0

    return true_forward+random.gauss(0.0, motion_noise) , true_direction+random.gauss(0.0, direction_noise)%(2*pi)
def genBias(GPS_bias, n_vehicles):

    tmp = []
    for i in range(n_vehicles):
        tmp.append([random.gauss(0, GPS_bias[0]), random.gauss(0, GPS_bias[0])])

    return tmp
