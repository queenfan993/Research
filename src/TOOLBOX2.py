import random
import numpy as np

from numpy import dot, sum
from numpy.linalg import inv 
from numpy import *
from math import *
import random

from GPS_simulator import *
from CHANNEL import *
from nodes import *
from EVALUATION import *
from CRLB_TOOL import *
from TOOLBOX import *
from numpy import linalg as LA
from matplotlib import patches

def cal_all_error_rss(anchors, filter_index, target_index):
    total = 0.
    for i in range(len(anchors)):
        if i == target_index: 
            continue
            anchor_x, anchor_y, anchor_o = anchors[target_index].getposition()
            error = distance(anchor_x,anchor_y,anchors[target_index].get_filter_rss_X(filter_index)[0][0],anchors[target_index].get_filter_rss_X(filter_index)[1][0])
            total += error
        else:
            if i not in anchors[target_index].candidate: continue
            anchor_x, anchor_y, anchor_o = anchors[i].getposition()
            tm_x = anchors[target_index].neighbor_KF[filter_index][i].get_X()[0][0]
            tm_y = anchors[target_index].neighbor_KF[filter_index][i].get_X()[1][0]
            error = distance(anchor_x,anchor_y,tm_x,tm_y)
            total += error

    if len(anchors[target_index].candidate)>0:
        total /= len(anchors[target_index].candidate)

    return total

def cal_all_error_toa(anchors, filter_index, target_index):
    total = 0.
    for i in range(len(anchors)):
        if i == target_index: 
            continue
            anchor_x, anchor_y, anchor_o = anchors[target_index].getposition()
            error = distance(anchor_x,anchor_y,anchors[target_index].get_filter_toa_X(filter_index)[0][0],anchors[target_index].get_filter_toa_X(filter_index)[1][0])
            total += error
        else:
            if i not in anchors[target_index].candidate: continue
            anchor_x, anchor_y, anchor_o = anchors[i].getposition()
            tm_x = anchors[target_index].neighbor_KF[filter_index][i].get_X()[0][0]
            tm_y = anchors[target_index].neighbor_KF[filter_index][i].get_X()[1][0]
            error = distance(anchor_x,anchor_y,tm_x,tm_y)
            total += error

    if len(anchors[target_index].candidate)>0:
        total /= len(anchors[target_index].candidate)

    return total 
def cal_all_error_tdoa(anchors, filter_index, target_index):
    total = 0.
    for i in range(len(anchors)):
        if i == target_index: 
            continue
            anchor_x, anchor_y, anchor_o = anchors[target_index].getposition()
            error = distance(anchor_x,anchor_y,anchors[target_index].get_filter_tdoa_X(filter_index)[0][0],anchors[target_index].get_filter_tdoa_X(filter_index)[1][0])
            total += error
        else:
            if i not in anchors[target_index].candidate: continue
            anchor_x, anchor_y, anchor_o = anchors[i].getposition()
            tm_x = anchors[target_index].neighbor_KF[filter_index][i].get_X()[0][0]
            tm_y = anchors[target_index].neighbor_KF[filter_index][i].get_X()[1][0]
            error = distance(anchor_x,anchor_y,tm_x,tm_y)
            total += error

    if len(anchors[target_index].candidate)>0:
        total /= len(anchors[target_index].candidate)

    return total           
def cal_all_error_rss_toa(anchors, filter_index, target_index):
    total = 0.
    for i in range(len(anchors)):
        if i == target_index: 
            continue
            anchor_x, anchor_y, anchor_o = anchors[target_index].getposition()
            error = distance(anchor_x,anchor_y,anchors[target_index].get_filter_rss_toa_X(filter_index)[0][0],anchors[target_index].get_filter_rss_toa_X(filter_index)[1][0])
            total += error
        else:
            if i not in anchors[target_index].candidate: continue
            anchor_x, anchor_y, anchor_o = anchors[i].getposition()
            tm_x = anchors[target_index].neighbor_KF[filter_index][i].get_X()[0][0]
            tm_y = anchors[target_index].neighbor_KF[filter_index][i].get_X()[1][0]
            error = distance(anchor_x,anchor_y,tm_x,tm_y)
            total += error

    if len(anchors[target_index].candidate)>0:
        total /= len(anchors[target_index].candidate)

    return total 

def cal_all_error_rss_toa_tdoa(anchors, filter_index, target_index):
    total = 0.
    for i in range(len(anchors)):
        if i == target_index: 
            continue
            anchor_x, anchor_y, anchor_o = anchors[target_index].getposition()
            error = distance(anchor_x,anchor_y,anchors[target_index].get_filter_rss_toa_tdoa_X(filter_index)[0][0],anchors[target_index].get_filter_rss_toa_tdoa_X(filter_index)[1][0])
            total += error
        else:
            if i not in anchors[target_index].candidate: continue
            anchor_x, anchor_y, anchor_o = anchors[i].getposition()
            tm_x = anchors[target_index].neighbor_KF[filter_index][i].get_X()[0][0]
            tm_y = anchors[target_index].neighbor_KF[filter_index][i].get_X()[1][0]
            error = distance(anchor_x,anchor_y,tm_x,tm_y)
            total += error

    if len(anchors[target_index].candidate)>0:
        total /= len(anchors[target_index].candidate)

    return total     


#================================================================================
def cal_all_smart_error_rss(anchors, filter_index, smart_list):
    total = 0.

    for i in range(len(anchors)):
        if i not in smart_list: continue
        anchor_x, anchor_y, anchor_o = anchors[i].getposition()
        error = distance(anchor_x, anchor_y, anchors[i].get_filter_rss_X(filter_index)[0][0], anchors[i].get_filter_rss_X(filter_index)[1][0])
        total += error

    total /= len(smart_list)

    return total

def cal_all_smart_error_toa(anchors, filter_index, smart_list):
    total = 0.

    for i in range(len(anchors)):
        if i not in smart_list: continue
        anchor_x, anchor_y, anchor_o = anchors[i].getposition()
        error = distance(anchor_x, anchor_y, anchors[i].get_filter_toa_X(filter_index)[0][0], anchors[i].get_filter_toa_X(filter_index)[1][0])
        total += error

    total /= len(smart_list)

    return total
def cal_all_smart_error_tdoa(anchors, filter_index, smart_list):
    total = 0.

    for i in range(len(anchors)):
        if i not in smart_list: continue
        anchor_x, anchor_y, anchor_o = anchors[i].getposition()
        error = distance(anchor_x, anchor_y, anchors[i].get_filter_tdoa_X(filter_index)[0][0], anchors[i].get_filter_tdoa_X(filter_index)[1][0])
        total += error

    total /= len(smart_list)

    return total 

def cal_all_smart_error_rss_toa(anchors, filter_index, smart_list):
    total = 0.

    for i in range(len(anchors)):
        if i not in smart_list: continue
        anchor_x, anchor_y, anchor_o = anchors[i].getposition()
        error = distance(anchor_x, anchor_y, anchors[i].get_filter_rss_toa_X(filter_index)[0][0], anchors[i].get_filter_rss_toa_X(filter_index)[1][0])
        total += error

    total /= len(smart_list)

    return total

def cal_all_smart_error_rss_toa_tdoa(anchors, filter_index, smart_list):
    total = 0.

    for i in range(len(anchors)):
        if i not in smart_list: continue
        anchor_x, anchor_y, anchor_o = anchors[i].getposition()
        error = distance(anchor_x, anchor_y, anchors[i].get_filter_rss_toa_tdoa_X(filter_index)[0][0], anchors[i].get_filter_rss_toa_tdoa_X(filter_index)[1][0])
        total += error

    total /= len(smart_list)

    return total    

#============================
def plot_dynamic2(an_loc, smart, anchors, gps_measurements, color_list, ax):

    #plt.subplot(1, 1, 1)
    #plt.cla()

    plt.xlim(0, 50)
    plt.ylim(0, 50)

    c = ['g','k','b','m','y']
    count = 0
    num_anchors = len(an_loc)
    an_loc = np.array(an_loc)

    for i in range(num_anchors):
        if i in smart:
            ax.scatter(anchors[i].filter_list[0].X[0][0], anchors[i].filter_list[0].X[1][0] ,marker='x',alpha=0.5, c=color_list[i]) 
            #plt.scatter(anchors[i].filter_list[3].X[0][0], anchors[i].filter_list[3].X[1][0] ,marker='^',alpha=0.5, c=color_list[i]) 

            #plt.scatter(gps_measurements[i][0][0], gps_measurements[i][1][0],alpha=0.5, c=color_list[i]) 

    #gps_measurements = np.array(gps_measurements)
    an_loc = np.array(an_loc)

    for i in range(num_anchors):
        ax.scatter(an_loc[i, 0], an_loc[i, 1], c=color_list[i]) 
        '''
        if i in smart:
            plt.scatter(an_loc[i, 0], an_loc[i, 1], c=c[count]) 
            count += 1
        '''

    #plt.plot(gps_measurements[:, 0], gps_measurements[:, 1], c='g', alpha=0.5)
    #plt.plot(kf_loc[:, 0], kf_loc[:, 1], c='m', alpha=0.5)
    plt.pause(0.01)


def plot_dynamic(an_loc,p_loc):
    plt.subplot(1, 1, 1)
    plt.cla()

    plt.xlim(-200, 200)
    plt.ylim(-200, 200)

    #gps_measurements = np.array(gps_measurements)
    an_loc = np.array(an_loc)
    p_loc = np.array(p_loc)

    plt.scatter(p_loc[:, 0], p_loc[:, 1], c='b') 
    plt.scatter(an_loc[:, 0], an_loc[:, 1], c='r') 

    #plt.plot(gps_measurements[:, 0], gps_measurements[:, 1], c='g', alpha=0.5)
    #plt.plot(kf_loc[:, 0], kf_loc[:, 1], c='m', alpha=0.5)

    plt.pause(0.01)


def genAnchors(num_nodes, world_size, channel_obj, TOA_obj,TDOA_obj, forward_noise, turn_noise, num_particles, smart_list, GPS_obj_list, ini_direction, velocity_noise_percent):

    landmarks = []
    for i in range(num_nodes):
        x = np.random.uniform(0, world_size, 2).tolist()
        landmarks.append(Vehicle(i, num_nodes, channel_obj, TOA_obj, TDOA_obj, GPS_obj_list[i], num_particles, velocity_noise_percent))
        landmarks[i].set(x[0], x[1], ini_direction)
        landmarks[i].set_noise(forward_noise, turn_noise)
        landmarks[i].set_smart(smart_list)

    return landmarks

def compute_Mean_variance(error):
    mu=np.mean(error, axis=0)
    std=np.std(error, axis=0, ddof = 1)
    return mu, std

def gen_particles_according_location(pos, num_particles, GPS_obj):

    particle_set = []

    for i in range(num_particles):
        particle_set.append(GPS_obj.genGPS_coodinate(pos[0][0], pos[1][0]))
    return particle_set

def compute_DistAllNeighbors(target_true, target_fixed):

    error_list = []
    for i in range(len(target_true)):
        error_list.append(distance(target_true[i][0], target_true[i][1], target_fixed[i][0][0], target_fixed[i][1][0]))

    return error_list

def genBar(data, index, bar_width, color, labels):
    context = []
    for i in range(len(data)):
        tmp_mean = np.mean(data[i])
        tmp_mean = sqrt(tmp_mean)

        tmp_x = index+i*bar_width
        context.append(
            plt.bar(tmp_x,
           tmp_mean, 
           bar_width,
           color=color[i],
           alpha=.5,
           label=labels[i]) 
            )
        stdqq = [sqrt(k) for k in np.mean(data[i] ,axis=0)]

        plt.errorbar(tmp_x,tmp_mean, yerr=np.std(stdqq,ddof = 1),fmt='o',ecolor=color[i],color=color[i],elinewidth=2,capsize=4)
    return context

def createLabels(data):
    for item in data:
        height = item.get_height()

        plt.text(
            item.get_x()+item.get_width()/2., 
            height*0.1, 
            '%.2f' % float(height),
            ha = "center",
            va = "bottom",
        )

def genBias(GPS_bias, n_vehicles):

    tmp = []
    for i in range(n_vehicles):
        g = abs(random.gauss(0,GPS_bias[0]))
        #g = -GPS_bias[0]
        tmp.append([g, g])

    return tmp

def getAllLOC(anchors, num_anchors):

    loc = []

    for i in range(num_anchors):
        anchor_x, anchor_y, anchor_o = anchors[i].getposition()
        loc.append([anchor_x, anchor_y])

    return loc

def getParticleLoc(num_particles, pf_obj):

    loc = []
    for i in range(num_particles):
        x = pf_obj.getP_loc(i)[0][0]
        y = pf_obj.getP_loc(i)[1][0]

        loc.append([x,y])
        #print pf_obj.particle_set[i]

    return loc

def genVelocity(num_anchors, mid_v):
    v = []
    for i in range(num_anchors):
        v.append( abs(random.gauss(mid_v, 0.2*mid_v)))
        #v.append(mid_v)
    return v

def genTurnVelocity(num_anchors, mid_v):
    v = []
    for i in range(num_anchors):
        #v.append( abs(random.gauss(mid_v, 0.2*mid_v)))
        v.append(mid_v)
    return v

def choose_smart_vehicle(num_anchors, prob):

    smart = []
    for i in range(num_anchors):
        if i == 0:
            smart.append(i)
            continue
        if random.uniform(0, 1) <= prob:
            smart.append(i)
    return smart

def genLocation(num_anchors):
    loc = []
    for i in range(num_anchors):
        x = np.random.uniform(0, world_size, 2).tolist()
        loc.append(x)
    return loc



def plot_error_cllipse_pretty(cov_matrix, ax, real_x, real_y, color_list):
    w, v = LA.eig(cov_matrix)

    s = -2*log(1-0.95)

    ind = np.argmax(w)
    angle = (np.arctan2(v[:,ind][1],v[:,ind][0]))*180/pi

    if w[0] > w[1]:
        width = 2*sqrt(s*w[0])
        height = 2*sqrt(s*w[1])
    else:
        width = 2*sqrt(s*w[1])
        height = 2*sqrt(s*w[0])

    theta = np.deg2rad(np.arange(0.0, 360.0, 1.0))
    x = 0.5 * width * np.cos(theta)
    y = 0.5 * height * np.sin(theta)

    rtheta = np.radians(angle)
    R = np.array([
        [np.cos(rtheta), -np.sin(rtheta)],
        [np.sin(rtheta),  np.cos(rtheta)],
        ])
    
    x, y = np.dot(R, np.array([x, y]))
    x += real_x
    y += real_y

    ax.fill(x, y, alpha=0.5, facecolor=color_list,
        edgecolor=color_list, linewidth=1, zorder=2)
    e1 = patches.Ellipse((real_x, real_y),  width, height,
                     angle=angle, linewidth=2, fill=False, zorder=1)

    ax.add_patch(e1)

def plot_error_cllipse(cov_matrix, ax, real_x, real_y):
    w, v = LA.eig(cov_matrix)
    s = -2*log(1-0.95)

    ind = np.argmax(w)
    angle = (np.arctan2(v[:,ind][1],v[:,ind][0]))*180/pi

    if w[0] > w[1]:
        width = 2*sqrt(s*w[0])
        height = 2*sqrt(s*w[1])
    else:
        width = 2*sqrt(s*w[1])
        height = 2*sqrt(s*w[0])

    e1 = patches.Ellipse((real_x, real_y), width, height,
                     angle=angle, linewidth=2, fill=False, zorder=1)

    ax.add_patch(e1)
    plt.pause(0.01)



def plot_dynamic_ellipse(an_loc, smart, anchors, gps_measurements, color_list, ax, filter_index,filter_index2):

    #plt.subplot(1, 1, 1)
    an_loc = np.array(an_loc)
    plt.cla()

    plt.xlim(an_loc[0, 0]-25, an_loc[0, 0]+25)
    plt.ylim(an_loc[0, 1]-25, an_loc[0, 1]+25)

    

    c = ['g','k','b','m','y']
    count = 0
    num_anchors = len(an_loc)
    an_loc = np.array(an_loc)

    for i in range(num_anchors):

        if i in smart:
            plot_error_cllipse_pretty(anchors[0].get_filter_P(filter_index), ax,anchors[0].get_filter_X(filter_index)[0][0],anchors[0].get_filter_X(filter_index)[1][0],color_list[i])
            plot_error_cllipse_pretty(anchors[0].true_fisher_info_matrix_rss_gps, ax,anchors[0].x,anchors[0].y,'r')

            ax.scatter(anchors[i].filter_list[filter_index].X[0][0], anchors[i].filter_list[filter_index].X[1][0] ,marker='x',alpha=1.0, c=color_list[i]) 
        #self.neighbor_KF[filter_index][i].X
        else:
            plot_error_cllipse_pretty(anchors[0].neighbor_KF[0][i].P, ax, anchors[0].neighbor_KF[0][i].X[0][0] ,anchors[0].neighbor_KF[0][i].X[1][0],color_list[i])

    

    for i in range(num_anchors):
        ax.scatter(an_loc[i, 0], an_loc[i, 1], c=color_list[i]) 

    plt.pause(0.01)







