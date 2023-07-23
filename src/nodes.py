import random
import numpy as np

from numpy import dot, sum
from numpy.linalg import inv 
from numpy import *
from math import *
import random

from KF_TOOL import *
from PF_TOOL import *
from TOOLBOX import *
from CRLB_TOOL import *


class Vehicle:

    def __init__(self, ID, num_nodes, channel_obj, TOA_obj, TDOA_obj, GPS_obj, num_particles, velocity_noise_percent):

        self.forward_noise = 0.0
        self.turn_noise    = 0.0
 

        self.ID = ID
        self.num_nodes = num_nodes

        self.GPS_obj = GPS_obj
        self.GPS_std = GPS_obj.getGPS_std()
        self.GPS_variance = array([[self.GPS_std**2,0],[0,self.GPS_std**2]])
        #modified why add gps std initial
        self.true_fisher_info_matrix_rss_gps = array([[self.GPS_std**2,0],[0,self.GPS_std**2]])
        self.true_fisher_info_matrix_toa_gps = array([[self.GPS_std**2,0],[0,self.GPS_std**2]])
        self.true_fisher_info_matrix_tdoa_gps = array([[self.GPS_std**2,0],[0,self.GPS_std**2]])
        self.true_fisher_info_matrix_rss_toa_gps = array([[self.GPS_std**2,0],[0,self.GPS_std**2]])
        self.true_fisher_info_matrix_rss_tdoa_gps = array([[self.GPS_std**2,0],[0,self.GPS_std**2]])
        self.true_fisher_info_matrix_toa_tdoa_gps = array([[self.GPS_std**2,0],[0,self.GPS_std**2]])
        self.true_fisher_info_matrix_rss_toa_tdoa_gps = array([[self.GPS_std**2,0],[0,self.GPS_std**2]])
        
        
        self.true_fisher_info_matrix_rss = array([[0,0],[0,0]])
        self.true_fisher_info_matrix_toa = array([[0,0],[0,0]])
        self.true_fisher_info_matrix_tdoa = array([[0,0],[0,0]])
        self.true_fisher_info_matrix_rss_toa = array([[0,0],[0,0]])
        self.true_fisher_info_matrix_rss_tdoa = array([[0,0],[0,0]])
        self.true_fisher_info_matrix_toa_tdoa = array([[0,0],[0,0]]) 
        self.true_fisher_info_matrix_rss_toa_tdoa = array([[0,0],[0,0]])

        self.true_fisher_info_matrix_gps = array([[self.GPS_std**2,0],[0,self.GPS_std**2]])

        self.channel = channel_obj
        self.tx_power = channel_obj.Txpower
        self.PLE = channel_obj.PLE
        self.PL0 = channel_obj.PL0
        self.shadowing  = channel_obj.shadowing
        self.EnergyDetectionThreshold = channel_obj.get_EDT()
        
        self.TOA = TOA_obj
        self.c = TOA_obj.c
        self.t0 = TOA_obj.t0
        self.syn_error = TOA_obj.syn_error

        self.TDOA = TDOA_obj
       
        self.neighbors = []
        self.filter_list_rss = []
        self.filter_list_toa = [] 
        self.filter_list_tdoa = []

        self.filter_list_toa_tdoa = []
        self.filter_list_rss_toa = []
        self.filter_list_rss_tdoa = []
        self.filter_list_rss_toa_tdoa = []  

        self.neighbor_KF = []

        self.num_particles = num_particles

        self.now_neighbor_velocity = [0] * self.num_nodes
        self.neighbor_whether_meet_state = [0] * self.num_nodes

        self.gps_miss = [False] * self.num_nodes

        self.interval_odometry = 0.1
        self.velocity_noise_percent = velocity_noise_percent
        self.smart_list = []

        self.covariance_percent = 1.0

    def set_smart(self, smart_list):
        self.smart_list = smart_list

    def setAllinfor(self, all_measurement_gps, observed_data, perfect_neighbor):

        self.all_measurement_gps = all_measurement_gps
        self.observed_data = observed_data
        self.perfect_neighbor = perfect_neighbor

    def compute_std_given_Accuracy_Quentile(self, accuracy, quentile):

        return accuracy/(sqrt(-2*log(1-0.99)))

    def communicating_LogNormalModel(self, nodes):
        self.all_measurement_rssi = []

        for i in range(self.num_nodes):
            if i == self.ID : 
                self.all_measurement_rssi.append(0)
                continue
            anchor_x, anchor_y, anchor_ori  = nodes[i].getposition()
            dist = distance(self.x, self.y, anchor_x, anchor_y)
            self.all_measurement_rssi.append(self.channel.genRSS_coodinate(anchor_x, anchor_y, self.x, self.y) )
    
    def communicating_TOA(self, nodes):
        self.all_measurement_TOA = []
        
        for i in range(self.num_nodes):
            if i == self.ID : 
                self.all_measurement_TOA.append(0)
                continue
            anchor_x, anchor_y, anchor_ori  = nodes[i].getposition()
            dist = distance(self.x, self.y, anchor_x, anchor_y)
            self.all_measurement_TOA.append(self.TOA.genTOA_coodinate(anchor_x, anchor_y, self.x, self.y) )    

    def communicating_TDOA(self, nodes):
        self.all_measurement_TDOA = []
        
        for i in range(self.num_nodes):
            for j in range(i):
                if (i == self.ID) or (j == self.ID) : 
                    self.all_measurement_TDOA.append(0)
                    continue
                anchor_x_1, anchor_y_1, anchor_ori_1 = nodes[i].getposition()
                anchor_x_2, anchor_y_2, anchor_ori_2 = nodes[j].getposition()

                self.all_measurement_TDOA.append(self.TDOA.genTDOA_coodinate(anchor_x_1, anchor_y_1, anchor_x_2, anchor_y_2, self.x, self.y) )  
        

    def genFilteringObj(self, mode, x,y):
        
        qaq = ["KF",

                "KF_EKF_GPS_ori", "KF_EKF_table_ori","KF_EKF_GPS_NU_GSTD",  \

                "KF_EKF_table_NU_CT","KF_EKF_table_NU_GSTD",

                "KF_EKF_table_ECRLB_CT_estimate","KF_EKF_table_ECRLB_GSTD_estimate", "KF_EKF_table_ECRLB_CT_TF","KF_EKF_table_ECRLB_GSTD_TF",

                "KF_EKF_ture_ECRLB_CT_TF","KF_EKF_ture_ECRLB_GSTD_TF", "KF_EKF_ture_ECRLB_CT_estimate","KF_EKF_ture_ECRLB_GSTD_estimate","KF_EKF_true_ori",

                "KF_EKF_true_NU_CT", "KF_EKF_true_NU_GSTD",


                 "Fisher", "Fisher_measurement", "Fisher_gps"\
                ]
        

        self.neighbor_KF.append([])
        if mode == "KF" or mode == "PF_FILTER" or mode == "KF2" :
            self.filter_list_rss.append(KalmanFilter( mode,x,y, self.forward_noise, self.GPS_std))
            self.filter_list_toa.append(KalmanFilter( mode,x,y, self.forward_noise, self.GPS_std))
            self.filter_list_tdoa.append(KalmanFilter( mode,x,y, self.forward_noise, self.GPS_std))

            self.filter_list_toa_tdoa.append(KalmanFilter( mode,x,y, self.forward_noise, self.GPS_std))
            self.filter_list_rss_toa.append(KalmanFilter( mode,x,y, self.forward_noise, self.GPS_std))
            self.filter_list_rss_tdoa.append(KalmanFilter( mode,x,y, self.forward_noise, self.GPS_std))
            self.filter_list_rss_toa_tdoa.append(KalmanFilter( mode,x,y, self.forward_noise, self.GPS_std))

        if mode == "EKF_ori"or mode == "EKF_perfect"or mode == "EKF_perfect_proposed" or mode == "EKF_proposed" or mode == "EKF_para"or mode == "EKF_para_proposed":
            self.filter_list_rss.append(ExtendedKalmanFilter( mode,x,y, self.forward_noise, self.shadowing))
            self.filter_list_toa.append(ExtendedKalmanFilter( mode,x,y, self.forward_noise, self.shadowing))
            self.filter_list_tdoa.append(ExtendedKalmanFilter( mode,x,y, self.forward_noise, self.shadowing))

            self.filter_list_toa_tdoa.append(ExtendedKalmanFilter( mode,x,y, self.forward_noise, self.shadowing))
            self.filter_list_rss_toa.append(ExtendedKalmanFilter( mode,x,y, self.forward_noise, self.shadowing))
            self.filter_list_rss_tdoa.append(ExtendedKalmanFilter( mode,x,y, self.forward_noise, self.shadowing))
            self.filter_list_rss_toa_tdoa.append(ExtendedKalmanFilter( mode,x,y, self.forward_noise, self.shadowing))
                
        if mode == "KF_EKF_ori"or mode =="KF_EKF_proposed2"or mode == "KF_EKF_CRLB3"or mode == "KF_EKF_CRLB4" \
            or mode == "KF_EKF_CRLB5" or mode == "KF_EKF_perfect_proposed_smart"  or mode == "KF_EKF_ECRLB" \
            or mode == "KF_EKF_ECRLB2"or mode == "KF_EKF_ECRLB3"or mode == "KF_EKF_ECRLB4"or mode == "KF_EKF_ECRLB4_para"\
            or mode == "KF_EKF_CRLB_mod"or mode == "KF_EKF_ECRLB_mod"or mode == "KF_EKF_CRLB2"or mode == "KF_EKF_CRLB"\
            or mode == "KF_EKF_CRLB_mod2"or mode == "KF_EKF_CRLB_mod3"or mode == "KF_EKF_perfect_proposed2"or mode == "KF_EKF_perfect_proposed3"\
            or mode == "KF_EKF_perfect_proposed_gps"or mode == "KF_EKF_test"or mode == "KF_EKF_GPS"\
            or mode == "KF_EKF_proposed" or mode == "KF_EKF_GPS_std" or mode == "KF_EKF_perfect" \
            or mode == "KF_EKF_proposed2" or mode == "KF_EKF_perfect_proposed"or mode == "KF_EKF_no_table"\
            or mode == "KF_EKF_ori_para"or mode == "KF_EKF_para_proposed"or mode == "KF_EKF_proposed_smart"\
            or mode == "KF_EKF_ECRLB_GPS"or mode == "KF_EKF_proposed_GPS"or mode == "KF_EKF_proposed_smart"\
            or mode == "KF_EKF_proposed_noKF"or mode == "KF_EKF_ECRLB_noKF" or mode == "KF_EKF_ECRLB_mod2"or mode == "KF_EKF_ECRLB_mod3":
            
            self.filter_list_rss.append(KF_ExtendedKalmanFilter( mode,x,y, self.forward_noise, self.GPS_std))
            self.filter_list_toa.append(KF_ExtendedKalmanFilter( mode,x,y, self.forward_noise, self.GPS_std))
            self.filter_list_tdoa.append(KF_ExtendedKalmanFilter( mode,x,y, self.forward_noise, self.GPS_std))

            self.filter_list_toa_tdoa.append(KF_ExtendedKalmanFilter( mode,x,y, self.forward_noise, self.GPS_std))
            self.filter_list_rss_toa.append(KF_ExtendedKalmanFilter( mode,x,y, self.forward_noise, self.GPS_std))
            self.filter_list_rss_tdoa.append(KF_ExtendedKalmanFilter( mode,x,y, self.forward_noise, self.GPS_std))
            self.filter_list_rss_toa_tdoa.append(KF_ExtendedKalmanFilter( mode,x,y, self.forward_noise, self.GPS_std))

        if mode == "KF_EKF_ori" or mode == "KF_EKF_ori_20"or mode == "KF_EKF_ori_40"or mode == "KF_EKF_ori_80"\
            or mode == "KF_EKF_ori_05"or mode == "KF_EKF_ori_160"or mode == "KF_EKF_ori_320"or mode == "KF_EKF_ori_640":
            self.filter_list_rss.append(KF_ExtendedKalmanFilter( mode,x,y, self.forward_noise, self.GPS_std))
            self.filter_list_toa.append(KF_ExtendedKalmanFilter( mode,x,y, self.forward_noise, self.GPS_std))
            self.filter_list_tdoa.append(KF_ExtendedKalmanFilter( mode,x,y, self.forward_noise, self.GPS_std))

            self.filter_list_toa_tdoa.append(KF_ExtendedKalmanFilter( mode,x,y, self.forward_noise, self.GPS_std))
            self.filter_list_rss_toa.append(KF_ExtendedKalmanFilter( mode,x,y, self.forward_noise, self.GPS_std))
            self.filter_list_rss_tdoa.append(KF_ExtendedKalmanFilter( mode,x,y, self.forward_noise, self.GPS_std))
            self.filter_list_rss_toa_tdoa.append(KF_ExtendedKalmanFilter( mode,x,y, self.forward_noise, self.GPS_std))

        if mode in qaq:
            self.filter_list_rss.append(KF_ExtendedKalmanFilter( mode,x,y, self.forward_noise, self.GPS_std))
            self.filter_list_toa.append(KF_ExtendedKalmanFilter( mode,x,y, self.forward_noise, self.GPS_std))
            self.filter_list_tdoa.append(KF_ExtendedKalmanFilter( mode,x,y, self.forward_noise, self.GPS_std))

            self.filter_list_toa_tdoa.append(KF_ExtendedKalmanFilter( mode,x,y, self.forward_noise, self.GPS_std))
            self.filter_list_rss_toa.append(KF_ExtendedKalmanFilter( mode,x,y, self.forward_noise, self.GPS_std))
            self.filter_list_rss_tdoa.append(KF_ExtendedKalmanFilter( mode,x,y, self.forward_noise, self.GPS_std))
            self.filter_list_rss_toa_tdoa.append(KF_ExtendedKalmanFilter( mode,x,y, self.forward_noise, self.GPS_std))


        if mode == "PF_proposed":
            self.filter_list_rss.append(ParticleFilter(mode,x, y, self.channel, self.GPS_obj, self.num_particles, self.forward_noise, self.num_nodes))
            self.filter_list_toa.append(ParticleFilter(mode,x, y, self.channel, self.GPS_obj, self.num_particles, self.forward_noise, self.num_nodes))
            self.filter_list_tdoa.append(ParticleFilter(mode,x, y, self.channel, self.GPS_obj, self.num_particles, self.forward_noise, self.num_nodes))

            self.filter_list_toa_tdoa.append(ParticleFilter(mode,x, y, self.channel, self.GPS_obj, self.num_particles, self.forward_noise, self.num_nodes))
            self.filter_list_rss_toa.append(ParticleFilter(mode,x, y, self.channel, self.GPS_obj, self.num_particles, self.forward_noise, self.num_nodes))
            self.filter_list_rss_tdoa.append(ParticleFilter(mode,x, y, self.channel, self.GPS_obj, self.num_particles, self.forward_noise, self.num_nodes))
            self.filter_list_rss_toa_tdoa.append(ParticleFilter(mode,x, y, self.channel, self.GPS_obj, self.num_particles, self.forward_noise, self.num_nodes))

    def registerNeighbors(self, packet_loss):
    
        self.candidate = []
        for i in range(self.num_nodes):
            if i == self.ID: continue
            if self.all_measurement_rssi[i] > self.EnergyDetectionThreshold:
                if random.uniform(0, 1) >= packet_loss:
                    self.candidate.append(i)

    def handle_never_seen(self):

        for i in range(self.num_nodes):

            if i not in self.candidate: continue
            if self.neighbor_whether_meet_state[i] == 0: # never seen before 
                for filter_index in range(len(self.neighbor_KF)):
                    v = self.now_neighbor_velocity[i]
                    motion_x_std = v * cos(self.orientation) * self.interval_odometry * self.velocity_noise_percent
                    motion_y_std = v * sin(self.orientation) * self.interval_odometry * self.velocity_noise_percent   

                    anchor_x, anchor_y, anchor_ori  = self.perfect_neighbor[i].getposition()             
                    self.neighbor_KF[filter_index][i].set(self.all_measurement_gps[i][0][0], self.all_measurement_gps[i][1][0])
                    #self.neighbor_KF[filter_index][i].set(anchor_x, anchor_y)

                    self.neighbor_KF[filter_index][i].P = self.GPS_variance

                    self.neighbor_KF[filter_index][i].updateQ(motion_x_std, motion_y_std)
                    #self.neighbor_KF[filter_index][i].updateR(self.GPS_variance) # We think this neighbor should suffer from the same GPS noise

            self.neighbor_whether_meet_state[i] += 1



#    def getGDOP_rssi(self, mse):
#       b = pow(10.*self.PLE/self.shadowing/log(10) ,2)
#       return sqrt(mse*b)
    
    def gps_crlb_fisher(self):
    
        J = np.zeros((2, 2))
        Q = array([[self.motion_x_std**2, 0], [0, self.motion_y_std**2]])
        
        qqq = array([[1./(self.GPS_std**2),0],[0,1./(self.GPS_std**2)]])
        J += qqq
        J += inv(self.true_fisher_info_matrix_gps + Q)
        inv_J = inv(J)

        self.true_fisher_MSE_gps = inv_J[0][0]+ inv_J[1][1]
        self.true_fisher_info_matrix_gps = inv_J

    def ini_neighbor_KF(self, neighbor_measurement_gps, neighbor_motion_noise, neighbor_measurement_noise, filter_index):

        for i in range(self.num_nodes):
            tmp_x = neighbor_measurement_gps[i][0][0]
            tmp_y = neighbor_measurement_gps[i][1][0]
            self.neighbor_KF[filter_index].append(KalmanFilter("neighbor_of_{0}_{1}".format(self.ID, filter_index),tmp_x, tmp_y, neighbor_motion_noise, neighbor_measurement_noise))

    def updata_neighbor_velocity(self, velocity_list):
        for i in range(self.num_nodes):
            if i in self.candidate:
                self.now_neighbor_velocity[i] = velocity_list[i]

    def set_GPS_miss(self, flag):
        self.gps_miss = flag

    def updata_neighbor_table(self, filter_index, smart_filter_index, t):
    
        for i in range(self.num_nodes):

            if self.ID == i: continue
            # we have used initilalization process to update the current neighbor i situation 
            if self.neighbor_whether_meet_state[i] <= 1: continue

            v = self.now_neighbor_velocity[i]
            motion_x_std = v * cos(self.orientation) * self.interval_odometry * self.velocity_noise_percent
            motion_y_std = v * sin(self.orientation) * self.interval_odometry * self.velocity_noise_percent
            U = array([ [v*self.interval_odometry*cos(self.orientation)], [v*self.interval_odometry*sin(self.orientation)] ])
            # =========== NEIGHBOR KF MENTATION =================
            if i in self.candidate and not self.gps_miss[i]:

                #print self.all_measurement_gps[i]
                self.neighbor_KF[filter_index][i].updateQ(motion_x_std, motion_y_std)

                if i in self.smart_list and t>= 10000000 and smart_filter_index == filter_index:

                    #if len(self.observed_data[i])>flag and filter_index == 1 and t>= 1 :
                    #self.neighbor_KF[filter_index][i].updateR(self.observed_data[i][1])
                    #self.neighbor_KF[filter_index][i].update(self.observed_data[i][0])
                    #a = GPS_obj_list[i].getGPS_std()
                    #tmp_var = array([[a**2,0],[0,a**2]] )
                    #self.neighbor_KF[filter_index][i].updateR(tmp_var)
                    #self.neighbor_KF[filter_index][i].X = self.observed_data[i][0]
                    #self.neighbor_KF[filter_index][i].P = self.observed_data[i][1]

                    #self.neighbor_KF[filter_index][i].X[0][0] = 1
                    #anchor_x, anchor_y, anchor_ori  = self.perfect_neighbor[i].getposition()

                    self.neighbor_KF[filter_index][i].predict(U)
                    self.neighbor_KF[filter_index][i].update(self.GPS_variance, self.observed_data[i][0])

                else:
                    self.neighbor_KF[filter_index][i].predict(U)
                    self.neighbor_KF[filter_index][i].update(self.GPS_variance, self.all_measurement_gps[i])
                    #self.neighbor_KF[filter_index][i].updateR(self.neighbor_KF[filter_index][i].P)
            else:
                self.neighbor_KF[filter_index][i].predict(U)

    def updata_neighbor_table_rssi(self, filter_index):
    
    
        for i in range(self.num_nodes):
    
            # =========== NEIGHBOR KF MENTATION =================
            if i in self.candidate:

                old_x = self.neighbor_KF[filter_index][i].get_X()[0][0]
                old_y = self.neighbor_KF[filter_index][i].get_X()[1][0]
                own_x = self.get_filter_rss_X(filter_index)[0][0]
                own_y = self.get_filter_rss_X(filter_index)[1][0]
                dist = distance(old_x, old_y, own_x, own_y)
                h = array([[self.tx_power - self.PL0 - 10*self.PLE*np.log10(dist)]])
                EKF_H = array([[ -(10*self.PLE* ( old_x- own_x ))/ (pow( dist, 2)*log(10) ) , - (10*self.PLE* (old_y- own_y   )  )/(pow(dist, 2) *log(10)) ]])
                #EKF_R = array([[self.shadowing**2+ ((sqrt(self.neighbor_KF[filter_index][i].get_P()[0][0]) *10*self.PLE)/log(10)/dist)**2]])
                EKF_R = array([[self.shadowing**2 ]])

                self.neighbor_KF[filter_index][i].ekf_update(self.all_measurement_rssi[i], EKF_R, EKF_H, h)

    def set(self, new_x, new_y, new_orientation):

        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)
    
    def set_noise(self, forward_noise, turn_noise):

        self.forward_noise = float(forward_noise)
        self.turn_noise    = float(turn_noise)

    def Gaussian(self, mu, sigma, x):
        
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))

    def getposition(self):
        return self.x, self.y, self.orientation

    def updateControl2(self, turn, forward):

        self.turn = float(turn) + random.gauss(0.0, self.turn_noise)
        self.forward = float(forward) + random.gauss(0.0, self.forward_noise)
        self.orientation += self.turn 
        self.orientation %= 2 * pi

    def move(self):

        self.orientation += self.turn_velocity
        self.orientation %= 2 * pi
        
        self.x += self.velocity * self.interval_odometry * cos(self.orientation) 
        self.y += self.velocity * self.interval_odometry * sin(self.orientation) 


    def updateVelocity(self,velocity , turn_v):
        self.velocity = velocity
        self.turn_velocity = turn_v


    def updateControl(self):

        forward_tmp = self.velocity * self.interval_odometry

        self.motion_x_std = forward_tmp * cos(self.orientation) * self.velocity_noise_percent
        self.motion_y_std = forward_tmp * sin(self.orientation) * self.velocity_noise_percent

        observed_forward = forward_tmp + random.gauss(0.0, forward_tmp * self.velocity_noise_percent)
        #observed_forward = velocity * self.interval_odometry

        #observed_turn = self.orientation + random.gauss(0.0, self.turn_noise)
        #observed_turn %= 2 * pi
        observed_turn = self.orientation 

        self.control = [observed_forward*cos(observed_turn) , observed_forward*sin(observed_turn), observed_turn]
        #self.turn = float(turn) + random.gauss(0.0, self.turn_noise)
        #self.forward = float(forward) + random.gauss(0.0, self.forward_noise)
        #self.orientation += self.turn 
        #self.orientation %= 2 * pi
   
    def updata_ego_kf(self, index):

        U = array([ [self.control[0]], [self.control[1]] ])

        self.filter_list_rss[index].updateQ(self.motion_x_std, self.motion_y_std)
        self.filter_list_rss[index].predict(U)
        self.filter_list_toa[index].updateQ(self.motion_x_std, self.motion_y_std)
        self.filter_list_toa[index].predict(U)
        self.filter_list_tdoa[index].updateQ(self.motion_x_std, self.motion_y_std)
        self.filter_list_tdoa[index].predict(U)

        self.filter_list_toa_tdoa[index].updateQ(self.motion_x_std, self.motion_y_std)
        self.filter_list_toa_tdoa[index].predict(U)
        self.filter_list_rss_toa[index].updateQ(self.motion_x_std, self.motion_y_std)
        self.filter_list_rss_toa[index].predict(U)
        self.filter_list_rss_tdoa[index].updateQ(self.motion_x_std, self.motion_y_std)
        self.filter_list_rss_tdoa[index].predict(U)
        self.filter_list_rss_toa_tdoa[index].updateQ(self.motion_x_std, self.motion_y_std)
        self.filter_list_rss_toa_tdoa[index].predict(U)

        if not self.gps_miss[self.ID]:
            self.filter_list_rss[index].update(self.GPS_variance, self.all_measurement_gps[self.ID])
            self.filter_list_toa[index].update(self.GPS_variance, self.all_measurement_gps[self.ID])
            self.filter_list_tdoa[index].update(self.GPS_variance, self.all_measurement_gps[self.ID])
            self.filter_list_toa_tdoa[index].update(self.GPS_variance, self.all_measurement_gps[self.ID])
            self.filter_list_rss_toa[index].update(self.GPS_variance, self.all_measurement_gps[self.ID])
            self.filter_list_rss_tdoa[index].update(self.GPS_variance, self.all_measurement_gps[self.ID])
            self.filter_list_rss_toa_tdoa[index].update(self.GPS_variance, self.all_measurement_gps[self.ID])


    def get_filter_rss_X(self, index):
        return np.copy(self.filter_list_rss[index].get_X())

    def get_filter_toa_X(self, index):
        return np.copy(self.filter_list_toa[index].get_X())

    def get_filter_tdoa_X(self, index):
        return np.copy(self.filter_list_tdoa[index].get_X())  
    def get_filter_toa_tdoa_X(self, index):
        return np.copy(self.filter_list_toa_tdoa[index].get_X())
    def get_filter_rss_toa_X(self, index):
        return np.copy(self.filter_list_rss_toa[index].get_X())
    def get_filter_rss_tdoa_X(self, index):
        return np.copy(self.filter_list_rss_tdoa[index].get_X())
    def get_filter_rss_toa_tdoa_X(self, index):    
        return np.copy(self.filter_list_rss_toa_tdoa[index].get_X())


    def get_filter_rss_P(self, index):
        return np.copy(self.filter_list_rss[index].get_P())

    def get_filter_toa_P(self, index):
        return np.copy(self.filter_list_toa[index].get_P())   

    def get_filter_tdoa_P(self, index):
        return np.copy(self.filter_list_tdoa[index].get_P())
    
    def get_filter_toa_tdoa_P(self, index):
        return np.copy(self.filter_list_toa_tdoa[index].get_P())
    def get_filter_rss_toa_P(self, index):
        return np.copy(self.filter_list_rss_toa[index].get_P())
    def get_filter_rss_tdoa_P(self, index):
        return np.copy(self.filter_list_rss_tdoa[index].get_P())
    def get_filter_rss_toa_tdoa_P(self, index):    
        return np.copy(self.filter_list_rss_toa_tdoa[index].get_P())
            

    def __repr__(self):
        return '[x=%.6s y=%.6s orient=%.6s factor=%.6s factor2=%.6s\n]' % (str(self.x), str(self.y), str(self.orientation), str(self.PLE),str(self.PL0))








