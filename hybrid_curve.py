import numpy as np
from numpy import *
import matplotlib.pyplot as plt
from math import *
import random
import numpy as np
import random
import sys

sys.path.insert( 0,'./src/')
from GPS_simulator import *
from CHANNEL import *
from TOA_MODEL import *
from TDOA_MODEL import *
from nodes import *
from TOOLBOX import *
from TOOLBOX2 import *
from EVALUATION import *
from CRLB_TOOL import *

from RSS_TOA_TDOA_hybrid_update import *
from TOA_MODEL_UPDATE import *
import time


np.random.seed(1)
random.seed(1)

error_test_set = [\
"KF_EKF_GPS_ori","KF_EKF_table_NU_CT","Fisher"
\
]

#rss
tx_power = 20.
real_PLE = 2.8
real_PL0 = 65

#toa
c = 3.*pow(10,8)
t0 = 0.

real_x_ori = 25
real_y_ori = 25
real_loc = (real_x_ori, real_y_ori)

num_particles = 200
#test_GPS = float(sys.argv[1])

runs = 100
TOTAL_EPISODE = 300
flag = 1

packet_loss = 0.0
quantile, number_gps_samples, GPS_bias = 0.95, 1, array([0, 0]).T

#==============default_set================
world_size = 100
num_anchors = 20
shadow = 6.0
syn_error = 4.*pow(10,-8)
accuracy = 17.0
velocity = 20.0
turn_velocity = 0.01
smart_prob = 0.0
velocity_noise_percent = 0.1
plot_helper = None
seperate_space = 20
variable = sys.argv[1]
plot_helper = num_anchors


if variable == "num_vehicles":
    num_anchors = int(sys.argv[2])
    plot_helper = num_anchors
elif variable == "syn_error":
    syn_error = float(sys.argv[2])
    plot_helper = syn_error

elif variable == "gps_accuracy":
    accuracy = float(sys.argv[2])
    plot_helper = accuracy
elif variable == "gps_std":
    GPS_std = float(sys.argv[2])
    plot_helper = GPS_std

elif variable == "velocity":
    velocity = float(sys.argv[2])
    plot_helper = velocity
elif variable == "smart_prob":
    smart_prob = float(sys.argv[2])
    plot_helper = smart_prob
elif variable == "motion_noise":
    velocity_noise_percent = float(sys.argv[2])
    plot_helper = velocity_noise_percent

else:
    print ("Wrong for variable!")

#=========================================

rss_model = CHANNEL(tx_power, real_PL0, real_PLE, shadow, -10000000)
toa_model = TOA_MODEL(t0, c, syn_error)
tdoa_model = TDOA_MODEL(c, 2*syn_error)

GPS_obj_list = []
for i in range(num_anchors):
    GPS_obj_list.append(GPS_MODEL(accuracy, quantile, number_gps_samples))
    if variable == "gps_std":
        GPS_obj_list[-1].change_GPSstd(GPS_std)

GPS_std = GPS_obj_list[0].getGPS_std()

#=========================== Mobility noise model ==========================
turn_noise = turn_velocity * 0.1
motion_noise = velocity * velocity_noise_percent * 0.1

#=========================== Evaluation object =============================

# to record the resluts for every test (epoch) 
# for test_name, vehicles in senario, total episode

eva_obj_list_rss_toa_tdoa = []
eva_obj_list_toa = []

eva_obj_total_error_rss_toa_tdoa = []
eva_obj_total_error_toa = []

eva_obj_total_smart_error_rss_toa_tdoa = []
eva_obj_total_smart_error_toa = []

#every list matix(len(error_test_set),len(num_anchors)) in eva_obj y_coordinatE with list lenth TOTAL_EPISODE  
for i in range(len(error_test_set)):
    eva_obj_list_rss_toa_tdoa.append([])
    eva_obj_list_toa.append([])

    eva_obj_total_error_rss_toa_tdoa.append([])
    eva_obj_total_error_toa.append([])

    for j in range(num_anchors):
        eva_obj_list_rss_toa_tdoa[i].append( Evaluation(j, error_test_set[i], TOTAL_EPISODE ,runs) )
        eva_obj_list_toa[i].append( Evaluation(j, error_test_set[i], TOTAL_EPISODE ,runs) )
       

        eva_obj_total_error_rss_toa_tdoa[i].append( Evaluation(j, error_test_set[i], TOTAL_EPISODE ,runs) )
        eva_obj_total_error_toa[i].append( Evaluation(j, error_test_set[i], TOTAL_EPISODE ,runs) )
        

    eva_obj_total_smart_error_rss_toa_tdoa.append( Evaluation("Total_Smart", error_test_set[i], TOTAL_EPISODE ,runs) )
    eva_obj_total_smart_error_toa.append( Evaluation("Total_Smart", error_test_set[i], TOTAL_EPISODE ,runs) )
    

#==========================================================================================================

#if the neighbors return the hybrud system prediction
smart = choose_smart_vehicle(num_anchors, smart_prob)

smart_filter_index = 0

num_neighbors = []

color_list = []
for i in range(len(error_test_set)):
    color_list.append((random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1)) )
fig = plt.figure()
ax = fig.add_subplot(111, aspect='auto')

for episode in range(TOTAL_EPISODE):

    #print episode

    ini_direction = pi/4.

    anchors = genAnchors(num_anchors, world_size, rss_model, toa_model,tdoa_model, motion_noise, turn_noise, num_particles, smart, GPS_obj_list, ini_direction, velocity_noise_percent)
    GPS_bias_neighbors = genBias(GPS_bias, num_anchors) #bias list with length num_anchors

    ini_neighbor_measurement_gps = []
    ini_real_loc = []

    for i in range(num_anchors):
        anchor_x, anchor_y, anchor_o = anchors[i].getposition()
        # construct the x,y in GPS_obj_list from the given quantile and add gaussian noise and append to neighbor_measurement)gps
        ini_neighbor_measurement_gps.append(GPS_obj_list[i].genGPS_coordinate_bias(anchor_x, anchor_y, GPS_bias_neighbors[i])) 
        ini_real_loc.append([[anchor_x], [anchor_y]])

        for j in range(len(error_test_set)):
            if error_test_set[j] == "Fisher" or error_test_set[j] == "Fisher_measurement" or error_test_set[j] == "Fisher_gps": continue
            if i not in smart: continue
            anchors[i].genFilteringObj(error_test_set[j], ini_neighbor_measurement_gps[i][0][0], ini_neighbor_measurement_gps[i][1][0]) #
            #anchors[i].genFilteringObj(error_test_set[j], anchor_x, anchor_y)

    for i in range(num_anchors):
        for j in range(len(error_test_set)):
            if error_test_set[j] == "Fisher" or error_test_set[j] == "Fisher_measurement" or error_test_set[j] == "Fisher_gps": continue
            if i not in smart: continue

            #anchors[i].ini_neighbor_KF(ini_real_loc, motion_noise, GPS_std, j)
            anchors[i].ini_neighbor_KF(ini_neighbor_measurement_gps, motion_noise, GPS_std, j)

    #==================== TIME PROCESSING ==================
    velocity_list = genVelocity(num_anchors, velocity)
    turn_list = gentTurnVelocity(num_anchors, turn_velocity)
    velocity_list[0] = velocity
    turn_list[0] = turn_velocity

    fake_velocity = velocity_list

    num_neighbors.append([])
    orientation = ini_direction

    last_measurement_gps = []
    all_measurement_gps = []


    for t in range(runs):

        #print "time:", t
        
        last_velocity_list = fake_velocity

        last_measurement_gps = all_measurement_gps

        all_measurement_gps = []
        observed_data = []
        true_all_loc = []

        # ======================================= Move =======================================
        fake_velocity = []
        orientation += turn_velocity
        all_node_loc = []


        for i in range(len(anchors)):

            anchors[i].updateVelocity(velocity_list[i], turn_list[i])

            #for the real move
            anchors[i].move() 
            #for the control error
            anchors[i].updateControl()
            fake_velocity.append(sqrt(anchors[i].control[0]**2 + anchors[i].control[1]**2) * 10 )

            anchor_x, anchor_y, anchor_o = anchors[i].getposition()
            all_node_loc.append([anchor_x, anchor_y])

        # ======================================= Obesrvation =======================================
        for i in range(len(anchors)):
            observed_data.append([])

            anchor_x, anchor_y, anchor_o = anchors[i].getposition()
            true_all_loc.append([anchor_x ,anchor_y])
            if i in smart:
                anchors[i].communicating_LogNormalModel(anchors)
                anchors[i].communicating_TOA(anchors)
                anchors[i].communicating_TDOA(anchors)

            tmp_GPS = GPS_obj_list[i].genGPS_coordinate_bias(anchor_x, anchor_y, GPS_bias_neighbors[i])

            all_measurement_gps.append(tmp_GPS)
            observed_data[i].append(tmp_GPS)
            
        # ======================================= Localization =======================================
        for i in range(len(anchors)):

            if i not in smart: continue # only smart do the update
            anchors[i].setAllinfor(all_measurement_gps, observed_data, anchors)
            anchors[i].registerNeighbors(packet_loss)
            anchors[i].updata_neighbor_velocity(last_velocity_list)
            anchors[i].handle_never_seen()

            num_neighbors[episode].append(len(anchors[i].candidate))

            for j in range(len(error_test_set)):

                if error_test_set[j] == "Fisher": 
                    rss_toa_tdoa_motion_gps_crlb_fisher(anchors[i])
                    toa_motion_gps_crlb_fisher(anchors[i])
                    continue

                anchors[i].updata_neighbor_table(j, smart_filter_index, t)

                if error_test_set[j] == "KF":
                    anchors[i].updata_ego_kf(j)

                if error_test_set[j] == "KF_EKF_GPS_ori": 
                    update_ego_kf_ekf_GPS_ori_rss_toa_tdoa(anchors[i],j)
                    update_ego_kf_ekf_GPS_ori_toa(anchors[i],j)
                    

                if error_test_set[j] == "KF_EKF_table_NU_CT":
                    update_ego_kf_ekf_table_nu_Ctable_rss_toa_tdoa(anchors[i],j)
                    update_ego_kf_ekf_table_nu_Ctable_toa(anchors[i],j)
                           
            anchor_x, anchor_y, anchor_o = anchors[i].getposition()
            for j in range(len(error_test_set)):
                #becuse smart = 0, i alwasys = 0 
                if error_test_set[j] == "Fisher": 
                    eva_obj_list_rss_toa_tdoa[j][i].push_error_back(episode, anchors[0].true_fisher_MSE_rss_toa_tdoa_gps)
                    eva_obj_list_toa[j][i].push_error_back(episode, anchors[0].true_fisher_MSE_toa_gps) 
                    
                    continue

                # rss    
                tmp_X_rss_toa_tdoa = anchors[i].get_filter_rss_toa_tdoa_X(j)
                tmp_X_toa = anchors[i].get_filter_toa_X(j)

                error_square_rss_toa_tdoa = distance2(anchor_x, anchor_y, tmp_X_rss_toa_tdoa[0][0], tmp_X_rss_toa_tdoa[1][0])
                error_square_toa = distance2(anchor_x, anchor_y, tmp_X_toa[0][0], tmp_X_toa[1][0])
                

                eva_obj_list_rss_toa_tdoa[j][i].push_error_back(episode, error_square_rss_toa_tdoa)
                eva_obj_list_toa[j][i].push_error_back(episode, error_square_toa)
                        

                total_error_rss_toa_tdoa = cal_all_error_rss_toa_tdoa(anchors, j, i)
                total_error_toa = cal_all_error_toa(anchors, j, i)
            

                eva_obj_total_error_rss_toa_tdoa[j][i].push_error_back(episode, total_error_rss_toa_tdoa)
                eva_obj_total_error_toa[j][i].push_error_back(episode, total_error_toa)
               
      


        for j in range(len(error_test_set)):
            if error_test_set[j] == "Fisher" or error_test_set[j] == "Fisher_measurement" or error_test_set[j] == "Fisher_gps": continue
            total_smart_error_rss_toa_tdoa = cal_all_smart_error_rss_toa_tdoa(anchors, j, smart)
            total_smart_error_toa = cal_all_smart_error_toa(anchors, j, smart)
            

            eva_obj_total_smart_error_rss_toa_tdoa[j].push_error_back(episode, total_smart_error_rss_toa_tdoa)
            eva_obj_total_smart_error_toa[j].push_error_back(episode, total_smart_error_toa)
        
        #a_loc = getAllLOC(anchors, num_anchors)
        #plot_dynamic2(a_loc, smart, anchors, all_measurement_gps, color_list,ax)
        #plot_dynamic_ellipse(a_loc, smart, anchors, all_measurement_gps, color_list, ax, 2,8)

    # ======================================= Output DATA =======================================
    for j in range(len(error_test_set)):
        #title = "_{0}_{1}".format(variable, plot_helper)
        title = "./raw/{0}/total_smart_{1}_{0}_{2}_rss_toa_tdoa.txt".format(variable, error_test_set[j], plot_helper)
        eva_obj_total_smart_error_rss_toa_tdoa[j].outText(title, episode)
        title = "./raw/{0}/total_smart_{1}_{0}_{2}_toa.txt".format(variable, error_test_set[j], plot_helper)
        eva_obj_total_smart_error_toa[j].outText(title, episode)
        

        for i in range(num_anchors):
            if i not in smart: continue
            title = "./raw/{0}/error_{1}_node{3}_{0}_{2}_rss_toa_tdoa.txt".format(variable, error_test_set[j], plot_helper, i)
            eva_obj_list_rss_toa_tdoa[j][i].outText(title, episode)
            title = "./raw/{0}/error_{1}_node{3}_{0}_{2}_toa.txt".format(variable, error_test_set[j], plot_helper, i)
            eva_obj_list_toa[j][i].outText(title, episode)

            title = "./raw/{0}/total_neighbor_node{3}_{1}_{0}_{2}_rss_toa_tdoa.txt".format(variable, error_test_set[j], plot_helper, i)
            eva_obj_total_error_rss_toa_tdoa[j][i].outText(title, episode)
            title = "./raw/{0}/total_neighbor_node{3}_{1}_{0}_{2}_toa.txt".format(variable, error_test_set[j], plot_helper, i)
            eva_obj_total_error_toa[j][i].outText(title, episode)
            

#====================== PLOT ============================================= 

fig = plt.figure()
ax = plt.subplot(1, 1, 1)

for j in range(len(error_test_set)):
    if error_test_set[j] == "Fisher" or error_test_set[j] == "Fisher_measurement" or error_test_set[j] == "Fisher_gps": continue
    plt.plot(range(runs), eva_obj_total_smart_error_rss_toa_tdoa[j].gethistMean(),label=error_test_set[j])
plt.legend(loc=0)
plt.savefig("./figures/{0}/total_smart_error_rss_toa_tdoa_{0}_{1}.pdf".format(variable, plot_helper))

fig = plt.figure()
ax = plt.subplot(1, 1, 1)

for j in range(len(error_test_set)):
    if error_test_set[j] == "Fisher" or error_test_set[j] == "Fisher_measurement" or error_test_set[j] == "Fisher_gps": continue
    plt.plot(range(runs), eva_obj_total_smart_error_toa[j].gethistMean(),label=error_test_set[j])
plt.legend(loc=0)
plt.savefig("./figures/{0}/total_smart_error_toa_{0}_{1}.pdf".format(variable, plot_helper))


#===========================================================================
#color = ['r','k','m','g','y','b','c']

for i in range(num_anchors):
    if i in smart:
        #
        fig = plt.figure()
        ax = plt.subplot(1, 1, 1)

        for j in range(len(error_test_set)):
            tmpqq = [sqrt(k) for k in eva_obj_list_rss_toa_tdoa[j][i].gethistMean()]
            plt.plot(range(runs),  tmpqq  ,c=color_list[j], label=error_test_set[j])

        plt.legend(loc=0,prop={'size': 6})
        ax.set_title('rss_{0}_{1}'.format(variable,plot_helper))
        ax.set_xlabel('time')
        ax.set_ylabel('error'+'('+'m'+')')
        plt.savefig("./figures/{0}/rss_toa_tdoa_mse_node{1}_{0}_{2}.pdf".format(variable, i, plot_helper))

        fig = plt.figure()
        ax = plt.subplot(1, 1, 1)

        for j in range(len(error_test_set)):
            if error_test_set[j] == "Fisher" or error_test_set[j] == "Fisher_measurement" or error_test_set[j] == "Fisher_gps": continue
            plt.plot(range(runs),  eva_obj_total_error_rss_toa_tdoa[j][i].gethistMean()  ,c=color_list[j], label=error_test_set[j])

        ax.set_title('rss-{0}-Error-{1}times-average_std'.format(variable,TOTAL_EPISODE))
        plt.legend(loc=0,prop={'size': 6})
        plt.savefig("./figures/{0}/rss_toa_tdoa_neighbor_error_node{1}_{0}_{2}.pdf".format(variable, i, plot_helper))

for i in range(num_anchors):
    if i in smart:
        fig = plt.figure()
        ax = plt.subplot(1, 1, 1)

        for j in range(len(error_test_set)):
            tmpqq = [sqrt(k) for k in eva_obj_list_toa[j][i].gethistMean()]
            plt.plot(range(runs),  tmpqq  ,c=color_list[j], label=error_test_set[j])

        plt.legend(loc=0,prop={'size': 6})
        ax.set_title('rss_{0}_{1}'.format(variable,plot_helper))
        ax.set_xlabel('time')
        ax.set_ylabel('error'+'('+'m'+')')
        plt.savefig("./figures/{0}/toa_mse_node{1}_{0}_{2}.pdf".format(variable, i, plot_helper))

        fig = plt.figure()
        ax = plt.subplot(1, 1, 1)

        for j in range(len(error_test_set)):
            if error_test_set[j] == "Fisher" or error_test_set[j] == "Fisher_measurement" or error_test_set[j] == "Fisher_gps": continue
            plt.plot(range(runs),  eva_obj_total_error_toa[j][i].gethistMean()  ,c=color_list[j], label=error_test_set[j])

        ax.set_title('rss-{0}-Error-{1}times-average_std'.format(variable,TOTAL_EPISODE))
        plt.legend(loc=0,prop={'size': 6})
        plt.savefig("./figures/{0}/toa_neighbor_error_node{1}_{0}_{2}.pdf".format(variable, i, plot_helper))        

#===========================================================================
for i in range(num_anchors):
    data = []
    if i in smart:
        for j in range(len(error_test_set)):
            tmp_hist = np.array(eva_obj_list_rss_toa_tdoa[j][i].hist_error)
            data.append(tmp_hist)

        fig = plt.figure()
        ax = plt.subplot(1, 1, 1)
        index = np.arange(1)
        bar_width = 0.1
        out = genBar(data, index, bar_width, color_list, error_test_set)

        for q in range(len(data)):
            createLabels(out[q])
        plt.legend(loc=0,prop={'size': 6})
        ax.set_title('rss_toa_tdoa_mean_std_{0}_{1}'.format(variable,plot_helper))
        ax.set_xlabel('filters')
        ax.set_ylabel('error'+'('+'m'+')')

        plt.savefig("./figures/{0}/rss_toa_tdoa_mean_std_node{1}_{0}_{2}.pdf".format(variable, i, plot_helper))

for i in range(num_anchors):
    data = []
    if i in smart:
        for j in range(len(error_test_set)):
            tmp_hist = np.array(eva_obj_list_toa[j][i].hist_error)
            data.append(tmp_hist)

        fig = plt.figure()
        ax = plt.subplot(1, 1, 1)
        index = np.arange(1)
        bar_width = 0.1
        out = genBar(data, index, bar_width, color_list, error_test_set)

        for q in range(len(data)):
            createLabels(out[q])
        plt.legend(loc=0,prop={'size': 6})
        ax.set_title('toa_mean_std_{0}_{1}'.format(variable,plot_helper))
        ax.set_xlabel('filters')
        ax.set_ylabel('error'+'('+'m'+')')

        plt.savefig("./figures/{0}/toa_mean_std_node{1}_{0}_{2}.pdf".format(variable, i, plot_helper))

