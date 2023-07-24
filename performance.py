import numpy as np
import sys
import matplotlib
import matplotlib.pyplot as plt
from math import *
import random


random.seed(1)
name = "RSS-TOA-TDOA-Curve"
error_test_set = [\

"KF",

"KF_EKF_ori", "KF_EKF_table_mean",  \

"KF_EKF_proposed",

"KF_EKF_ECRLB",

"KF_EKF_perfect",

"KF_EKF_CRLB",

  "Fisher"\
]

error_test_set = [\
"KF", "KF_EKF_GPS_ori", "KF_EKF_table_ori",
"KF_EKF_table_NU_CT","KF_EKF_table_ECRLB_CT_estimate",
"KF_EKF_true_NU_CT","KF_EKF_ture_ECRLB_CT_TF","Fisher"
\
]


performance_dir = ["num_vehicles", "shadowing_effect", "gps_std", "velocity","motion_noise"]
value_list = [0,1,2,3]
range_number_list = [10,7+1,7+1,10+1,12]
seperate_list = [5,3,3,4]

color_list = []
for i in range(len(error_test_set)):
    color_list.append((random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1)) )

def genBar(data, index, bar_width, color, labels):
    context = []
    for i in range(len(data)):
        tmp_mean = np.mean(data[i])
        tmp_x = index+i*bar_width
        context.append(
            plt.bar(tmp_x,
           tmp_mean, 
           bar_width,
           color=color[i],
           alpha=.5,
           label=labels[i]) 
            )

        plt.errorbar(tmp_x+0.5*bar_width,tmp_mean, yerr=np.std(data[i],ddof = 1),fmt='o',ecolor=color[i],color=color[i],elinewidth=2,capsize=4)
    return context

def createLabels(data):
    for item in data:
        height = item.get_height()

        plt.text(
            item.get_x()+item.get_width()/2., 
            height*0.1, 
            '%.3f' % float(height),
            ha = "center",
            va = "bottom",
        )

def pairInfo(title):
    pairinfo = []
    for line in open(title,'r'):
        pairinfo.append([])
        for i in range(len(line.split(" "))):
            pairinfo[-1].append((float)(line.split(" ")[i]))
  
    return pairinfo

def pairInfo_mse(title):
    pairinfo = []
    for line in open(title,'r'):
        pairinfo.append([])
        for i in range(len(line.split(" "))):
            pairinfo[-1].append(  sqrt((float)(line.split(" ")[i]))   )
  
    return pairinfo


def compute_mean(data):
    tmp = []
    for i in range(len(data)):
        tmp.append(np.mean(data[i]))
    return tmp

def compute_mean_mse(data):
    tmp = []
    for i in range(len(data)):
        tmp.append(  sqrt(np.mean(data[i]))  )
    return tmp

def compute_std(data):
    tmp = []
    for i in range(len(data)):
        tmp.append(np.std(data[i],ddof = 1))
    return tmp


def plot_performance_std(performance, range_number, error_test_set, node_index):

    raw_data2 = []
    
    fig = plt.figure()
    ax = plt.subplot(1, 1, 1)
    for j in range(len(error_test_set)):
        raw_data = []
        for variable in range(range_number):
            if performance == "velocity":
                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, variable+0.0)
            elif performance == "num_vehicles":
                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, variable+1)
            elif performance == "seperate_space":
                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, (variable+1)*5. )
               

            else:
                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, variable+1.0)
            tmp_data = pairInfo(title)
            raw_data.append(tmp_data)
        a = compute_std(raw_data)
        
        new_list = [x+1 for x in range(len(a))]
    
        plt.plot(new_list, a, label=error_test_set[j])

    ax.set_title('{0}-Error-{1}times-average_std'.format(performance,len(tmp_data)))
    plt.legend(loc=0)
    plt.xlim(1, range_number)
    ax.set_xlabel('{0}'.format(performance))
    ax.set_ylabel('error(m)')
    
    plt.savefig("{0}-Error-{1}times-average_std{2}.pdf".format(performance,len(tmp_data),node_index ))


def plot_performance_std_mse(performance, range_number, error_test_set, node_index):

    raw_data2 = []
    
    fig = plt.figure()
    ax = plt.subplot(1, 1, 1)
    for j in range(len(error_test_set)):
        raw_data = []
        for variable in range(range_number):
            if performance == "velocity":
                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, variable+0.0)
            elif performance == "num_vehicles":
                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, variable+1)
            elif performance == "seperate_space":
                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, (variable+1)*5. )
            else:
                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, variable+1.0)
            tmp_data = pairInfo_mse(title)
            raw_data.append(tmp_data)
        a = compute_std(raw_data)
        
        new_list = [x+1 for x in range(len(a))]
    
        plt.plot(new_list, a, label=error_test_set[j])

    ax.set_title('{0}-Error-{1}times-average_std'.format(performance,len(tmp_data)))
    plt.legend(loc=0)
    plt.xlim(1, range_number)
    ax.set_xlabel('{0}'.format(performance))
    ax.set_ylabel('error(m)')
    
    plt.savefig("{0}-Error-{1}times-average_std{2}.pdf".format(performance,len(tmp_data),node_index ))
def plot_performance_std_mse_seperate(performance, range_number, error_test_set, node_index,seperate):

    raw_data2 = []
    
    fig = plt.figure()
    ax = plt.subplot(1, 1, 1)
    for j in range(len(error_test_set)):
        raw_data = []
        x_list = []
        for variable in range(range_number):
            if performance == "velocity":
                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, variable+0.0)
            elif performance == "num_vehicles":
                if (variable+1)%seperate !=0 and variable!= 0: continue
                x_list.append(variable+1)                
                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, variable+1)
            elif performance == "seperate_space":
                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, (variable+1)*5. )
            else:
                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, variable+1.0)
            tmp_data = pairInfo_mse(title)
            raw_data.append(tmp_data)
        a = compute_std(raw_data)
        
        plt.plot(x_list, a, label=error_test_set[j])

    ax.set_title('{0}-Error-{1}times-average_std'.format(performance,len(tmp_data)))
    plt.legend(loc=0)
    plt.xlim(1, range_number)
    ax.set_xlabel('{0}'.format(performance))
    ax.set_ylabel('error(m)')
    
    plt.savefig("{0}-Error-{1}times-average_std{2}.pdf".format(performance,len(tmp_data),node_index ))




def plot_performance_mean(performance, range_number, error_test_set, node_index):

    raw_data2 = []
    
    fig = plt.figure()
    ax = plt.subplot(1, 1, 1)
    for j in range(len(error_test_set)):
        raw_data = []
        for variable in range(range_number):
            if performance == "velocity":
                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, variable+0.0)
            elif performance == "num_vehicles":
                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, variable+1)
            elif performance == "seperate_space":
                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, (variable+1)*5. )

            else:
                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, variable+1.0)
            tmp_data = pairInfo(title)
            raw_data.append(tmp_data)
        a = compute_mean(raw_data)

        new_list = [x+1 for x in range(len(a))]
    
        plt.plot(new_list, a, label=error_test_set[j])

    ax.set_title('{0}-Error-{1}times-average_mean'.format(performance, len(tmp_data)))
    plt.legend(loc=0)
    plt.xlim(1, range_number)
    ax.set_xlabel('{0}'.format(performance))
    ax.set_ylabel('error(m)')
    
    plt.savefig("{0}-Error-{1}times-average_mean{2}.pdf".format(performance,len(tmp_data),node_index ))





def plot_performance_mean_mse(performance, range_number, error_test_set, node_index):

    raw_data2 = []
    
    fig = plt.figure()
    ax = plt.subplot(1, 1, 1)
    for j in range(len(error_test_set)):
        raw_data = []
        for variable in range(range_number):
            if performance == "velocity":
                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, variable+0.0)
            elif performance == "num_vehicles":

                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, variable+1)
            elif performance == "seperate_space":
                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, (variable+1)*5. )
            else:
                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, variable+1.0)
            tmp_data = pairInfo(title)
            raw_data.append(tmp_data)
        a = compute_mean_mse(raw_data)

        new_list = [x+1 for x in range(len(a))]
    
        plt.plot(new_list, a, label=error_test_set[j])



    ax.set_title('{0} - RMSE'.format(performance, len(tmp_data)))
    plt.legend(loc=0)
    plt.xlim(1, range_number)
    ax.set_xlabel('{0}'.format(performance))
    ax.set_ylabel('error(m)')
    
    plt.savefig("{0}-Error-{1}times-average_mean{2}.pdf".format(performance,len(tmp_data),node_index ))



def plot_performance_mean_mse_seperate(performance, range_number, error_test_set, node_index, seperate):

    raw_data2 = []
    
    fig = plt.figure()
    ax = plt.subplot(1, 1, 1)

    for j in range(len(error_test_set)):
        raw_data = []
        x_list = []
        for variable in range(range_number):

            if performance == "velocity":
                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, variable+0.0)
            elif performance == "num_vehicles":
                if (variable+1)%seperate !=0 and variable!= 0: continue
                x_list.append(variable+1)
                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, variable+1)
            elif performance == "seperate_space":
                x_list.append(variable)
                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, (variable+1)*5. )
            elif performance == "shadowing_effect":
                if (variable+1)%seperate !=0 and variable!= 0: continue
                x_list.append(variable+1)
                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, (variable+1.0) )
            else:
                if (variable+1)%seperate !=0 and variable!= 0: continue
                x_list.append(variable+1)
                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, (variable+1.0) )


            tmp_data = pairInfo(title)
            raw_data.append(tmp_data)
        a = compute_mean_mse(raw_data)
    
        plt.plot(x_list, a, label=error_test_set[j])

    ax.set_title('{0}-Error-{1}times-average_mean'.format(performance, len(tmp_data)))
    plt.legend(loc=0)
    plt.xlim(1, range_number)
    ax.set_xlabel('{0}'.format(performance))
    ax.set_ylabel('error(m)')
    
    plt.savefig("{0}-Error-{1}times-average_mean{2}.pdf".format(performance,len(tmp_data),node_index ))


def plot_performance_mean_mse_seperate_multi(performance, range_number, error_test_set, node_index, multi, color_list):

    raw_data2 = []
    
    fig = plt.figure()
    ax = plt.subplot(1, 1, 1)

    for j in range(len(error_test_set)):
        raw_data = []
        x_list = []
        for variable in range(range_number):

            if performance == "velocity":
                x_list.append( (variable+0.0)*multi)
                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, (variable+0.0)*multi)
            elif performance == "num_vehicles":
                x_list.append( (variable+1)*multi)
                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, (variable+1)*multi)

            elif performance == "motion_noise":
                x_list.append((variable)*0.1)
                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, (variable)*0.1 )
            elif performance == "shadowing_effect":
                x_list.append(variable*multi)
                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, (variable+0.0)*multi )
                if variable==0: 
                    x_list[-1] = variable+1
                    title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, (variable+1.0))
            elif performance == "gps_std":

                x_list.append(variable*multi)
                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, (variable+0.0)*multi )
                if variable==0: 
                    x_list[-1] = variable+1
                    title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, (variable+1.0))

            else:
                if (variable+1)%seperate !=0 and variable!= 0: continue
                x_list.append(variable+1)
                title = "raw/{2}/error_{0}_node{1}_{2}_{3}.txt".format(error_test_set[j], node_index, performance, (variable+1.0) )


            tmp_data = pairInfo(title)
            raw_data.append(tmp_data)
        a = compute_mean_mse(raw_data)
        print (x_list)

        marker_style = dict( marker='o',markersize=10,markeredgewidth=2,fillstyle="none")

        if error_test_set[j] == "KF_EKF_ECRLB":
            plt.plot(x_list, a, c=color_list[j], label="KF_EKF_proposed_ECRLB", linewidth=2,**marker_style)
        elif error_test_set[j] == "KF_EKF_ECRLB_dream":
            plt.plot(x_list, a, c=color_list[j], label="KF_EKF_proposed_ECRLB_brute", linewidth=2 ,**marker_style)
        elif error_test_set[j] == "Fisher":
            plt.plot(x_list, a, c=color_list[j], label="DCRLB", linewidth=2 ,**marker_style)
        else:
            plt.plot(x_list, a, c=color_list[j], label=error_test_set[j], linewidth=2 ,**marker_style)

    if performance == "num_vehicles":

        ax.set_title('{0} - Number of vehicles - RMSE'.format(name))
    if performance == "shadowing_effect":

        ax.set_title('{0} - Shadowing effect - RMSE'.format(name))

    if performance == "gps_std":

        ax.set_title('{0} - GPS standard deviation - RMSE'.format(name))
    if performance == "velocity":
        ax.set_title('{0} - Velocity - RMSE'.format(name))


    plt.legend(loc=0, prop={'size': 8},framealpha=0.5)
    #plt.legend(loc=0, )
    plt.xlim(min(x_list), max(x_list))
    if performance == "velocity":
        ax.set_xlabel('{0} m/s'.format(performance))
    else:
        ax.set_xlabel('{0}'.format(performance))
    ax.set_ylabel('error(m)')
    
    plt.savefig("{0}-Error-{1}times-average_mean{2}.pdf".format(performance,len(tmp_data),node_index ))


raw_data = []

target_node = 0

for i in range(len(performance_dir)):
    print ("performance: ", performance_dir[i])
    #if i == 0 or i == 3  : continue
    if i not in value_list:continue
    plot_performance_mean_mse_seperate_multi(performance_dir[i],range_number_list[i], error_test_set, target_node, seperate_list[i],color_list)
    #plot_performance_std_mse_seperate(performance_dir[i],range_number_list[i], error_test_set, target_node, 5)
    #if i != 0:continue
    #plot_performance_mean_mse(performance_dir[i], range_number_list[i], error_test_set, target_node)
    #plot_performance_std_mse(performance_dir[i], range_number_list[i], error_test_set, target_node)

