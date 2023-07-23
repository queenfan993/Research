from math import *
import random
import numpy as np

from numpy import dot, sum, tile, linalg
from numpy.linalg import inv 
from numpy import *
import matplotlib.pyplot as plt
from math import *
import random


class DynamicPlt:
	
	def __init__(self, OPEN):

		if OPEN:
			plt.figure(figsize=(16,9))
			plt.ion()

	# data_error: [0]:real [1]:kf [2]:ekf [3]:gps
	def error_processing(self, data_error):

		self.data_error = []
		for i in range(len(data_error)):
			self.data_error.append(data_error[i])

	# data_pos: [0]:real [1]:kf [2]:ekf [3]:gps
	def pos_processing(self,data_pos):

		self.data_pos = []
		for i in range(len(data_pos)):
			self.data_pos.append(data_pos[i])

	def anchors_processing(self,nodes):

		self.anchors = nodes

	def particles_processing(self, particles):
		self.p = particles

	def plot_dynamic(self):

		dt = 0.1
		#pf_loc = array(self.data_pos[4])
		GPS_measurements = array(self.data_pos[3])
		ekf_loc = array(self.data_pos[2])
		kf_loc = array(self.data_pos[1])
		real_loc = array(self.data_pos[0])

		x_list = []
		y_list = []
		#for i in range(len(self.p)):
		#	x,y,o = self.p[i].getposition()
		#	x_list.append(x)
		#	y_list.append(y)

		landmarks_x, landmarks_y = [], []
		for i in range(len(self.anchors)):
		    landmarks_x.append(self.anchors[i].getposition()[0])
		    landmarks_y.append(self.anchors[i].getposition()[1])

		plt.subplot(2, 1, 1)
		plt.cla()
		plt.plot(real_loc[:, 0], real_loc[:, 1], c='g', label="REAL_ROUTE")
		plt.scatter(landmarks_x, landmarks_y, c='y', s=300, label="OTHER_VEHICLES")
		plt.scatter(x_list, y_list, c='m', label="PF")
		plt.plot(ekf_loc[:, 0], ekf_loc[:, 1], c='r', label="EKF_ROUTE")
		plt.plot(kf_loc[:, 0], kf_loc[:, 1], c='b', label="KF_ROUTE")
		#plt.plot(pf_loc[:, 0], pf_loc[:, 1], c='k', label="PF_ROUTE")
		plt.scatter(GPS_measurements[:, 0], GPS_measurements[:, 1], c='k', label="gps")
		plt.legend( loc=2, bbox_to_anchor=(0.9, 1.0))

		plt.subplot(2, 1, 2)
	
		plt.cla()
		plt.ylim(0, 20)
		#my_new_list = [i * dt for i in range(len(self.data_error[4]))]
		#plt.plot(my_new_list, self.data_error[4], c='k', label="PF_error")

		my_new_list = [i * dt for i in range(len(self.data_error[2]))]
		plt.plot(my_new_list, self.data_error[2], c='r', label="EKF_error")

		my_new_list = [i * dt for i in range(len(self.data_error[1]))]
		plt.plot(my_new_list, self.data_error[1], c='b', label="KF_error")
		
		plt.legend( loc=2, bbox_to_anchor=(0.9, 1.0))

		plt.pause(0.001)

	def close_plt(self):

		plt.ioff()
		plt.show()
