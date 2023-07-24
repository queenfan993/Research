import numpy as np
from numpy import array
from math import *


class GPS_MODEL:

	def __init__(self, accuracy, quentile, number_samples):

		self.target_distance_std = self.compute_std_given_Accuracy_Quentile(accuracy, quentile)
		self.number_samples = number_samples
		self.cov = [[ self.target_distance_std**2, 0], [0, self.target_distance_std**2]]

	def genGPS_coodinate(self, x, y):
		
		tmp_x, tmp_y = np.random.multivariate_normal((x, y), self.cov, self.number_samples).T
		return tmp_x, tmp_y

	def genGPS_coordinate_bias(self, x, y, bias):
		
		tmp_x, tmp_y = self.genGPS_coodinate(x, y)
		return tmp_x+bias[0], tmp_y+bias[1]

	def getGPS_std(self):

		return self.target_distance_std

	def compute_std_given_Accuracy_Quentile(self, accuracy, quentile):

		return accuracy/(sqrt(-2*log(1-quentile)))

	def genGPS_cov(self, x, y, cov):

		tmp_x, tmp_y = np.random.multivariate_normal((x, y), cov, 1).T
		return tmp_x, tmp_y 

	# for given GPS std
	def change_GPSstd(self, GPS_std):
		self.target_distance_std = GPS_std
		self.cov = [[ self.target_distance_std**2, 0], [0, self.target_distance_std**2]]


