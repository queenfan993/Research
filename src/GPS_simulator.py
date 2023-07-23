import numpy as np
from numpy import array
from math import *

# According to US GPS performance report, we use: [Position Accuracy <= 9m 95% Horizontal, global average]
# to model out GPS simulator.

# I assume the coordinate x and y are Gaussian distribution and independent for each other.
# The problem is what is the distribution for d = sqrt(x^2 + y^2) without the actual standard deviation of x and y?
# Fortunately, the Rayleigh distribution is sufficient and we use its Quentile function to meet the US performance report.
# Rayleigh distribution: https://en.wikipedia.org/wiki/Rayleigh_distribution
'''
def GPS_model(accuracy, quentile, number_samples, true_loc, bias):

	target_distance_std = compute_std_given_Accuracy_Quentile(accuracy, quentile)
	cov = [[ target_distance_std**2, 0], [0, target_distance_std**2]]
	x, y = np.random.multivariate_normal(true_loc, cov, number_samples).T
	return x+bias[0], y+bias[1]
'''
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
#number_samples = 1000
#
#true_loc = [0, 0]
#

#x,y = GPS_model(9,0.95,1,true_loc, array([-5.0, 5.0]).T)
#
#dist = []
#pdf = []
#counter = 0.
#for i in range(number_samples):
#	dist.append( sqrt((x[i] - true_loc[0])**2 + (y[i] - true_loc[1])**2 ))
#	if dist[i] <= 9.:
#		counter += 1
#	pdf.append(Gaussian(0, 4.5, dist[i] ) )
#
#
#print counter/number_samples

#plt.scatter(dist, x )

#plt.plot(x, y, 'x')
#plt.axis('equal')


#plt.show()


