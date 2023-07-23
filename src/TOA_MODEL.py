import random
from math import log10, sqrt

class TOA_MODEL:

	def __init__(self, t0, c, syn_error):

		#transmit time at neighbor 
		self.t0 = t0

		self.c = c
		
		#The standard deviation of synchronization error
		self.syn_error = syn_error

	
	def genTOA_dist(self, dist):
		return dist/self.c + self.t0 + random.gauss(0.0, self.syn_error)

	def genTOA_dist_no_error(self, dist):

		return dist/self.c + self.t0 

	def genTOA_coodinate(self, ref_x, ref_y, x, y):

		dist = self.distance(x, y, ref_x, ref_y)

		return self.genTOA_dist(dist)

	

	def distance(self, x, y, x_t, y_t):
		return sqrt((x-x_t)**2 + (y-y_t)**2)

	def __repr__(self):
		return 'TOA PROPERTY[t0=%.6s c=%.6s syn_error=%.6s]' % (str(self.t0), str(self.c), str(self.syn_error))


