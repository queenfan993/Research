import random
from math import log10, sqrt

class TDOA_MODEL:

	def __init__(self, c, syn_error):

		self.c = c

		self.syn_error = syn_error

	def genTDOA_dist(self, dist1, dist2):
		return dist1/self.c - dist2/self.c + random.gauss(0.0, self.syn_error)

	def genTDOA_dist_no_error(self, dist):
		return dist1/self.c - dist2/self.c

	def genTDOA_coodinate(self, ref_x1, ref_y1, ref_x2, ref_y2, x, y):
		dist1 = self.distance(x, y, ref_x1, ref_y1)
		dist2 = self.distance(x, y, ref_x2, ref_y2)
		
		return self.genTDOA_dist(dist1,dist2)

	def distance(self, x, y, x_t, y_t):
		return sqrt((x-x_t)**2 + (y-y_t)**2)

	def __repr__(self):
		return 'TOA PROPERTY[t0=%.6s c=%.6s syn_error=%.6s]' % (str(self.t0), str(self.c), str(self.syn_error))


