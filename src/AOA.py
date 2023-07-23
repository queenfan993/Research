import random
import math
from math import log10, sqrt

class AOA_MODEL:

	def __init__(self, error):

		self.angle_error = error

	def genAOA_coodinate(self, ref_x1, ref_y1, x, y):
		
		return atan((y-ref_y_1)/(x-ref_x1)) + random.gauss(0.0, self.angle_error*2)

	def distance(self, x, y, x_t, y_t):
		return sqrt((x-x_t)**2 + (y-y_t)**2)

	def __repr__(self):
		return 'AOA PROPERTY[t0=%.6s c=%.6s syn_error=%.6s]' % (str(self.angle_error))