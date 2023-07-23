import random
from math import log10, sqrt

class CHANNEL:

	def __init__(self, tx_power, ref_pathLoss, pathLossExp, shadowing, EnergyDetectionThreshold):

		#Path loss exponent
		self.PLE = pathLossExp

		#Path loss at reference distance. Here we always use d=1m as our reference distance
		self.PL0 = ref_pathLoss

		#Transmit power
		self.Txpower = tx_power
		
		#The standard deviation of shadowing effect
		self.shadowing = shadowing

		#The energy of a received signal should be higher than this threshold (dbm) to 
		#allow the PHY layer to detect the signal.
		self.EnergyDetectionThreshold = EnergyDetectionThreshold

	def genRSS_dist(self, dist):

		return self.Txpower - self.PL0 - 10*self.PLE*log10(dist) + random.gauss(0.0, self.shadowing)

	def genRSS_dist_no_shadowing(self, dist):

		return self.Txpower - self.PL0 - 10*self.PLE*log10(dist) 

	def genRSS_coodinate(self, ref_x, ref_y, x, y):

		dist = self.distance(x, y, ref_x, ref_y)

		return self.genRSS_dist(dist)

	def get_EDT(self):
		return self.EnergyDetectionThreshold

	def set_EDT(self, threshold):
		self.EnergyDetectionThreshold = threshold

	def get_MaxRange(self):
		return pow(10, (self.Txpower - self.PL0 - self.EnergyDetectionThreshold)/(10*self.PLE))

	def distance(self, x, y, x_t, y_t):
		return sqrt((x-x_t)**2 + (y-y_t)**2)

	def __repr__(self):
		return 'CHANNEL PROPERTY[Txpower=%.6s PL0=%.6s PLE=%.6s Shadowing=%.6s]' % (str(self.Txpower), str(self.PL0), str(self.PLE), str(self.shadowing))


#test_channel = CHANNEL(20,65,2.8,6.0)
#print test_channel.genRSS_dist(10)
