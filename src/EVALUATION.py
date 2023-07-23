import numpy as np

class Evaluation:

	def __init__(self, ID, name , epoch, timeSlot):

		self.epoch = epoch
		self.timeSlot = timeSlot
		self.name = name
		self.ID = ID

		self.hist_error = []
		for i in range(epoch):
			self.hist_error.append([])

	def push_error_back(self, epoch, error):

		self.hist_error[epoch].append(error)

	def outText(self, dir_path, epoch):

	    #text_file = open(dir_path+str(self.name)+"_"+str(self.ID)+str(title)+".txt", "a")
	    text_file = open(dir_path, "a")
	    data = self.hist_error[epoch]
	    text_file.write(' '.join(str(i) for i in data))
	    text_file.write("\n")
	    text_file.close()

	def gethistMean(self):
		tmp_hist = np.array(self.hist_error)
		return np.mean(tmp_hist, axis=0) 
	def gettotalerror(self, index):
		return sum(self.hist_error[index])
'''
tmp = Evaluation(54,"error_distance",5,100)
'''



