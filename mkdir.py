from subprocess import call
import sys

directory = ["raw", "figures"]

for i in directory:
	
	call("mkdir {0}".format(i), shell=True)

performance_dir = ["num_vehicles", "shadowing_effect", "gps_accuracy", "gps_std", "velocity", "smart_prob","motion_noise"]
performance_matrix = sys.argv[1]

call("mkdir raw\\{0}".format(performance_matrix), shell=True)
call("mkdir figures\\{0}".format(performance_matrix), shell=True)

