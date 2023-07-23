import numpy as np
import shutil
import sys
from subprocess import call

#call("python clear.py", shell=True)

'''
USAGE

python script.py PERFORMANCE_TARGET 

'''

#==============default_set================

TARGET_PYTHON = "hybrid_curve.py"

variable = sys.argv[1]

range_number = 0
'''
if variable == "num_vehicles":
    range_number = 30
elif variable == "shadowing_effect":
    range_number = 20
elif variable == "gps_accuracy":
    range_number = 30
elif variable == "velocity":
    range_number = 40
elif variable == "smart_prob":
    range_number = 11
elif variable == "motion_noise":
    range_number = 11
elif variable == "seperate_space":
    range_number = 12
else:
    print "Wrong for variable!"
    sys.exit()
'''
if variable == "num_vehicles":
    range_number = 2
elif variable == "shadowing_effect":
    range_number = 7+1
elif variable == "gps_accuracy":
    range_number = 7+1
elif variable == "syn_error":
    range_number = 7+1    
elif variable == "gps_std":
    range_number = 6
elif variable == "world_size": 
    range_number = 10   
elif variable == "velocity":
    range_number = 10
elif variable == "smart_prob":
    range_number = 11
elif variable == "motion_noise":
    range_number = 11
elif variable == "seperate_space":
    range_number = 12
else:
    print ("Wrong for variable!")
    sys.exit()

#=========================================
call("python clear.py {0}".format(variable), shell=True)
call("python mkdir.py {0}".format(variable), shell=True)


if variable == "velocity":

	for i in range(range_number):
		#print variable," ", i
		call("python {2} {0} {1}".format(variable, i*5, TARGET_PYTHON) 
		    , shell=True)

elif variable == "syn_error":

	for i in range(range_number):
		#print variable," ", i*0.1
		call("python {2} {0} {1}".format(variable, (i+1)*10*pow(10,-8), TARGET_PYTHON) 
		    , shell=True)		
		
elif variable == "gps_std":

	for i in range(range_number):
		#print variable," ", i*0.1
		call("python {2} {0} {1}".format(variable, round((i+1)*5,1), TARGET_PYTHON) 
		    , shell=True)			

elif variable == "smart_prob":

	for i in range(range_number):
		#print variable," ", i*0.1
		call("python {2} {0} {1}".format(variable, round(i*0.1,1), TARGET_PYTHON) 
		    , shell=True)

elif variable == "world_size":

	for i in range(range_number):
		#print variable," ", i*0.1
		call("python {2} {0} {1}".format(variable, (i+1)*50, TARGET_PYTHON) 
		    , shell=True)		

elif variable == "motion_noise":

	for i in range(range_number):
		#print variable," ", i*0.1
		call("python {2} {0} {1}".format(variable, i*0.1, TARGET_PYTHON) 
		    , shell=True)
elif variable == "seperate_space":

	for i in range(range_number):
		#print variable," ", i*0.1
		call("python {2} {0} {1}".format(variable, (i+1)*5., TARGET_PYTHON) 
		    , shell=True)

elif variable == "num_vehicles":

	for i in range(range_number):
		#print variable," ", i*0.1
		call("python {2} {0} {1}".format(variable, (i+1)*5, TARGET_PYTHON) , shell=True)

else:

	for i in range(range_number):
		#print variable," ", i+1
		if i == 0:
			call("python {2} {0} {1}".format(variable, i+1, TARGET_PYTHON) , shell=True)
		else:
			call("python {2} {0} {1}".format(variable, i*3, TARGET_PYTHON) , shell=True)







