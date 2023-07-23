from subprocess import call
import sys

performance_matrix = sys.argv[1]
directory = ["raw", "figures"]


call("rm -rf topology", shell=True)



for i in directory:
	call("rm -rf {0}/{1}".format(i, performance_matrix), shell=True)

