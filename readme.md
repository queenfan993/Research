## Files: hybrid_curve.py, script.py, mkdir.py, clear.py, performance.py, src, readme.md

## Usage:
```
python script.py [performance_matrix]
```
[performance_matrix] can be:
* num_vehicles
* shadowing_effect
* gps_std
* velocity

```
python performance.py
```
generate the figures after simulations done

## Notice:
- Here we use:
* KF_EKF_ECRLB to represent KF_EKF_proposed_ECRLB
* Fisher to represent DCRLB
It's just a changing name for consistency with our thesis.

### hybrid_curve.py
- Simulation code with default values and algorithms.
- error_test_set : the target simulation model 
- variable = sys.argv[1] : variable to test
- run the simulation "TOTAL_EPISODE" times and with "runs" update times 


### script.py
- Script for changing performance variable value and call rss_different_topology_curve.py. This script will call "mkdir.py" and "clear.py" previously to ensure the folders are good and old raw data will be eliminated.

### mkdir.py
- Create "raw" and "figures" in current foloder and then create the performance matrix sub-folders in "raw" and "figures".

### clear.py
- Use rm command to delete the data generated from the simulation.

### src
- A folder includes all source code for our simulation and algorithm.

### performance.py
- Plot the performance.




