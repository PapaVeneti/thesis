#  Leg Planner
## Running the scripts: 
1. First source `env.sh`. (Lines 47,48 may need to be modified if acados is not installed in the home directory)
2. Launch matlab
3. To run the scripts first run `a_initialize_workspace` script, to add the directories of the workspace to the matlab path. 


## Structure
- The OPC is created in the `leg_ocp.m` script. 
- The ntnu model is defined in the `leg_model` function. (Dynamics are defined in section 4).
- Currently is solves 200 cases (with 2 different references, prints out some statistics and plots the final solution). 