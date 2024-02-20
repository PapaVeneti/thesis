#  Leg Planner
## Running the scripts: 
1. First source `env.sh`. (Lines 47,48 may need to be modified if acados is not installed in the home directory)
2. Launch matlab
3. To run the scripts first run `a_initialize_workspace` script, to add the directories of the workspace to the matlab path. 


## Structure
- The OPC is created in the `leg_ocp.m` script. 
- The ntnu model is defined in the `leg_model` function. (Dynamics are defined in section 4).
- Currently is solves 200 cases (with 2 different references, prints out some statistics and plots the final solution). 


# Drake Sim
Requirements:
C++ standard: C++17


## How to run drake simulations from VScode 
TODO: make it an install script.

1. symlink to the urdfs: (In the `drake_sim/` directory)
```bash
$  ln -s $(pwd)/drake_sim/urdf $(pwd)/drake_sim/build/examples/urdf
```
- Now simulations (after building) can be executed from either the `drake_sim/build` directory or the `drake_sim` directory. 

2.  Also, you can use the [VsCode Cmake Tools](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cmake-tools) to build and launch the simulations from within vscode. 


# Ros_ws
To run the simulations

1. From root directory (not home)
```bash
$ ln -s  $(pwd)/drake_sim/urdf $(pwd)/ros_ws/src/olympus_control/urdf
```