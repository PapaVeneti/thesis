# Set the minimum required version of CMake
cmake_minimum_required(VERSION 3.10)
set(CMAKE_FIND_DEBUG_MODE ON)
enable_language(CXX) #for find threads -> dependency of drake


#for mit examples glfags is a dependency
# find_package(gflags REQUIRED)
find_package(drake CONFIG REQUIRED PATHS /opt/drake)
# find_package(Eigen3 REQUIRED PATHS /usr/lib/)

# Set the project name and version
project(drake_sim_examples VERSION 1.0)


# Targets:
# Add the executable target
add_executable(ntnu_leg_sim    src/ntnu_leg_sim.cpp)
add_executable(robot_sim       src/olympus_sim.cpp)
add_executable(controller_test src/test2.cpp)

include_directories(include)

# Linking:
# Add dependencies
add_dependencies(ntnu_leg_sim    ntnu_leg)
add_dependencies(robot_sim       robot)
add_dependencies(controller_test position_controller)

# Link the target with the necessary libraries
target_link_libraries(ntnu_leg_sim    drake::drake ntnu_leg position_controller)
target_link_libraries(robot_sim       drake::drake robot ntnu_leg )
target_link_libraries(controller_test position_controller )

#include headers

