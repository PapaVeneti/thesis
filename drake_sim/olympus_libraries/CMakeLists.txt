# Set the minimum required version of CMake
cmake_minimum_required(VERSION 3.10)
set(CMAKE_FIND_DEBUG_MODE ON)
enable_language(CXX) #for find threads -> dependency of drake


#for mit examples glfags is a dependency
# find_package(gflags REQUIRED)
find_package(drake CONFIG REQUIRED PATHS /opt/drake)
find_package(Eigen3 REQUIRED PATHS /usr/lib/)
# find_package(graphviz REQUIRED)

# Set the project name and version
project(drake_sim VERSION 1.0)


# Targets:
# 1. Add the library target
add_library(ntnu_leg            src/ntnu_leg.cpp)
add_library(robot               src/olympus.cpp)
add_library(position_controller src/position_controller.cpp)

# 2. Add the executable target
# add_executable(multibody_leg multibody_leg.cpp) #deprecated
# add_executable(ntnu_leg_sim src/ntnu_leg_sim.cpp)
# add_executable(robot_sim src/olympus_sim.cpp)
# add_executable(controller_test test2.cpp)

# Linking:
# Add dependencies
add_dependencies(robot ntnu_leg)

# add header files -> PUBLIC to be available for all targets that link againts ntnu_leg
target_include_directories(ntnu_leg            PUBLIC include) 
target_include_directories(robot               PUBLIC include) 
target_include_directories(position_controller PUBLIC include) 


# Link the target with the necessary libraries
target_link_libraries(ntnu_leg drake::drake )
target_link_libraries(robot drake::drake ntnu_leg)

