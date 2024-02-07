# Set the minimum required version of CMake
cmake_minimum_required(VERSION 3.10)
set(CMAKE_FIND_DEBUG_MODE ON)
enable_language(CXX) #for find threads -> dependency of drake

### 


#for mit examples glfags is a dependency
# find_package(gflags REQUIRED)
find_package(drake CONFIG REQUIRED PATHS /opt/drake)
find_package(Eigen3 REQUIRED PATHS /usr/lib/)
# find_package(graphviz REQUIRED)

# Set the project name and version
project(ntnu_sim VERSION 1.0)

# Add the executable target
add_executable(multibody_leg multibody_leg.cpp)
add_executable(ntnu_leg ntnu_leg.cpp)




# Link the target with the necessary libraries
target_link_libraries(multibody_leg drake::drake )
target_link_libraries(ntnu_leg drake::drake )
