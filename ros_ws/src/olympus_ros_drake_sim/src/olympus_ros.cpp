//Simulation
#include "drake/systems/analysis/simulator.h"
#include "olympus.hpp"

//Visualization
#include "drake/visualization/visualization_config_functions.h" //Needed for AddDefaultVisualization

// ros interface
#include "drake_ros_interface.hpp"

//Optional includes
#include <iostream>

//flags
#define get_graph false
#define mb_time_step 0.0002 //in seconds!!!! -> must be as small as the simulation
#define integrator_time_step 1e-3
#define simulation_update_rate 1e-2 //GCD ( controller update rate, sensor update rate) (GREATER THAN mb_timestep)

struct sim_parameters {
  double realtime_rate = 1;
  bool publish_every_time_step = true;
  double sim_time = 5;
  double sim_update_rate = 0.01;
};

int main(int argc, char **argv){

// Ros: node SETUP
#pragma region
// Node Set up 
ros::init(argc, argv, "olympus_ros_simulation");
ros::NodeHandle n;

double loop_freq = 1/simulation_update_rate;
ros::Rate loop_rate(loop_freq); //HZ

#pragma endregion

// Drake: Plant set up
#pragma region
//0. Builder class to create the system
drake::systems::DiagramBuilder<double> builder; 

olympus robot(builder,mb_time_step); //robot plant creation

//2b. Add visualization (connect scene_graph and set_up meshcat)
meshcat_shared_ptr mescat_ptr =  std::make_shared<drake::geometry::Meshcat> ();
drake::visualization::AddDefaultVisualization(&builder,mescat_ptr);
mescat_ptr->SetTransform("/Helper symbols/Points",drake_tfd(Eigen::Vector3d({0,0,1-0.0056}))); //This must be updated with the current transform of the body

//2c. Finish building. Never use builder again
auto diagram = builder.Build(); //Connections before here


if (get_graph){ get_system_graph(diagram.get());  }
#pragma endregion

//Simulation Set Up: 
#pragma region
drake::systems::Simulator sim(*diagram);

// simulation parameters
sim.set_publish_every_time_step(false); // MUST BE FALSE, ELSE OUT OF MEMORY VERY FAST
// sim.set_target_realtime_rate(2);  //no target, run as fast as possible
sim.get_mutable_integrator().set_maximum_step_size(integrator_time_step);
double sim_update_rate = simulation_update_rate; 
assert(sim_update_rate >= mb_time_step);
#pragma endregion

//Initialization
#pragma region
auto& context            = sim.get_mutable_context();
auto& plant_context      = diagram-> GetMutableSubsystemContext ( robot.get_plant()                         ,&context);

robot.default_init(*diagram,context);


#pragma endregion

// Set up interface:
#pragma region 

drake_ros_elements leg_interface_elements(sim,4,4,20);
leg_interface_elements.system_input_ports_  = robot.get_input_ports(*diagram);
leg_interface_elements.system_output_ports_ = robot.get_output_ports(*diagram);
leg_interface_elements.joint_names_ = robot.get_joint_names();
leg_interface_elements.meshcat_ptr = mescat_ptr;

if (mescat_ptr == nullptr)
{
  std::cout <<"meshcat is null" <<std::endl;
}

simInterface simInterface(leg_interface_elements);
simInterface.set_monitor();


#pragma endregion


// Simulate:
sim.Initialize();
double sim_time = 0; 
while( ros::ok() ){ 
  simInterface.encoderUpdate();

  //simulation takes place:
  sim_time +=sim_update_rate;
  sim.AdvanceTo(sim_time);   


  ros::spinOnce(); //process callbacks
  if (!loop_rate.sleep()){ROS_DEBUG("Simulation is slower than realtime");} //returns false 
}

  return 0;
}