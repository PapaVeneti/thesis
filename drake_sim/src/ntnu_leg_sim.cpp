//Simulation
// #include <drake/math/rotation_matrix.h> //to set up first frame
#include "drake/systems/analysis/simulator.h"
#include "ntnu_leg.hpp"


//Visualization
#include "drake/visualization/visualization_config_functions.h" //Needed for AddDefaultVisualization
#include "drake/geometry/meshcat.h" //Access the visualizer (camera, recording etc)
using meshcat_shared_ptr = std::shared_ptr<drake::geometry::Meshcat>; 

//Optional includes
#include <iostream>
#include <fstream> //to export diagram


struct sim_parameters {
  double realtime_rate = 1;
  bool publish_every_time_step = true;
  double sim_time = 5;
  double sim_update_rate = 0.01;
};


int main(int argc, char **argv){

// Ros: node SETUP
#pragma region
// // Node Set up 
// ros::init(argc, argv, "drake simulation");
// ros::NodeHandle n;
// ros::Publisher clock_publisher = n.advertise<rosgraph_msgs::Clock>("/clock",10);

// double loop_freq = 100;
// ros::Rate loop_rate(loop_freq); //HZ

#pragma endregion

// Drake: Plant set up
#pragma region
//0. Builder class to create the system
drake::systems::DiagramBuilder<double> builder; 

//1. Add plant and populate it with the robot instances
auto [plant,scene_graph]  = drake::multibody::AddMultibodyPlantSceneGraph(&builder,mb_time_step);

//1a. Instace of robot body (FOR NOW WORLD)
const drake_rigidBody &robot_base= plant.world_body();

//create config structs for each leg
drake_tfd frleg_TF( drake_rotMat::MakeXRotation(-M_PI/2), drake::Vector3<double>::UnitZ() ); //FR
leg_config config_fr(leg_index::fr, robot_base,frleg_TF);

//1b. Instance of each leg
ntnu_leg leg(builder,plant,config_fr); 
auto fr_leg = plant.GetModelInstanceByName("fr_leg");

//3
//3a. Finish plant
plant.set_discrete_contact_approximation(drake::multibody::DiscreteContactApproximation::kSap);
plant.Finalize();


leg.connect_PID_system(builder,plant); //Connections happen after the plant is finalized

//3b. Add visualization (connect scene_graph and set_up meshcat)
meshcat_shared_ptr mescat_ptr =  std::make_shared<drake::geometry::Meshcat> ();
drake::visualization::AddDefaultVisualization(&builder,mescat_ptr);

//3d. Finish building. Never use builder again
auto diagram = builder.Build(); //Connections before here
#pragma endregion

bool export_graph = true;
// Write graph to file:
#pragma region 
if (export_graph){
std::string GraphString = diagram -> GetGraphvizString(1);
std::string filePath = "graph.dot";

    // Create an output file stream
    std::ofstream outFile(filePath);

    // Check if the file stream is open
    if (outFile.is_open()) {
        // Write the string to the file
        outFile << GraphString;

        // Close the file stream
        outFile.close();

        std::cout << "String successfully written to file: " << filePath << std::endl;
    } else {
        std::cerr << "Error opening the file: " << filePath << std::endl;
    }
}


#pragma endregion

//4. Simulation initialization: 
drake::systems::Simulator sim(*diagram);
sim.set_publish_every_time_step(true); // publish simulation as it happens
sim.set_target_realtime_rate(1);


auto& context            = sim.get_mutable_context();
auto& plant_context      = diagram-> GetMutableSubsystemContext ( plant                       ,&context);
auto& controller_context = diagram-> GetMutableSubsystemContext ( *( leg.get_leg_controller()),&context);


//4. Initialization
// Set initial conditions
Eigen::VectorXd p0(5); 
p0.setZero(); 
plant.SetPositions(&plant_context, fr_leg, p0);
plant.SetVelocities(&plant_context, fr_leg, p0);

// Set desired positions
Eigen::Vector3d des_angles{-M_PI_2,0,0}; 
Eigen::VectorXd qd(6); 
qd<<  des_angles[0],des_angles[1],des_angles[2],0,0,0;
diagram -> get_input_port(leg.get_controller_desired_state_port()).FixValue(&context,qd);;


//6.  Simulate:
sim.Initialize();
double sim_time = 0; 
double sim_update_rate = 0.01; 
assert(sim_update_rate >= mb_time_step);
mescat_ptr->StartRecording();
while( sim_time < 2.5){
    //realtime vis
    sim_time +=sim_update_rate;
    sim.AdvanceTo(sim_time);    
    // (leg).FixValue(&plant_context,qd); //for pid joints
}

//playback
mescat_ptr->PublishRecording();



  return 0;
}