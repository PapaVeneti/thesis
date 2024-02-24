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
#include "drake_helpers.hpp"
#include "position_controller.hpp"

#define mb_time_step 0.0002

//for collisions:
// #include <drake/geometry/collision_filter_declaration.h>
// #include <drake/geometry/scene_graph_inspector.h>
// #include <drake/geometry/geometry_set.h>


struct sim_parameters {
  double realtime_rate = 1;
  bool publish_every_time_step = true;
  double sim_time = 5;
  double sim_update_rate = 0.01;
};


int main(int argc, char **argv){


// Drake: Plant set up
#pragma region
//0. Builder class to create the system
drake::systems::DiagramBuilder<double> builder; 

//1. Add plant and populate it with the robot instances
auto [plant,scene_graph]  = drake::multibody::AddMultibodyPlantSceneGraph(&builder,mb_time_step);

//1a. Instace of robot body (FOR NOW WORLD)
const drake_rigidBody &robot_base= plant.world_body();

  //create config structs for each leg
// drake_tfd frleg_TF( drake_rotMat::MakeXRotation(-M_PI/2), drake::Vector3<double>::UnitZ() ); //FR
drake_rotMat R_W_FR = drake_rotMat::MakeXRotation(-M_PI_2);
drake_rotMat R_FR_RR = drake_rotMat::MakeZRotation(-M_PI);
drake_tfd frleg_TF( R_W_FR* R_FR_RR, drake::Vector3<double>::UnitZ() ); //RR
leg_config config_rr(leg_index::rr, robot_base,frleg_TF);

//1b. Instance of each leg
ntnu_leg leg(builder,plant,config_rr); 
auto fr_leg = leg.get_leg_model_instance();


//1c. Finish plant
plant.set_discrete_contact_approximation(drake::multibody::DiscreteContactApproximation::kSap);
plant.Finalize();

auto collision_manager = scene_graph.collision_filter_manager();
collision_manager.Apply(
  drake::geometry::CollisionFilterDeclaration().
  ExcludeWithin(leg.get_shanks_collision())
);

//2. Wire up diagram
leg.connect_PID_system(builder,plant); //Connections happen after the plant is finalized

//2b. Add visualization (connect scene_graph and set_up meshcat)
meshcat_shared_ptr mescat_ptr =  std::make_shared<drake::geometry::Meshcat> ();
drake::visualization::AddDefaultVisualization(&builder,mescat_ptr);

//2c. Finish buAddDefaultVisualization
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
// Eigen::Vector3d des_angles{-M_PI_2,0,0}; 
Eigen::Vector3d des_angles{0,1.5,-1.5}; 
Eigen::VectorXd qd(6); 
qd<<  des_angles[0],des_angles[1],des_angles[2],0,0,0;
diagram -> get_input_port(leg.get_controller_desired_state_port()).FixValue(&context,qd);

//Visualize the postion
position_controller pos_controller(2) ;
pos_controller.q = p0;
pos_controller.DK(des_angles);

drake_tfd T_W_MH(drake::math::RollPitchYawd(-M_PI_2,M_PI,0),{0,0,1});
drake_tfd T_MH_P(drake::math::RollPitchYawd(0,0,0),pos_controller.DK(des_angles));
drake_tfd T_W_P = T_W_MH*T_MH_P; 
// AddPoint(T_W_P,mescat_ptr.get(),"Goal Position");

// 6.  Simulate:
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

// // auto& context2            = sim.get_mutable_context();
// // Eigen::Vector3d p_W = {0.087722,0.13204,0.63377};
// // drake_tfd T_W_Pnew(drake::math::RollPitchYawd(0,0,0),p_W);
// // AddPoint(T_W_Pnew,mescat_ptr.get(),"New Goal Position",drake::geometry::Rgba(1,0,0,1));
// // Eigen::Vector3d new_goal = T_W_MH.inverse()*p_W ; 
// // std::cout << "New goal position in {MH} frame is:" << std::endl;
// // std::cout << new_goal << std::endl;
// // des_angles = pos_controller.IK(new_goal);
// // qd<<  des_angles[0],des_angles[1],des_angles[2],0,0,0;
// // diagram -> get_input_port(leg.get_controller_desired_state_port()).FixValue(&context2,qd);

// // while( sim_time < 3){
// //     //realtime vis
// //     sim_time +=sim_update_rate;
// //     sim.AdvanceTo(sim_time);    
// //     // (leg).FixValue(&plant_context,qd); //for pid joints
// // }
// //playback
mescat_ptr->PublishRecording();



  return 0;
}