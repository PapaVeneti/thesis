//Simulation
#include <drake/math/rotation_matrix.h> //to set up first frame
#include "drake/systems/analysis/simulator.h"

//Visualization
#include "drake/visualization/visualization_config_functions.h" //Needed for AddDefaultVisualization
#include "drake/geometry/meshcat.h" //Access the visualizer (camera, recording etc)
using meshcat_shared_ptr = std::shared_ptr<drake::geometry::Meshcat>; 

#include <iostream>
#include "ntnu_leg.hpp"

int main(){

//0. Builder class to create the system
drake::systems::DiagramBuilder<double> builder; 

//1. Add plant with corresponding scene graph
auto [plant,scene_graph]  = drake::multibody::AddMultibodyPlantSceneGraph(&builder,mb_time_step);

//TEMPORARY: these paramters  should be defined for each leg, 
drake_tfd frleg_TF( drake_rotMat::MakeXRotation(-M_PI/2), drake::Vector3<double>::UnitZ() );
// drake_tfd rrleg_TF( drake_rotMat::MakeXRotation(-M_PI/2), drake::Vector3<double>{-1,0,1});
const drake_rigidBody &robot_base= plant.world_body();

//2. Instance of each leg
ntnu_leg(builder,plant,leg_index::fr, robot_base,frleg_TF);
// ntnu_leg(builder,plant,leg_index::rr, robot_base,rrleg_TF);

//3
//3a. Finish plant
plant.set_discrete_contact_approximation(drake::multibody::DiscreteContactApproximation::kSap);
plant.Finalize();

//3b. Add visualization (connect scene_graph and set_up meshcat)
meshcat_shared_ptr mescat_ptr =  std::make_shared<drake::geometry::Meshcat> ();
drake::visualization::AddDefaultVisualization(&builder,mescat_ptr);

//3c. Finish building. Never use builder again
auto diagram = builder.Build();

//4. Simulation initialization: 
drake::systems::Simulator sim(*diagram);

auto& context = sim.get_mutable_context();
auto& plant_context = plant.GetMyMutableContextFromRoot(&context);


//4. Initialization
Eigen::Vector3d des_angles{-M_PI_2,0,0}; //1 desired -> opposite from my code

// Set initial conditions
Eigen::VectorXd p0(5); 
p0 << des_angles[0],des_angles[1],0,des_angles[2],0;

auto fr_leg = plant.GetModelInstanceByName("fr_leg");
// plant.SetPositions(
//         &plant_context,
//         fr_leg,
//         p0);

Eigen::VectorXd qd(6); 
qd<<  des_angles[0],des_angles[1],des_angles[2],0,0,0;
plant.get_desired_state_input_port(fr_leg).FixValue(&plant_context,qd); //for pid joints

// std::cout<< plant.GetJointActuatorIndices() <<std::endl;
for (auto ind:plant.GetJointActuatorIndices(fr_leg) ){
  std::cout<< plant.get_joint_actuator(ind).name() <<std::endl;
}







// Simulate:
sim.set_publish_every_time_step(true); // publish simulation as it happens
sim.Initialize();
sim.set_target_realtime_rate(1);

double sim_time = 0; 
double sim_update_rate = 0.01; 
assert(sim_update_rate >= mb_time_step);
mescat_ptr->StartRecording();
while( sim_time < 5){
    //realtime vis
    sim_time +=sim_update_rate;
    sim.AdvanceTo(sim_time);    
    // (leg).FixValue(&plant_context,qd); //for pid joints

}


//playback
mescat_ptr->PublishRecording();



  return 0;
}