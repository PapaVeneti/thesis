//robot class
#include "ntnu_leg.hpp"

//Simulation
#include <drake/math/rotation_matrix.h> //to set up first frame
#include "drake/systems/analysis/simulator.h"


//Visualization
#include "drake/visualization/visualization_config_functions.h" //Needed for AddDefaultVisualization
#include "drake/geometry/meshcat.h" //Access the visualizer (camera, recording etc)
using meshcat_shared_ptr = std::shared_ptr<drake::geometry::Meshcat>; 


ntnu_leg::ntnu_leg(
  drake_builder &builder,
  drake_plant & created_plant,
  bool right_side,
  const drake_rigidBody & ParentBody ,
  const drake_tfd & TF_B_MH): plant(created_plant) {

//1. Load leg model
// TODO: load depending on side.
drake::multibody::Parser parser(&plant);
auto fl_leg = parser.AddModels("../ntnu_leg.urdf").at(0); //only one model // this adds one instance

const drake::multibody::RevoluteJoint<double> & base_joint =  
plant.AddJoint< drake::multibody::RevoluteJoint >("jointMH",
                                                ParentBody,
                                                TF_B_MH,
                                                plant.GetBodyByName("MH"),
                                                drake_tfd(),
                                                Eigen::Vector3d{1,0,0}, //axis of joint in 
                                                -M_PI_2, // lower position limit
                                                M_PI_2); // upper position limit



//2. Add actuators
// TODO

//3.  Add bushing element

//4. Add linear spring


}


int main(){

//0. Builder class to create the system
drake::systems::DiagramBuilder<double> builder; 

//1. Add plant with corresponding scene graph
auto [plant,scene_graph]  = drake::multibody::AddMultibodyPlantSceneGraph(&builder,mb_time_step);

//TEMPORARY: these paramters  should be defined for each leg, 
drake_tfd frleg_TF( drake_rotMat::MakeXRotation(-M_PI/2), drake::Vector3<double>::UnitZ() );
const drake_rigidBody &robot_base= plant.world_body();

//2. Instance of each leg
ntnu_leg(builder,plant,true, robot_base,frleg_TF);

//3. Finish plant
plant.set_discrete_contact_approximation(drake::multibody::DiscreteContactApproximation::kSap);
plant.Finalize();


//4. Add visualization (connect scene_graph and set_up meshcat)
meshcat_shared_ptr mescat_ptr =  std::make_shared<drake::geometry::Meshcat> ();
drake::visualization::AddDefaultVisualization(&builder,mescat_ptr);

//4. Finish building. Never use builder again
auto diagram = builder.Build();

//5. Simulate: 
drake::systems::Simulator sim(*diagram);

// Initialization:
auto& context = sim.get_mutable_context();
auto& plant_context = plant.GetMyMutableContextFromRoot(&context);



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
    // (fl_leg).FixValue(&plant_context,qd); //for pid joints

}


//playback
mescat_ptr->PublishRecording();



  return 0;
}
