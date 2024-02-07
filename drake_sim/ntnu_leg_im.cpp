//robot class
#include "ntnu_leg.hpp"


ntnu_leg::ntnu_leg(
  const bool right_side = true,
  const drake_rigidBody & ParentBody ,
  const drake_tfd & TF_B_MH){

drake::systems::DiagramBuilder<double> builder; 

// Add plant (and scene_graph)
#pragma region

//0. multibody -> out of class
auto [plant,scene_graph]  = drake::multibody::AddMultibodyPlantSceneGraph(&builder,mb_time_step);
drake::multibody::Parser parser(&plant);

//1. Load models
auto fl_leg = parser.AddModels("../ntnu_leg.urdf").at(0); //only one model // this adds one instance

//Add world joint (could be in the urdf instead)
//Correct version 
// drake_tfd worldTF( drake_rotMat::MakeXRotation(-M_PI/2), drake::Vector3<double>::UnitZ() );
// plant.world_body()

const drake::multibody::WeldJoint<double> & weld_joint =  
plant.AddJoint< drake::multibody::WeldJoint >("world_joint",
                                                ParentBody,
                                                TF_B_MH,
                                                plant.GetBodyByName("MH"),
                                                drake_tfd(),
                                                drake_tfd::Identity());


//2. Add actuators
// TODO

//3.  Add bushing element

//4. Add linear spring




}


int main(){
  drake::systems::DiagramBuilder<double> builder; 
  auto [plant,scene_graph]  = drake::multibody::AddMultibodyPlantSceneGraph(&builder,mb_time_step);

  return 0;
}
