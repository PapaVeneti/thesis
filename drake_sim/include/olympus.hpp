#include "ntnu_leg.hpp"
#include "drake/multibody/tree/ball_rpy_joint.h"

//Visualization
#include "drake/visualization/visualization_config_functions.h" //Needed for AddDefaultVisualization
#include "drake/geometry/meshcat.h" //Access the visualizer (camera, recording etc)
using meshcat_shared_ptr = std::shared_ptr<drake::geometry::Meshcat>; 


class olympus {
  public:		

    olympus(drake_builder & builder, meshcat_shared_ptr meshcat_ptr, const double & time_step_ );

    void set_plant();
    void add_body(drake_plant & plant);
    void add_legs();
  private:
  double time_step = 0.002;
  // drake_plant & plant;
  // drake::geometry::SceneGraph<double> & scene_graph;
  drake::multibody::ModelInstanceIndex body_index;

};


void olympus::add_body(drake_plant & plant){
  //Add the body urdf
  drake::multibody::Parser parser(&plant);
  body_index = parser.AddModels("../urdf/quadruped_base.urdf").at(0);

  // Connect to world using a ball rpy joint (Connect {P} and {M})
  drake_tfd T_WP( Eigen::Translation3d{0,0,1}); //frame {P} in the {W} frame
  drake_tfd T_BM( plant.GetBodyByName("quadruped_base").default_com() ) ; //frame {M} in the {B} frame

  const drake::multibody::BallRpyJoint<double> & base_ball_joint =
  plant.AddJoint<drake::multibody::BallRpyJoint> ( "base ball joint",
                                                    plant.world_body(), 
                                                    T_WP,                                                   
                                                    plant.GetBodyByName("quadruped_base"),
                                                    T_BM );


}


olympus::olympus(drake_builder & builder,meshcat_shared_ptr meshcat_ptr, const double & time_step_): time_step(time_step_)
{

//1. Add plant and populate it with the robot instances
auto [plant,scene_graph]  = drake::multibody::AddMultibodyPlantSceneGraph(&builder,time_step);

//1a. Instace of robot body (FOR NOW WORLD)
const drake_rigidBody &robot_base= plant.world_body();
add_body(plant);


//TEMPORARY: create config structs for each leg
drake_tfd frleg_TF( drake_rotMat::MakeXRotation(-M_PI/2), drake::Vector3<double>::UnitZ() ); //FR
// drake_tfd rrleg_TF( drake_rotMat::MakeXRotation(-M_PI/2), drake::Vector3<double>{-1,0,1});

leg_config config_fr(leg_index::fr, robot_base,frleg_TF);

//1b. Instance of each leg
// ntnu_leg(builder,plant,config_fr); 
// auto fr_leg = plant.GetModelInstanceByName("fr_leg");

//3
//3a. Finish plant
plant.set_discrete_contact_approximation(drake::multibody::DiscreteContactApproximation::kSap);
plant.Finalize();

//3b. Add visualization (connect scene_graph and set_up meshcat)
drake::visualization::AddDefaultVisualization(&builder,meshcat_ptr);


//3d. Finish building. Never use builder again
// auto diagram = builder.Build(); //Connections before here

};