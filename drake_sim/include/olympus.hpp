#include "ntnu_leg.hpp" //For Legs
#include "drake/multibody/tree/ball_rpy_joint.h" //For base
#include "drake/multibody/tree/weld_joint.h" // (To be deleted) For base 

#define base_name "main_body" //name for both model instance, and body_name
#define weld_on true

class olympus {
  public:		
    olympus(drake_builder & builder, const double & time_step_ );

  std::unique_ptr<ntnu_leg> fr_leg;  //pointer_as cannot be initialized initially
  std::unique_ptr<ntnu_leg> rr_leg;
  private:

    void add_body(drake_plant & plant);
    void attach_legs(drake_builder & builder,drake_plant & plant);

  double time_step = 0.002;
  // drake_plant & plant;
  // drake::geometry::SceneGraph<double> & scene_graph;
  drake::multibody::ModelInstanceIndex body_index;

};

// Inside this function the following are created:
//1. drake::multibody::ModelInstanceIndex body_index -> class attribute
//2. const drake::multibody::BallRpyJoint<double> & base_ball_joint -> lost, as it is not actuated
void olympus::add_body(drake_plant & plant){
  //Add the body urdf
  drake::multibody::Parser parser(&plant);
  body_index = parser.AddModels("../urdf/quadruped_base.urdf").at(0);
  plant.RenameModelInstance(body_index, base_name); 

  // Connect to world using a ball rpy joint (Connect {P} and {M})
  drake_tfd T_WP( Eigen::Translation3d{0,0,1}); //frame {P} in the {W} frame
  drake_tfd T_BM( plant.GetBodyByName(base_name).default_com() ) ; //frame {M} in the {B} frame

  if (weld_on){
    const drake::multibody::WeldJoint<double> & weld_joint =
    plant.AddJoint<drake::multibody::WeldJoint> ( "base ball joint",
                                                    plant.world_body(), 
                                                    T_WP,                                                   
                                                    plant.GetBodyByName(base_name),
                                                    T_BM,
                                                    drake_tfd::Identity() );
  }else{
    const drake::multibody::BallRpyJoint<double> & base_ball_joint =
    plant.AddJoint<drake::multibody::BallRpyJoint> ( "base ball joint",
                                                    plant.world_body(), 
                                                    T_WP,                                                   
                                                    plant.GetBodyByName(base_name),
                                                    T_BM );

  }
}

// Inside this function the following are created:
//1. drake_tfd: Transforms for each leg -> maybe usefull
void olympus::attach_legs(drake_builder & builder, drake_plant & plant){
  const drake_rigidBody &robot_base= plant.GetBodyByName(base_name,body_index);

  //TEMPORARY: create config structs for each leg
  drake_tfd frleg_TF( drake_rotMat::MakeXRotation(-M_PI/2), {0.149 ,-0.105,0} ); //FR
  drake_tfd rrleg_TF( drake_rotMat::MakeXRotation(-M_PI/2), {-0.33,-0.15,0} ); //FR
  // drake_tfd rrleg_TF( drake_rotMat::MakeXRotation(-M_PI/2), drake::Vector3<double>{-1,0,1});

  leg_config config_fr(leg_index::fr, robot_base,frleg_TF);
  leg_config config_rr(leg_index::rr, robot_base,rrleg_TF);

  //1b. Instance of each leg
  fr_leg.reset(new ntnu_leg(builder,plant,config_fr) ); 
  rr_leg.reset(new ntnu_leg(builder,plant,config_rr) ); 
}

olympus::olympus(drake_builder & builder, const double & time_step_):
time_step(time_step_),
fr_leg(nullptr),
rr_leg(nullptr)
{
auto [plant,scene_graph]  = drake::multibody::AddMultibodyPlantSceneGraph(&builder,time_step);

add_body(plant);
attach_legs(builder, plant);

// Plant parameters: 
plant.set_discrete_contact_approximation(drake::multibody::DiscreteContactApproximation::kSap);
plant.Finalize();

fr_leg->connect_PID_system(builder,plant);
rr_leg->connect_PID_system(builder,plant);

};