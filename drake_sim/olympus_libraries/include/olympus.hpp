#include "ntnu_leg.hpp" //For Legs
#include "drake/multibody/tree/ball_rpy_joint.h" //For base
#include "drake/multibody/tree/weld_joint.h" // (To be deleted) For base 

#define base_name "main_body" //name for both model instance, and body_name
#define weld_on true

using drake_rpy       = drake::math::RollPitchYawd ;


class olympus {
  public:		
    olympus(drake_builder & builder, const double & time_step_ );
    
    //getters
    drake_plant & get_plant();

  std::unique_ptr<ntnu_leg> fr_leg;  //pointer_as cannot be initialized initially
  std::unique_ptr<ntnu_leg> rr_leg;
  std::unique_ptr<ntnu_leg> fl_leg;
  std::unique_ptr<ntnu_leg> rl_leg;
  private:

    void add_body();
    void attach_legs(drake_builder & builder);
    void handle_collisions(drake::geometry::SceneGraph<double> & scene_graph);

  double time_step = 0.002;
  drake_plant * plant;
  // drake::geometry::SceneGraph<double> & scene_graph;
  drake::multibody::ModelInstanceIndex body_index;
  drake::geometry::GeometrySet body_collision;

  //Transforms for each leg
  drake_tfd frleg_TF;
  drake_tfd rrleg_TF;
  drake_tfd flleg_TF;
  drake_tfd rlleg_TF;

};

// Inside this function the following are created:
//1. drake::multibody::ModelInstanceIndex body_index -> class attribute
//2. const drake::multibody::BallRpyJoint<double> & base_ball_joint -> lost, as it is not actuated
void olympus::add_body(){
  //Add the body urdf
  drake::multibody::Parser parser(plant);
  body_index = parser.AddModels("urdf/quadruped_base.urdf").at(0);
  plant->RenameModelInstance(body_index, base_name); 

  // Connect to world using a ball rpy joint (Connect {P} and {M})
  drake_tfd T_WP( Eigen::Translation3d{0,0,1}); //frame {P} in the {W} frame
  drake_tfd T_BM( plant->GetBodyByName(base_name).default_com() ) ; //frame {M} in the {B} frame

  if (weld_on){
    const drake::multibody::WeldJoint<double> & weld_joint =
    plant->AddJoint<drake::multibody::WeldJoint> ( "base ball joint",
                                                    plant->world_body(), 
                                                    T_WP,                                                   
                                                    plant->GetBodyByName(base_name),
                                                    T_BM,
                                                    drake_tfd::Identity() );
  }else{
    const drake::multibody::BallRpyJoint<double> & base_ball_joint =
    plant->AddJoint<drake::multibody::BallRpyJoint> ( "base ball joint",
                                                    plant->world_body(), 
                                                    T_WP,                                                   
                                                    plant->GetBodyByName(base_name),
                                                    T_BM );

  }

  body_collision.Add(plant->GetCollisionGeometriesForBody( plant->GetBodyByName(base_name) ) );
}

// Inside this function the following are created:
//1. drake_tfd: Transforms for each leg -> maybe usefull
void olympus::attach_legs(drake_builder & builder){
  const drake_rigidBody &robot_base= plant->GetBodyByName(base_name,body_index);

  drake_rpy  rpy_B_FR (Eigen::Vector3d( {-M_PI_2,0,0}    ));
  drake_rpy  rpy_B_RR (Eigen::Vector3d( {-M_PI_2,M_PI,0} ));
  drake_rpy  rpy_B_FL (Eigen::Vector3d( { M_PI_2,0,0}    ));
  drake_rpy  rpy_B_RL (Eigen::Vector3d( { M_PI_2,M_PI,0} ));

  double DX = 0.1485;
  double DY_f = 0.105;
  double DY_r = 0.15;
  double DZ = 0;

  // Save the transforms to keep them for later
  frleg_TF = drake_tfd( rpy_B_FR, { DX,-DY_f,DZ} ); //FR
  rrleg_TF = drake_tfd( rpy_B_RR, {-DX,-DY_r,DZ} ); //RR
  flleg_TF = drake_tfd( rpy_B_FL, { DX, DY_f,DZ} ); //RR
  rlleg_TF = drake_tfd( rpy_B_RL, {-DX, DY_r,DZ} ); //RR


  // drake_tfd rrleg_TF( drake_rotMat::MakeXRotation(-M_PI/2), drake::Vector3<double>{-1,0,1});

  leg_config config_fr(leg_index::fr, robot_base,frleg_TF);
  leg_config config_rr(leg_index::rr, robot_base,rrleg_TF);
  leg_config config_fl(leg_index::fl, robot_base,flleg_TF);
  leg_config config_rl(leg_index::rl, robot_base,rlleg_TF);

  //1b. Instance of each leg
  fr_leg.reset(new ntnu_leg(builder,*plant,config_fr) ); 
  rr_leg.reset(new ntnu_leg(builder,*plant,config_rr) ); 
  fl_leg.reset(new ntnu_leg(builder,*plant,config_fl) ); 
  rl_leg.reset(new ntnu_leg(builder,*plant,config_rl) ); 
}

void olympus::handle_collisions(drake::geometry::SceneGraph<double> & scene_graph){
  auto collision_manager = scene_graph.collision_filter_manager();
  
  //Exlude collisions between link21,12
  collision_manager.Apply(
  drake::geometry::CollisionFilterDeclaration().
  ExcludeWithin(fr_leg->get_shanks_collision()).
  ExcludeWithin(fl_leg->get_shanks_collision()).
  ExcludeWithin(rr_leg->get_shanks_collision()).
  ExcludeWithin(rl_leg->get_shanks_collision())
  );


  collision_manager.Apply(
  drake::geometry::CollisionFilterDeclaration().
  ExcludeBetween(fr_leg->get_MH_collision(),body_collision).
  ExcludeBetween(fl_leg->get_MH_collision(),body_collision).
  ExcludeBetween(rr_leg->get_MH_collision(),body_collision).
  ExcludeBetween(rl_leg->get_MH_collision(),body_collision)
  );


}

olympus::olympus(drake_builder & builder, const double & time_step_):
time_step(time_step_),
fr_leg(nullptr), rr_leg(nullptr), fl_leg(nullptr), rl_leg(nullptr)
{
auto [plant_,scene_graph]  = drake::multibody::AddMultibodyPlantSceneGraph(&builder,time_step);
plant = &plant_;

add_body();
attach_legs(builder);
handle_collisions(scene_graph);

// Plant parameters: 
plant->set_discrete_contact_approximation(drake::multibody::DiscreteContactApproximation::kSap);
plant->Finalize();

fr_leg->connect_PID_system(builder,*plant);
rr_leg->connect_PID_system(builder,*plant);
fl_leg->connect_PID_system(builder,*plant);
rl_leg->connect_PID_system(builder,*plant);

};

drake_plant & olympus::get_plant(){
  return *plant;
}
