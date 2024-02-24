#include "olympus.hpp"

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

//builder functions:

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

//functionality
//initialized both desired state and current state to 0

void olympus::default_init(drake::systems::Diagram<double> & diagram, drake::systems::Context<double> & context){
    auto& fr_controller_context = diagram. GetMutableSubsystemContext ( *( fr_leg->get_leg_controller()),&context);
    auto& rr_controller_context = diagram. GetMutableSubsystemContext ( *( rr_leg->get_leg_controller()),&context);
    auto& fl_controller_context = diagram. GetMutableSubsystemContext ( *( fl_leg->get_leg_controller()),&context);
    auto& rl_controller_context = diagram. GetMutableSubsystemContext ( *( rl_leg->get_leg_controller()),&context);

    auto& plant_context      = diagram. GetMutableSubsystemContext ( *plant,&context);
    Eigen::VectorXd q0(10);
    q0.setZero(); 
    plant->SetPositionsAndVelocities(&plant_context, fr_leg->get_leg_model_instance(), q0);
    plant->SetPositionsAndVelocities(&plant_context, rr_leg->get_leg_model_instance(), q0);
    plant->SetPositionsAndVelocities(&plant_context, fl_leg->get_leg_model_instance(), q0);
    plant->SetPositionsAndVelocities(&plant_context, rl_leg->get_leg_model_instance(), q0);

    Eigen::Matrix<double,6,1> qd;
    qd.setZero();
    diagram. get_input_port( fr_leg->get_controller_desired_state_port()).FixValue(&context,qd);
    diagram. get_input_port( rr_leg->get_controller_desired_state_port()).FixValue(&context,qd);
    diagram. get_input_port( fl_leg->get_controller_desired_state_port()).FixValue(&context,qd);
    diagram. get_input_port( rl_leg->get_controller_desired_state_port()).FixValue(&context,qd);
}

//getters

std::vector<std::string> olympus::get_joint_names(){
  std::vector<std::string> joint_names;

  for (auto joint_id : plant->GetJointIndices(fr_leg->get_leg_model_instance()) ) {
    joint_names.push_back( "fr_"+  plant->get_joint(joint_id).name() );
  }
  for (auto joint_id : plant->GetJointIndices(rr_leg->get_leg_model_instance()) ) {
    joint_names.push_back( "rr_"+  plant->get_joint(joint_id).name() );
  }
  for (auto joint_id : plant->GetJointIndices(fl_leg->get_leg_model_instance()) ) {
    joint_names.push_back( "fl_"+  plant->get_joint(joint_id).name() );
  }
  for (auto joint_id : plant->GetJointIndices(rl_leg->get_leg_model_instance()) ) {
    joint_names.push_back( "rl_"+  plant->get_joint(joint_id).name() );
  }
  
  
  return joint_names;
}
drake_plant & olympus::get_plant(){
  return *plant;
}
std::vector<const drake::systems::InputPort<double> *> olympus::get_input_ports(drake::systems::Diagram<double> & diagram){
  std::vector<const drake::systems::InputPort<double> *> input_ports_vector;

  input_ports_vector.push_back( & diagram.get_input_port(fr_leg->get_controller_desired_state_port()));
  input_ports_vector.push_back( & diagram.get_input_port(rr_leg->get_controller_desired_state_port()));
  input_ports_vector.push_back( & diagram.get_input_port(fl_leg->get_controller_desired_state_port()));
  input_ports_vector.push_back( & diagram.get_input_port(rl_leg->get_controller_desired_state_port()));

  return input_ports_vector ; 

}
std::vector<const drake::systems::OutputPort<double> *> olympus::get_output_ports(drake::systems::Diagram<double> & diagram){
  std::vector<const drake::systems::OutputPort<double> *>  output_ports_vector;


  output_ports_vector.push_back( & diagram.get_output_port(fr_leg->get_leg_output_state_port()));
  output_ports_vector.push_back( & diagram.get_output_port(rr_leg->get_leg_output_state_port()));
  output_ports_vector.push_back( & diagram.get_output_port(fl_leg->get_leg_output_state_port()));
  output_ports_vector.push_back( & diagram.get_output_port(rl_leg->get_leg_output_state_port()));

  return output_ports_vector ; 
}


