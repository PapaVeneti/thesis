//robot class
#include "ntnu_leg.hpp"
#include <iostream> //-> for some error msgs

// Definition and initialization of static member variables
const std::map<leg_index,std::string> leg_names::full = {
    {leg_index::fr, "front_right"},
    {leg_index::fl, "front_left"},
    {leg_index::rr, "rear_right"},
    {leg_index::rl, "rear_left"}
};

const std::map<leg_index,std::string> leg_names::suffix = {
    {leg_index::fr, "fr"},
    {leg_index::fl, "fl"},
    {leg_index::rr, "rr"},
    {leg_index::rl, "rl"}
};


ntnu_leg::ntnu_leg(
  drake_builder &builder,
  drake_plant & created_plant,
  const leg_config & config): 
  plant(created_plant), 
  leg_id(config.leg_id),
  BushingParams(config.BushingParams),
  SpringParams(config.SpringParams),
  Gains(config.Gains){

//1. Load leg model
// TODO: load depending on side.
drake::multibody::Parser parser(&plant);

switch (leg_id ){
case leg_index::fr:
  leg = parser.AddModels("urdf/ntnu_legFR.urdf").at(0); 
  break;

case leg_index::rr:
  leg = parser.AddModels("urdf/ntnu_legRR.urdf").at(0); 
  break;

case leg_index::fl:
leg = parser.AddModels("urdf/ntnu_legFL.urdf").at(0); 
break;

case leg_index::rl:
leg = parser.AddModels("urdf/ntnu_legRL.urdf").at(0); 
break;

default:
  std::cerr << "Left side legs are not implemented yet. The simulation will fail" <<std::endl;
  break;
}

plant.RenameModelInstance(leg, leg_names::suffix.at(leg_id)+"_leg"); //each model instance must have different name

//2. Connect to parent Body:
const drake::multibody::RevoluteJoint<double> & base_joint =  
plant.AddJoint< drake::multibody::RevoluteJoint >(
  "jointMH",
  config.ParentBody,
  config.TF_B_MH,
  plant.GetBodyByName("MH",leg),
  drake_tfd(),
  Eigen::Vector3d{1,0,0}, //axis of joint in 
  -M_PI_2, // lower position limit
  M_PI_2); // upper position limit



//3. Add actuators - bushing element and spring
add_bushing_joint();
add_linear_spring();
add_collision_sets();
add_actuators(); //MUST CONNECT
add_PID_system(builder,plant);
// connect_PID_system(builder,plant);

// TODO: Add constraint
// drake::systems::SystemConstraint<double> joint1_2_constr(
// ) ; 
// plant.AddExternalConstraint()


}

//getters:

drake_plant & ntnu_leg::get_plant(){
  return plant;
};
drake::systems::controllers::PidController<double> *  ntnu_leg::get_leg_controller(){
  return controller;
};

drake::systems::InputPortIndex       ntnu_leg::get_controller_desired_state_port(){
  return controller_desired_state_port;
};
drake::systems::OutputPortIndex      ntnu_leg::get_leg_output_state_port(){
  return leg_output_state_port;
}
drake::multibody::ModelInstanceIndex ntnu_leg::get_leg_model_instance(){
  return leg;
}
drake::geometry::GeometrySet         ntnu_leg::get_shanks_collision(){
  return shanks_collision_set;
}
drake::geometry::GeometrySet         ntnu_leg::get_MH_collision(){
  return MotorHousing_collision_set;
}
// Building Methods:

//has motors
inline void ntnu_leg::add_actuators(){
  std::string suffix = leg_names::suffix.at(leg_id);
  auto& motorMH = plant.AddJointActuator(suffix+"motorMH",plant.GetJointByName("jointMH",leg));
  auto& motor1  = plant.AddJointActuator(suffix+"motor1" ,plant.GetJointByName("joint11",leg));
  auto& motor2  = plant.AddJointActuator(suffix+"motor2" ,plant.GetJointByName("joint12",leg));
}

//has 1) ee frames, 2)bushing joint
inline void ntnu_leg::add_bushing_joint(){
  double attachment_point_link21 =  0.29977;
  double attachment_point_link22 =  0.29929;

  auto& link21_frame = plant.GetBodyByName("link21",leg).body_frame();
  auto& link22_frame = plant.GetBodyByName("link22",leg).body_frame();

  auto& link21_endpoint = plant.AddFrame(
      std::make_unique<drake::multibody::FixedOffsetFrame<double>>
      (
          leg_names::suffix.at(leg_id)+"link21_endpoint",
          link21_frame,
          drake_tfd(Eigen::Translation3d{attachment_point_link21,0,0})
      ) 
  );

  auto& link22_endpoint = plant.AddFrame(
      std::make_unique<drake::multibody::FixedOffsetFrame<double>>
      (
          leg_names::suffix.at(leg_id)+"link22_endpoint",
          link22_frame,
          drake_tfd(Eigen::Translation3d{attachment_point_link22,0,0})
      ) 
  );

  auto&  bushing = plant.AddForceElement<drake::multibody::LinearBushingRollPitchYaw>(
      link21_endpoint,
      link22_endpoint,
      BushingParams.torque_stiffness_constants,
      BushingParams.torque_damping_constants,
      BushingParams.force_stiffness_constants,
      BushingParams.force_damping_constant
  );

}

//has spring
inline void ntnu_leg::add_linear_spring(){
  auto&  spring = plant.AddForceElement<drake::multibody::LinearSpringDamper>(
      plant.GetBodyByName("link21",leg),
      drake::Vector3<double>::Zero(),
      plant.GetBodyByName("link22",leg),
      drake::Vector3<double>::Zero(),
      SpringParams.free_length,
      SpringParams.stiffness,
      SpringParams.damping
  );
}

//
void ntnu_leg::add_collision_sets(){
  //add shanks collision
  auto &link21 = plant.GetBodyByName("link21",leg);
  auto &link22 = plant.GetBodyByName("link22",leg);

  auto &link21_collision = plant.GetCollisionGeometriesForBody(link21);
  auto &link22_collision = plant.GetCollisionGeometriesForBody(link22);

  shanks_collision_set.Add(link21_collision);
  shanks_collision_set.Add(link22_collision);

  //add motor housing collision
  auto &MH = plant.GetBodyByName("MH",leg);
  auto &MH_collision = plant.GetCollisionGeometriesForBody(link21);
  MotorHousing_collision_set.Add(MH_collision);
}

void ntnu_leg::add_PID_system(drake_builder & builder, drake_plant & plant){
  //0. get prefix:
  std::string prefix = leg_names::suffix.at(leg_id);

  //1. Define Control Projection Matrix:
  //Given estimated state x_in = (q_in, v_in), the controlled state x_c = (q_c, v_c) is computed by x_c = P_x * x_in
  Eigen::Matrix<double,6,10> ControlProjectionMatrix ; 
  ControlProjectionMatrix.setZero();
  //positions
  ControlProjectionMatrix(0,leg_states::qMH) = 1;
  ControlProjectionMatrix(1,leg_states::q11) = 1;
  ControlProjectionMatrix(2,leg_states::q12) = 1;
  //gen velocities
  ControlProjectionMatrix(3,leg_states::vMH) = 1;
  ControlProjectionMatrix(4,leg_states::v11) = 1;
  ControlProjectionMatrix(5,leg_states::v12) = 1;

  //3. Add PID system
  controller = builder.AddNamedSystem<drake::systems::controllers::PidController<double>>(
    prefix+"_controller",
    ControlProjectionMatrix,
    Gains.Kp,
    Gains.Kd,
    Gains.Ki);
}
void ntnu_leg::connect_PID_system(drake_builder & builder, drake_plant & plant){
  if (!plant.is_finalized()){
    std::cerr << "`connect_PID_system` must be called after `plant.Finalize()`" <<std::endl;
  }
  
  //0. get prefix:
  std::string prefix = leg_names::suffix.at(leg_id);
  //4. Connections:

  builder.Connect(plant.get_state_output_port(leg)       ,controller->get_input_port_estimated_state());
  builder.Connect(controller -> get_output_port_control(), plant.get_actuation_input_port(leg)        );

  controller_desired_state_port = builder.ExportInput(controller -> get_input_port_desired_state(),prefix+"leg_setpoint");
  leg_output_state_port             = builder.ExportOutput(plant.get_state_output_port(leg)           ,prefix+"leg_state"   );

}