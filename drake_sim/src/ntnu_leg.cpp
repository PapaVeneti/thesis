//robot class
#include "ntnu_leg.hpp"
#include <iostream>

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
  const leg_config & config): plant(created_plant), 
                              leg_id(config.leg_id){

//1. Load leg model
// TODO: load depending on side.
drake::multibody::Parser parser(&plant);

if (leg_id == leg_index::fr || leg_id == leg_index::rr ){
  leg = parser.AddModels("../urdf/ntnu_leg.urdf").at(0); 
} else {
  std::cerr << "Left side legs are not implemented yet. The simulation will fail" <<std::endl;
  leg = parser.AddModels("../urdf/ntnu_leg.urdf").at(0); 
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
add_actuators();

// TODO: Add constraint
// drake::systems::SystemConstraint<double> joint1_2_constr(
// ) ; 
// plant.AddExternalConstraint()


}

inline void ntnu_leg::add_actuators(){
  std::string suffix = leg_names::suffix.at(leg_id);
  auto& motorMH = plant.AddJointActuator(suffix+"motorMH",plant.GetJointByName("jointMH",leg));
  auto& motor1  = plant.AddJointActuator(suffix+"motor1" ,plant.GetJointByName("joint11",leg));
  auto& motor2  = plant.AddJointActuator(suffix+"motor2" ,plant.GetJointByName("joint12",leg));

  

  // plant.get_mutable_joint_actuator(motorMH.index()).set_controller_gains(Gains.gains_MH     ); 
  // plant.get_mutable_joint_actuator(motor1 .index()).set_controller_gains(Gains.gains_joint11); 
  // plant.get_mutable_joint_actuator(motor2 .index()).set_controller_gains(Gains.gains_joint21); 
}
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

void ntnu_leg::set_leg_gains(const controllerGains & newGains){
  Gains = newGains ; 
}
void ntnu_leg::set_bushing_params(const BushingParamsStruct & newBushingParams){
  BushingParams = newBushingParams;
}
void ntnu_leg::set_spring_params(const SpringParamsStruct & newSpringParams){
  SpringParams = newSpringParams;
}


