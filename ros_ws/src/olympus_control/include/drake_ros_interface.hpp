//ros
#include "ros/ros.h"

#include "olympus_control/leg_msg.h"
#include "sensor_msgs/JointState.h"
#include "drake/systems/analysis/simulator.h"

//1. add function to publish joint_states
//2. add spheres as target -> tf on ros -> available in rviz too
//3. robot sim
using drake_OutputPortd = drake::systems::OutputPort<double> ;
using drake_InputPortd  = drake::systems::InputPort<double> ;


struct drake_ros_elements {
  drake::systems::Simulator<double> & simulator_;
  const u_int16_t  nj ;
  const u_int16_t  no ;
  const u_int16_t  nu ;
  std::vector <const drake_InputPortd *>  system_input_ports_  ;
  std::vector <const drake_OutputPortd *> system_output_ports_ ;
  std::vector <std::string >              joint_names_ ;

  drake_ros_elements(
    drake::systems::Simulator<double> & deployed_simulator, 
    const u_int16_t & n_control, 
    const u_int16_t & n_output, 
    const u_int16_t & n_joints ):
  simulator_(deployed_simulator),
  nu(n_control),
  no(n_output),
  nj(n_joints)
  {
    joint_names_.assign(nj,"");
  };

  void fill_with_nullptr(){
    system_input_ports_.assign(nu,nullptr);
    system_output_ports_.assign(no,nullptr);
  }

};

class simInterface{
public:

  simInterface(drake_ros_elements & param );

  //core functionality
  void encoderPublish(const sensor_msgs::JointState & CurrentJointState);
  void encoderUpdate();

  //setters
  void set_Joint_states(const sensor_msgs::JointState & CurrentJointState);
  void set_controller_names(const std::vector<std::string> controller_names_);
  void set_joint_names(const std::vector<std::string> joint_names_);
private:

  void controllerCallback(const olympus_control::leg_msg::ConstPtr& msg);

  const uint16_t num_controllers;// (uint16_t) nu;
  const uint16_t num_outputs;// (uint16_t) nu;
  const uint16_t num_joints;// (uint16_t) nu;
  
  //ROS
  ros::NodeHandle n;
  ros::Publisher  encoderPub; //one joint_states_topic
  ros::Subscriber controllerSub;

  sensor_msgs::JointState joint_state; 
  std::vector<std::string> controller_names ; 

  //sensors
  std::vector<std::string> joint_names;
  sensor_msgs::JointState JointState;

  //drake connection
  std::vector <const drake_InputPortd *>  input_ports;
  std::vector <const drake_OutputPortd *> output_ports;
  drake::systems::Simulator<double> & simulator;

};