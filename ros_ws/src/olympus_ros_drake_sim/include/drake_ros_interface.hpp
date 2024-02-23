//ros
#include "ros/ros.h"

//messages:
#include "olympus_ros_drake_sim/leg_msg.h"
#include "sensor_msgs/JointState.h"
#include "olympus_ros_drake_sim/sphere_signature.h"

//drake specific
#include "drake/systems/analysis/simulator.h"
#include "drake/geometry/meshcat.h" //Access the visualizer (camera, recording etc)

//custom:
#include "drake_helpers.hpp"

//2. add spheres as target -> tf on ros -> available in rviz too
//3. robot sim
using drake_OutputPortd  = drake::systems::OutputPort<double> ;
using drake_InputPortd   = drake::systems::InputPort<double> ;
using meshcat_shared_ptr = std::shared_ptr<drake::geometry::Meshcat>; 
using drake_tfd          = drake::math::RigidTransform<double>;


struct point_stack_element {
  std::string path; //used for deletion by meshcat
  uint16_t id = 0;
  ros::WallTime initialization_time; 

  point_stack_element(const std::string & path_,const ros::WallTime & time): 
  path (path_),
  initialization_time(time) {};
};

struct drake_ros_elements {
  drake::systems::Simulator<double> & simulator_;
  const u_int16_t  nj ;
  const u_int16_t  no ;
  const u_int16_t  nu ;
  std::vector <const drake_InputPortd *>  system_input_ports_  ;
  std::vector <const drake_OutputPortd *> system_output_ports_ ;
  std::vector <std::string >              joint_names_ ;
  meshcat_shared_ptr meshcat_ptr;

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
    meshcat_ptr = nullptr;
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

  void controllerCallback(const olympus_ros_drake_sim::leg_msg::ConstPtr& msg);
  void addSphereCallback(const olympus_ros_drake_sim::sphere_signature::ConstPtr& msg);
  void add_sphere(Eigen::Vector3d Pw, uint16_t id=0, std::string path_name = "Goal Position");
  void delete_sphere_callback(const ros::WallTimerEvent&);
  void renew_sphere_timer();

  const uint16_t num_controllers;// (uint16_t) nu;
  const uint16_t num_outputs;    // (uint16_t) nu;
  const uint16_t num_joints;     // (uint16_t) nu;
  
  //ROS
  ros::NodeHandle n;
  ros::Publisher  encoderPub; //one joint_states_topic
  ros::Subscriber controllerSub;
  ros::Subscriber sphereSub;

  sensor_msgs::JointState joint_state; 
  std::vector<std::string> controller_names ; 

  ros::WallTimer timer;
  std::vector<point_stack_element> sphere_stack; 

  //sensors
  std::vector<std::string> joint_names;
  sensor_msgs::JointState JointState;

  //drake connection
  std::vector <const drake_InputPortd *>  input_ports;
  std::vector <const drake_OutputPortd *> output_ports;
  drake::systems::Simulator<double> & simulator;


  meshcat_shared_ptr meshcat_ptr ;
};