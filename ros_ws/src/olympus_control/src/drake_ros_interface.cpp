#include "drake_ros_interface.hpp"

// Overload << operator to output vector data
template <typename T>
std::ostream& operator<<(std::ostream& stream, const std::vector<T>& vec) {
    for (auto it = vec.begin() ; it < vec.end()-1 ; ++it ){
      stream << *it << ",";
    }
    if (!vec.empty()) {
        stream << *(vec.end() - 1); // Output the last element without a comma
    }
    return stream;
}

simInterface::simInterface(drake_ros_elements & param ):
  n(),
  simulator(param.simulator_),
  num_controllers(param.nu),
  num_outputs(param.no),
  num_joints(param.nj),
  input_ports(param.system_input_ports_),
  output_ports(param.system_output_ports_)
  { 
    assert(param.system_input_ports_.size()  == num_controllers);
    assert(param.system_output_ports_.size() == num_outputs);
    controllerSub  = n.subscribe("/position_controller", 1000, &simInterface::controllerCallback,this);
    encoderPub     = n.advertise<sensor_msgs::JointState>("/joint_states",10);

    set_joint_names(param.joint_names_);
    JointState.position.assign(num_joints,0);
    JointState.velocity.assign(num_joints,0);
    JointState.effort.assign(num_joints,0);
  }

void simInterface::encoderPublish(const sensor_msgs::JointState & CurrentJointState){
  joint_state = CurrentJointState;
  encoderPub.publish(joint_state);

} 

void simInterface::encoderUpdate(){
  int index = 0;
  for (auto output_port : output_ports){
    int port_SO_state_size = output_port -> size() / 2  ; //second order state size
    output_port -> Eval(simulator.get_context());
    const Eigen::VectorXd  & state_vector = output_port->Eval(simulator.get_context());

    int i = 0 ;
    for (i; i < port_SO_state_size ;i++){
      JointState.position[index + i] = state_vector[i];
      JointState.velocity[index + i] = state_vector[i + port_SO_state_size]; 
    }
    index = i;
  
  }

  // const Eigen::VectorXd  & state_vector = output_ports[0]->Eval(simulator.get_context());

  // for (int i = 0; i < num_joints ;i++){
  //   JointState.position[i] = state_vector[i];
  //   JointState.velocity[i] = state_vector[i+num_joints];   
  // }

  JointState.header.stamp = ros::Time::now();
  encoderPublish(JointState);
}

void simInterface::set_Joint_states(const sensor_msgs::JointState & CurrentJointState){
  joint_state = CurrentJointState;
}

void simInterface::set_controller_names(const std::vector<std::string> controller_names_){
  if ( controller_names_.size() != num_controllers) {
    ROS_ERROR("[Drake Sim Interface]: Different number of controller names given. The system has %d controllers active.",num_controllers);
  } else {
    controller_names = controller_names_;
  }
}

void simInterface::set_joint_names(const std::vector<std::string> joint_names_){
  if ( joint_names_.size() != num_joints) {
    ROS_ERROR("[Drake Sim Interface]: Different number of joint names given. The system has %d joints.",num_joints);
  } else {
    joint_names = joint_names_;
    JointState.name = joint_names;
  }
}

void simInterface::controllerCallback(const olympus_control::leg_msg::ConstPtr& msg){
  const uint16_t& N   = msg->n_actuated_joints;
  const uint16_t& id =  msg->controller_id;
  if( id <= num_controllers && id > 0 ){

  ROS_INFO_STREAM(
    "[simController]: New command for "<< 
    "controller 1" << " : [" <<
    msg->joint_angles << "]" );

  const std::vector<double> & p=  msg->joint_angles;
  const std::vector<double> & v=  msg->joint_velocities;

  Eigen::VectorXd qd(2*N) ;
  for(int i = 0; i<N ; i++){
    qd[i]   = p[i];
    qd[i+N] = v[i];
  }

  auto& sim_context = simulator.get_mutable_context();
  input_ports[id-1]->FixValue(&sim_context,qd);
  }
  else{
    ROS_INFO_STREAM(
    "[simController]: The controller has  #" <<
    num_controllers <<  
    "active. The command was specified the controller n. "<<
    id << ". Thus command is discarded" );
  }
}

