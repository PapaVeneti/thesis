#include "leg_controller.hpp"

leg_controller::leg_controller(const leg_controller_parameters & params) :
    leg_id(params.leg_id),
    configuration(params.configuration),
    prefix(params.prefix),
    T_W_MH(params.tf_W_MH),
    index_offset(  (params.leg_id - 1)*3 ), 
    Kinematics(params.configuration)
    {   
      controller_interface = n.subscribe("/goal_set", 1000, &leg_controller::userInterfaceCallback, this);
      encoder_sub          = n.subscribe("/joint_states", 1000, &leg_controller::encoderCallback, this);
      controller_pub       = n.advertise<olympus_ros_drake_sim::leg_msg>("/command_interface", 1000,true); //latched publisher
      spherePub            = n.advertise<olympus_ros_drake_sim::sphere_signature>("/publish_spheres",10,true);

      //initialize leg_command
      leg_command.joint_angles.assign(3,0);
      leg_command.joint_velocities.assign(3,0);
      leg_command.n_actuated_joints = 3;
      leg_command.n_actuated_joints = 3;
      leg_command.controller_id =leg_id;     

      //initialize sphere publish
      sphere_sig.path_name = prefix + "leg_goal";
      sphere_sig.sphere_id = leg_id; 
      ROS_INFO_STREAM("[" << prefix << "Controller]: Initialized");

    }
    
void leg_controller::encoderCallback(const sensor_msgs::JointStateConstPtr & JointState){
  q[0] = JointState->position[0 + index_offset];
  q[1] = JointState->position[1 + index_offset];
  q[2] = JointState->position[3 + index_offset];
  

  qt[0] = JointState->velocity[0 + index_offset];
  qt[1] = JointState->velocity[1 + index_offset];
  qt[2] = JointState->velocity[3 + index_offset];
}

void leg_controller::userInterfaceCallback(const olympus_control::leg_goal::ConstPtr & setPoint){
  if (!setPoint->IK_based_command){
    leg_command.joint_angles[0] = setPoint->x1;
    leg_command.joint_angles[1] = setPoint->x2;
    leg_command.joint_angles[2] = setPoint->x3;
  } else {
    Eigen::Vector3d QD = Kinematics.IK({setPoint->x1,setPoint->x2,setPoint->x3});
    leg_command.joint_angles[0] = QD[0];
    leg_command.joint_angles[1] = QD[1];
    leg_command.joint_angles[2] = QD[2];
  }
  


  if (publishing_sphere_target){ publish_sphere_target(); }

  std::vector<double> & qd = leg_command.joint_angles;
  ROS_INFO("[Controller]: New set point: [%.3lf,%.3lf,%.3lf] for controller %d", qd[0],qd[1],qd[2], leg_id);
  publishCommand();
}

void leg_controller::publishCommand(){
    controller_pub.publish(leg_command);
}



void leg_controller::publish_sphere_target(){
    std::vector<double> & qd = leg_command.joint_angles;
    Eigen::Vector3d  P_MH = Kinematics.DK(Eigen::Vector3d(qd[0],qd[1],qd[2]));
    tf2::Vector3     P_MH_tfdatatype(P_MH[0],P_MH[1],P_MH[2]);

    //transform position to {W} frame:
    tf2::Vector3 P_W = T_W_MH*P_MH_tfdatatype ;
    ROS_INFO_STREAM("[Leg Position Controller]: Point in world {W} frame is: [" << P_W[0] << "," << P_W[1] << "," <<P_W[2] << "]" );

    sphere_sig.translation.x = P_W[0];
    sphere_sig.translation.y = P_W[1];
    sphere_sig.translation.z = P_W[2];
    spherePub.publish(sphere_sig);
}

void leg_controller::Do_publish_sphere_target(const bool & value){
  publishing_sphere_target = value;
}
