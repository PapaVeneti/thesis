#include "leg_controller.hpp"

leg_controller::leg_controller(const leg_controller_parameters & params) :
    leg_id(params.leg_id),
    configuration(params.configuration),
    prefix(params.prefix),
    T_B_MH(params.tf_B_MH),
    index_offset(  (params.leg_id - 1)*3 ), 
    Kinematics(params.configuration)
    {   
      controller_pub       = n.advertise<olympus_ros_drake_sim::leg_msg>("/command_interface", 1000,true); //latched publisher
      spherePub            = n.advertise<olympus_ros_drake_sim::sphere_signature>("/publish_spheres",10,true);

      //initialize leg_command
      leg_command.joint_angles.assign(3,0);
      leg_command.joint_velocities.assign(3,0);
      leg_command.n_actuated_joints = 3;
      leg_command.controller_id =leg_id;     

      //initialize sphere publish
      sphere_sig.path_name = prefix + "leg_goal";
      sphere_sig.sphere_id = leg_id; 
      ROS_INFO_STREAM("[" << prefix << "Controller]: Initialized");
    }

//Setters:
void leg_controller::set_state(const Eigen::Vector3d & q_,const Eigen::Vector3d & qt_){
  q = q_;
  qt=qt_;
}
void leg_controller::set_state(const std::vector<double> & q_,const std::vector<double> & qt_){
  if ( q_.size() == 3 && qt_.size() ) {
    q[0] = q_[0];
    q[1] = q_[1];
    q[2] = q_[2];

    qt[0] = qt_[0];
    qt[1] = qt_[1];
    qt[2] = qt_[2];
  }else {
    ROS_ERROR("[Position Controller]: wrong state size");
  }
}
    
void leg_controller::set_command(const Eigen::Vector3d & qd_){
  leg_command.joint_angles[0] = qd_[0];
  leg_command.joint_angles[1] = qd_[1];
  leg_command.joint_angles[2] = qd_[2];
}
void leg_controller::set_command(const std::vector<double> & qd_){
  if ( qd_.size() == 3 ) {
    leg_command.joint_angles[0] = qd_[0];
    leg_command.joint_angles[1] = qd_[1];
    leg_command.joint_angles[2] = qd_[2];
  }else {
    ROS_ERROR("[Position Controller]: wrong desired state size");
  }
}

//option setters:
void leg_controller::Do_publish_sphere_target(const bool & value){
  publishing_sphere_target = value;
}

//getters:
const int  leg_controller::get_leg_id() const {
  return leg_id ;
}

//Functional: Kinematics
Eigen::Vector3d  leg_controller::IK_querry(const Eigen::Vector3d & xd_){
  return Kinematics.IK({xd_[0],xd_[1],xd_[2]});
}
Eigen::Vector3d  leg_controller::IK_querry(const std::vector<double> & xd_){
  if ( xd_.size() == 3 ) {
    return Kinematics.IK({xd_[0],xd_[1],xd_[2]});
  }else {
    ROS_ERROR("[Position Controller]: wrong desired position");
    ROS_ERROR("[Position Controller]: return current position");
    return q;
  }
}

//Functional: Publishing
void leg_controller::publishCommand(){
    controller_pub.publish(leg_command);
}
void leg_controller::publish_sphere_target(){
    std::vector<double> & qd = leg_command.joint_angles;
    Eigen::Vector3d  P_MH = Kinematics.DK(Eigen::Vector3d(qd[0],qd[1],qd[2]));
    tf2::Vector3     P_MH_tfdatatype(P_MH[0],P_MH[1],P_MH[2]);

    //transform position to {W} frame:
    tf2::Vector3 P_B = T_B_MH*P_MH_tfdatatype ;
    ROS_INFO_STREAM("[Leg Position Controller]: Point in body {B} frame is: [" << P_B[0] << "," << P_B[1] << "," <<P_B[2] << "]" );

    sphere_sig.translation.x = P_B[0];
    sphere_sig.translation.y = P_B[1];
    sphere_sig.translation.z = P_B[2];
    spherePub.publish(sphere_sig);
}

tf2::Transform set_tf_transform(const tf2::Vector3 & translation, const tf2::Vector3 & rpy){
  tf2::Transform Tf;
  Tf.setOrigin(translation);
  tf2::Quaternion quat;
  quat.setRPY(rpy[0],rpy[1],rpy[2]);
  Tf.setRotation(quat); 

  return Tf;
}