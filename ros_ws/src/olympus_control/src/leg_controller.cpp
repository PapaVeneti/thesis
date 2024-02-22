#include "ros/ros.h"
// #include "std_msgs/Float64.h"
// #include "std_msgs/Float64MultiArray.h"
// #include "geometry_msgs/Vector3.h"
#include "olympus_control/sphere_signature.h"
#include "sensor_msgs/JointState.h"

#include "olympus_control/leg_msg.h"
#include "position_controller.hpp"
#include <chrono>


// class controller {
// public:
//     controller() : xd(0.0), n(), controller_interface(){
//         controller_interface = n.subscribe("/goal_set", 1000, &controller::interfaceCallback, this);
//         encoder_sub          = n.subscribe("/joint_states", 1000, &controller::encoderCallback, this);
//         controller_pub       = n.advertise<std_msgs::Float64>("/joint1_position_controller", 1000,true); //latched publisher
//         ROS_INFO("[Controller]: Initialized");

//     }
    
//     void encoderCallback(const sensor_msgs::JointStateConstPtr & JointState){
//         ROS_DEBUG("[Controller]: Pendulum position is: %.3lf", JointState->position[0]);
//     }

//     void interfaceCallback(const std_msgs::Float64::ConstPtr & setPoint){
//         xd = setPoint->data; 
//         ROS_INFO("[Controller]: New controller set point: %.3lf", xd);
//         publishCommand();
//     }

//     void publishCommand(){
//         joint1_command.data = xd ; 
//         controller_pub.publish(joint1_command);
//     }

// private:
//     double xd;
//     ros::NodeHandle n;
//     std_msgs::Float64 joint1_command;
//     ros::Subscriber controller_interface;
//     ros::Subscriber encoder_sub;
//     ros::Publisher controller_pub; 
// };

int main(int argc, char **argv){

  ros::init(argc, argv, "Controller_placeholder");
  // controller controller;

  ros::NodeHandle n;
  // ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/position_controller",10,true);
  ros::Publisher pub  = n.advertise<olympus_control::leg_msg>("/position_controller",10,true);
  ros::Publisher pub2 = n.advertise<olympus_control::sphere_signature>("/publish_spheres",10,true);
  ros::Rate loop_rate(10);

  olympus_control::leg_msg msg;
  olympus_control::sphere_signature sphere_sig;
  if (argc != 4) {  
    ROS_ERROR_STREAM("[Position Controller]: The number of arguments must be 3");
    ROS_ERROR_STREAM("[Position Controller]: No command");
    return 1;
  }


  // msg.data.push_back(1);
  // msg.data.push_back(1);
  // msg.data.push_back(1);
  ros::spinOnce();

  msg.controller_id =1;
  msg.n_actuated_joints = 3;
  msg.joint_angles.push_back(atof(argv[1]));
  msg.joint_angles.push_back(atof(argv[2]));
  msg.joint_angles.push_back(atof(argv[3]));
  msg.joint_velocities.assign(3,0);
  pub.publish(msg);

  // position_controller con(2);
  // Eigen::Isometry3d tf;
  // tf.translate(Eigen::Vector3d({0,0,1}));
  // Eigen::Matrix3d rot;
  // rot.setZero();
  // rot(0,0)= 1;
  // rot(1,2) = 1;
  // rot(2,1) = -1;
  // tf.rotate(rot);

  // drake_tfd T_MH_P(drake::math::RollPitchYawd(0,0,0),pos_controller.DK(des_angles));
// drake_tfd T_W_P = frleg_TF*T_MH_P; 

  sphere_sig.path_name = "hey";
  sphere_sig.sphere_id = 1;
  sphere_sig.translation.x = 1;
  sphere_sig.translation.y = 1;
  sphere_sig.translation.z = 1;
  pub2.publish(sphere_sig);



  auto starting_time = std::chrono::system_clock::now();

  while(true){
    auto current_time = std::chrono::system_clock::now();

    auto duration =  std::chrono::duration_cast<std::chrono::milliseconds> (current_time - starting_time) ;
    if (duration >= std::chrono::milliseconds(500) ) {
      break;
    }
    ros::spinOnce();
  }





  return 0;


}