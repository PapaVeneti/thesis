#include "ros/ros.h"

#include "position_controller.hpp"

//messages:
#include "olympus_control/sphere_signature.h"
#include "sensor_msgs/JointState.h"
#include "olympus_control/leg_msg.h"
#include "geometry_msgs/Vector3.h"



class leg_controller {
public:
    leg_controller() :Kinematics(2), n(), controller_interface(){
        controller_interface = n.subscribe("/goal_set", 1000, &leg_controller::interfaceCallback, this);
        encoder_sub          = n.subscribe("/joint_states", 1000, &leg_controller::encoderCallback, this);
        controller_pub       = n.advertise<olympus_control::leg_msg>("/position_controller", 1000,true); //latched publisher
        spherePub            = n.advertise<olympus_control::sphere_signature>("/publish_spheres",10,true);
        ROS_INFO("[Controller]: Initialized");

        //initialize leg_command
        leg_command.joint_angles.assign(3,0);
        leg_command.joint_velocities.assign(3,0);
        leg_command.n_actuated_joints = 3;
        leg_command.n_actuated_joints = 3;
        leg_command.controller_id =1;
    }
    
    void encoderCallback(const sensor_msgs::JointStateConstPtr & JointState){
      q[0] = JointState->position[0];
      q[1] = JointState->position[1];
      q[2] = JointState->position[3];

      qt[0] = JointState->velocity[0];
      qt[1] = JointState->velocity[1];
      qt[2] = JointState->velocity[3];
    }

    void interfaceCallback(const geometry_msgs::Vector3::ConstPtr & setPoint){
      leg_command.joint_angles[0] = setPoint->x;
      leg_command.joint_angles[1] = setPoint->y;
      leg_command.joint_angles[2] = setPoint->z;


      auto  P_MH = Kinematics.DK(Eigen::Vector3d({setPoint->x,setPoint->x,setPoint->x}));

      Eigen::Matrix3d rot_rr ;
      rot_rr.setZero();

      rot_rr(0,0) = -1;
      rot_rr(1,2) =  1;
      rot_rr(2,1) =  1;

      Eigen::Isometry3d TF_W_RR;
      TF_W_RR.translate(Eigen::Vector3d({0,0,1}));
      TF_W_RR.rotate(rot_rr);

      Eigen::Isometry3d TF_MH_P ;
      TF_MH_P.rotate(Eigen::Matrix3d::Identity());;
      TF_MH_P.translate(Eigen::Vector3d(P_MH));

      // auto TF_D  = TF_W_RR*TF_MH_P; 
      
      auto TF_D = rot_rr*P_MH  + Eigen::Vector3d({0,0,1});

      

      sphere_sig.path_name = "rr_leg_goal";
      sphere_sig.sphere_id = 2;
      sphere_sig.translation.x = TF_D(0);
      sphere_sig.translation.y = TF_D(1);
      sphere_sig.translation.z = TF_D(2);
      spherePub.publish(sphere_sig);

      ROS_INFO("[Controller]: New controller set point: [%.3lf,%.3lf,%.3lf]", setPoint->x,setPoint->y,setPoint->z);
      publishCommand();
    }

    void publishCommand(){
        controller_pub.publish(leg_command);
    }

private:
    position_controller Kinematics ;


    //ROS
    ros::NodeHandle n;

    //Encode bus:
    ros::Subscriber encoder_sub;
    Eigen::Vector3d q;
    Eigen::Vector3d qt;
    // sensor_msgs::JointState joint_state; 

    //Control bus:
    olympus_control::leg_msg leg_command;
    ros::Publisher  controller_pub; 

    //Visualization
    olympus_control::sphere_signature sphere_sig;
    ros::Publisher  spherePub;

    //ROS Interface  (this may change)
    ros::Subscriber controller_interface;

    
    
};

int main(int argc, char **argv){

  ros::init(argc, argv, "Controller_placeholder");


  leg_controller rr_control; 
  ros::Rate loop_rate(10);


  while( ros::ok() ){

    ros::spinOnce();
    loop_rate.sleep();
  }



  return 0;


}