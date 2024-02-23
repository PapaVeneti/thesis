#include "ros/ros.h"

#include "leg_kinematics.hpp"
#include "tf2/buffer_core.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
//messages:
#include "olympus_ros_drake_sim/sphere_signature.h"
#include "sensor_msgs/JointState.h"
#include "olympus_ros_drake_sim/leg_msg.h"
#include "olympus_control/leg_goal.h"
// #include "geometry_msgs/Vector3.h"


//joint_states : {legfr,legrr,legfl,legrl,body}

enum class leg_index { fr, rr, fl, rl};

struct leg_controller_parameters{
  int leg_id = 0;
  int configuration = 0;
  std::string prefix = "";
  tf2::Transform tf_W_MH ; 

  leg_controller_parameters() = default ;
  leg_controller_parameters(const leg_index & id_ ){
    switch (id_){
      case leg_index::fr : 
        leg_id = 1;
        configuration = 1;
        prefix = "fr_";
        break; 
      case leg_index::rr : 
        leg_id = 2;
        configuration = 2;
        prefix = "rr_";
        break; 
      case leg_index::fl : 
        leg_id = 3;
        configuration = 2;
        prefix = "fl_";
        break; 
      case leg_index::rl : 
        leg_id = 4;
        configuration = 1;
        prefix = "rl_";
        break; 
    }
  } 

};

class leg_controller {
public:
    leg_controller(const leg_controller_parameters & params) :
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
    
    void encoderCallback(const sensor_msgs::JointStateConstPtr & JointState){
      q[0] = JointState->position[0 + index_offset];
      q[1] = JointState->position[1 + index_offset];
      q[2] = JointState->position[3 + index_offset];
      

      qt[0] = JointState->velocity[0 + index_offset];
      qt[1] = JointState->velocity[1 + index_offset];
      qt[2] = JointState->velocity[3 + index_offset];
    }

    void userInterfaceCallback(const olympus_control::leg_goal::ConstPtr & setPoint){
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

    void publishCommand(){
        controller_pub.publish(leg_command);
    }



    void publish_sphere_target(){
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

    void Do_publish_sphere_target(const bool & value){
      publishing_sphere_target = value;
    }

private:
    //leg_specific
    leg_kinematics Kinematics ;

    //part of larger_system
    const int index_offset = 0;
    const int leg_id = 0;
    const int configuration = 0;
    const std::string prefix ;

    //ROS
    ros::NodeHandle n;

    //Encode bus:
    ros::Subscriber encoder_sub;
    Eigen::Vector3d q;
    Eigen::Vector3d qt;

    //Control bus:
    olympus_ros_drake_sim::leg_msg leg_command;
    ros::Publisher  controller_pub; 

    //Visualization
    bool publishing_sphere_target =false;
    olympus_ros_drake_sim::sphere_signature sphere_sig;
    ros::Publisher  spherePub;

    //ROS Interface  (this may change)
    ros::Subscriber controller_interface;

    tf2::Transform T_W_MH;
};

int main(int argc, char **argv){

  ros::init(argc, argv, "Controller_placeholder");


  tf2::Transform T_W_MH;
  T_W_MH.setOrigin({0,0,1});
  tf2::Quaternion orientation_W_MH;
  orientation_W_MH.setRPY(-M_PI_2,0,0);
  T_W_MH.setRotation(orientation_W_MH); 


  //settings
  leg_controller_parameters fr_params(leg_index::fr);
  fr_params.tf_W_MH = T_W_MH;
  leg_controller rr_control(fr_params); 
  rr_control.Do_publish_sphere_target(true);


  ros::Rate loop_rate(10);


  while( ros::ok() ){

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;


}