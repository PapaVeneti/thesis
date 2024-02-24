#include "ros/ros.h"

#include "leg_kinematics.hpp"

#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
//messages:
#include "olympus_ros_drake_sim/sphere_signature.h"
#include "sensor_msgs/JointState.h"
#include "olympus_ros_drake_sim/leg_msg.h"
#include "olympus_control/leg_goal.h"


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
    leg_controller(const leg_controller_parameters & params);
    
    void encoderCallback(const sensor_msgs::JointStateConstPtr & JointState);

    void userInterfaceCallback(const olympus_control::leg_goal::ConstPtr & setPoint);

    void publishCommand();

    void publish_sphere_target();

    void Do_publish_sphere_target(const bool & value);

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