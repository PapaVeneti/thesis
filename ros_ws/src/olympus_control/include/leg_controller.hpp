#include "ros/ros.h"

//Kinematics
#include "leg_kinematics.hpp"
//tf2
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
//messages:
#include "olympus_ros_drake_sim/sphere_signature.h"
#include "olympus_ros_drake_sim/leg_msg.h"


//joint_states : {legfr,legrr,legfl,legrl,body}

enum class leg_index { fr, rr, fl, rl};

struct leg_controller_parameters{
  int leg_id = 0;
  int configuration = 0;
  std::string prefix = "";
  tf2::Transform tf_B_MH ; 

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

tf2::Transform set_tf_transform(const tf2::Vector3 & translation, const tf2::Vector3 & rpy);

class leg_controller {
public:
    leg_controller(const leg_controller_parameters & params);

    //setters:
    void set_state(const Eigen::Vector3d & q_,const Eigen::Vector3d & qt_);
    void set_state(const std::vector<double> & q_,const std::vector<double> & qt_);
    void set_command(const Eigen::Vector3d & qd_);
    void set_command(const std::vector<double> & qd_);

    //option setters:
    void Do_publish_sphere_target(const bool & value);

    //getters:
    const int  get_leg_id() const ;

    //Functional: Kinematics
    Eigen::Vector3d  IK_querry(const Eigen::Vector3d & xd_);
    Eigen::Vector3d  IK_querry(const std::vector<double> & xd_);

    //Functional: Publishing
    void publishCommand();
    void publish_sphere_target();

private:
    //leg_specific
    leg_kinematics Kinematics ;

    //part of larger_system
    const int index_offset = 0;
    const int leg_id = 0;
    const int configuration = 0;
    const std::string prefix ;

    tf2::Transform T_B_MH;

    //ROS
    ros::NodeHandle n;

    //Current State:
    Eigen::Vector3d q;
    Eigen::Vector3d qt;

    //Control bus:
    olympus_ros_drake_sim::leg_msg leg_command;
    ros::Publisher  controller_pub; 

    //Visualization
    bool publishing_sphere_target =false;
    olympus_ros_drake_sim::sphere_signature sphere_sig;
    ros::Publisher  spherePub;
};