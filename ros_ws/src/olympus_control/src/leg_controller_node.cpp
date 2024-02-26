#include "ros/ros.h"

#include "leg_controller.hpp"

//messages:
#include "sensor_msgs/JointState.h"
#include "olympus_control/leg_goal.h"


class leg_controller_wrapper {
  public:		
    leg_controller_wrapper(const leg_controller_parameters & control_params):
    leg_control(control_params)
    {
      encoder_sub          = n.subscribe("/joint_states", 1000, &leg_controller_wrapper::encoderCallback, this);
      controller_interface = n.subscribe("/goal_set", 1000, &leg_controller_wrapper::userInterfaceCallback, this);
      publishing_sphere_target = true;
    }

  private:

    void encoderCallback(const sensor_msgs::JointStateConstPtr & JointState);
    void userInterfaceCallback(const olympus_control::leg_goal::ConstPtr & setPoint);

    ros::NodeHandle n;
    leg_controller leg_control;

    ros::Subscriber encoder_sub;
    ros::Subscriber controller_interface;

    bool publishing_sphere_target =false;

};

void leg_controller_wrapper::encoderCallback(const sensor_msgs::JointStateConstPtr & JointState){
  leg_control.set_state(JointState->position,JointState->velocity);
}

void leg_controller_wrapper::userInterfaceCallback(const olympus_control::leg_goal::ConstPtr & setPoint){
  const double & x1 = setPoint->x1;
  const double & x2 = setPoint->x2;
  const double & x3 = setPoint->x3;


  if (!setPoint->IK_based_command){
    leg_control.set_command(std::vector<double>{x1,x2,x3});
    ROS_INFO("[Controller]: New set point: [%.3lf,%.3lf,%.3lf] for controller %d", x1,x2,x3, leg_control.get_leg_id());
  } else {
    Eigen::Vector3d QD = leg_control.IK_querry(Eigen::Vector3d({x1,x2,x3}));
    leg_control.set_command(QD);
    ROS_INFO("[Controller]: New set point: [%.3lf,%.3lf,%.3lf] for controller %d", QD[0],QD[1],QD[2], leg_control.get_leg_id());

  }
  
  if (publishing_sphere_target){ leg_control.publish_sphere_target(); }

  leg_control.publishCommand();
}


int main(int argc, char **argv){

  ros::init(argc, argv, "Controller_placeholder");

  //settings
  leg_controller_parameters fr_params(leg_index::fr);
  fr_params.tf_B_MH = set_tf_transform({ 0,0,1},{-M_PI_2,   0,0});

  leg_controller_wrapper fr_control(fr_params);
  ros::Rate loop_rate(10);


  while( ros::ok() ){

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;


}