#include "ros/ros.h"

#include "leg_controller.hpp"
//messages:
#include "sensor_msgs/JointState.h"
#include "olympus_control/leg_goal.h"


class olympus_controller {
  public:		
    olympus_controller(const std::array<leg_controller_parameters,4> & leg_params);
  private:

  void encoderCallback(const sensor_msgs::JointStateConstPtr & JointState);
  void userInterfaceCallback(const olympus_control::leg_goal::ConstPtr & setPoint);

  leg_controller fr_controller;
  leg_controller rr_controller;
  leg_controller fl_controller;
  leg_controller rl_controller;

  std::array<leg_controller * ,4> controllers;

  //ROS
  ros::NodeHandle n;
  ros::Subscriber encoder_sub;
  ros::Subscriber controller_interface;


};

olympus_controller::olympus_controller(const std::array<leg_controller_parameters,4> & leg_params ): 
fr_controller(leg_params[0]),
rr_controller(leg_params[1]),
fl_controller(leg_params[2]),
rl_controller(leg_params[3])
{
  encoder_sub          = n.subscribe("/joint_states", 1000, &olympus_controller::encoderCallback, this);
  controller_interface = n.subscribe("/goal_set", 1000, &olympus_controller::userInterfaceCallback, this);

  controllers[0] = &fr_controller;
  controllers[1] = &rr_controller;
  controllers[2] = &fl_controller;
  controllers[3] = &rl_controller;


  //publish spheres
  fr_controller.Do_publish_sphere_target(true);
  rr_controller.Do_publish_sphere_target(true);
  fl_controller.Do_publish_sphere_target(true);
  rl_controller.Do_publish_sphere_target(true);
}

void olympus_controller::encoderCallback(const sensor_msgs::JointStateConstPtr & JointState){
  //use pointers to efficiently traverse the joint state, as i know its structure
  auto it_p = JointState->position.begin();
  auto it_v = JointState->velocity.begin();


  if( JointState->position.size() >=12 ){
    for (int i = 0; i< 4; ++i){
      controllers[i]->set_state(
        Eigen::Vector3d({*it_p,*(++it_p),*(++it_p)}),
        Eigen::Vector3d({*it_v,*(++it_v),*(++it_v)}));

      ++it_p; 
      ++it_v;
    }
  }

}

void olympus_controller::userInterfaceCallback(const olympus_control::leg_goal::ConstPtr & setPoint){
  const double & x1 = setPoint->x1;
  const double & x2 = setPoint->x2;
  const double & x3 = setPoint->x3;
  const uint16_t & controller_id = setPoint->controller_id; 


  if (!setPoint->IK_based_command){
    controllers[controller_id]->set_command(std::vector<double>{x1,x2,x3});
    ROS_INFO("[Controller]: New set point: [%.3lf,%.3lf,%.3lf] for controller %d", x1,x2,x3, controller_id);
  } else {
    Eigen::Vector3d QD = controllers[controller_id]->IK_querry(Eigen::Vector3d({x1,x2,x3}));
    controllers[controller_id]->set_command(QD);
    ROS_INFO("[Controller]: New set point: [%.3lf,%.3lf,%.3lf] for controller %d", QD[0],QD[1],QD[2], controller_id);

  }
  
  // if (controllers[controller_id]->publishing_sphere_target){ leg_control.publish_sphere_target(); }
  controllers[controller_id]->publish_sphere_target();
  controllers[controller_id]->publishCommand();
}

//user_interface
//Control_allocation


int main(int argc, char **argv){

  ros::init(argc, argv, "Controller_placeholder");

  double DX = 0.1485;
  double DY_f = 0.105;
  double DY_r = 0.15;
  // double DZ = -0.0056;
  double DZ = 0;

  tf2::Transform T_B_MH_fr = set_tf_transform({ DX,-DY_f,DZ},{-M_PI_2,   0,0});
  tf2::Transform T_B_MH_rr = set_tf_transform({-DX,-DY_r,DZ},{-M_PI_2,M_PI,0});
  tf2::Transform T_B_MH_fl = set_tf_transform({ DX, DY_f,DZ},{ M_PI_2,   0,0});
  tf2::Transform T_B_MH_rl = set_tf_transform({-DX, DY_r,DZ},{ M_PI_2,M_PI,0});

  // tf2::Transform T_W_B = set_tf_transform({0,0,1},{ 0,0,0});
  tf2::Transform T_W_B = set_tf_transform({0,0,0},{ 0,0,0});

  leg_controller_parameters fr_params(leg_index::fr);
  leg_controller_parameters rr_params(leg_index::rr);
  leg_controller_parameters fl_params(leg_index::fl);
  leg_controller_parameters rl_params(leg_index::rl);

  fr_params.tf_B_MH = T_W_B*T_B_MH_fr;
  rr_params.tf_B_MH = T_W_B*T_B_MH_rr;
  fl_params.tf_B_MH = T_W_B*T_B_MH_fl;
  rl_params.tf_B_MH = T_W_B*T_B_MH_rl;

  const std::array<leg_controller_parameters,4> & leg_controller_params{fr_params,rr_params,fl_params,rl_params};

  olympus_controller controller(leg_controller_params);
  
  ros::Rate loop_rate(10);


  while( ros::ok() ){

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;


}