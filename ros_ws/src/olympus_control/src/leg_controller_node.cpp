#include "ros/ros.h"

#include "leg_controller.hpp"


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