#include "position_controller.hpp"

//calculate_joint_angles use pc1,pc2,p_EE_22d from current state -> thus faster for state estimation
//CANNOT BE USED FOR CONSISTENT INITIALIZATION
//I may want to know where a state gets me without going there.

//1. DK return Xw
//2. DK_update (update current cartesian position)
//3. calculate_joint_angles (must be used for consistent initial positions)
//4. update_current_joint_angles -> with conjuction of DK_update for fast state_estimation
// new features:
//5. IK return qd

//6. Set desired cartesian position
//7. Document functions

// testing
#include <iostream>

void position_controller::DK(const Eigen::Vector3d &JointPositionVector){
  double qMH = JointPositionVector[0] ; 
  double q11 = side_sign* JointPositionVector[1] + q11_offset; //Positive joint angles opposite from MH
  double q12 = side_sign* JointPositionVector[2] + q12_offset; 

  //circle centers:
  Eigen::Vector2d vc;    //distance vector from `p1c` and `p2c` circle centers
  Eigen::Vector2d vn;    //normal to distance vector `vc`

  p1c[0] = j11_Dx + l11*cos(q11);
  p1c[1] = j_Dy   + l11*sin(q11);

  p2c[0] = j12_Dx + l12*cos(q12);
  p2c[1] = j_Dy   + l12*sin(q12);

  //2. Distance vectors and normal to distance
  vc = p2c-p1c;
  double d = vc.norm(); //distance of circle centers `p1c` and `p2c`
  vc.normalize();             

  vn << -vc[1],vc[0];

  //3. Triangle solution: 
  //see https://www.petercollingridge.co.uk/tutorials/computational-geometry/circle-circle-intersections/
  double a,h;
  a = ( (d*d) + (l21*l21) - (l22*l22) )/( 2*d );
  h = sqrt( (l21*l21) - (a*a));

  //4. Calculate Intersection point in the rotated {MH} frame.
  Eigen::Vector3d p_MH = {0,0,z_MH_j11j21};
  p_EE_22d = p1c + (a*vc) + (h*vn);

  p_MH.head(2) = p_EE_22d; 
  //5. Rotate to the default {MH} frame = {MH} for q1=0
  pEE = rotate_x(-qMH)*p_MH;
};

void position_controller::calculate_joint_angles(const Eigen::Vector3d &JointPositionVector){
  //The calculations are done in the rotated frame. 
  Eigen::Vector2d link11_v = p1c-pj11;
  Eigen::Vector2d link12_v = p2c-pj12;

  Eigen::Vector2d link21_v = p_EE_22d-p1c;
  Eigen::Vector2d link22_v = p_EE_22d-p2c;
  
  double theta1=   acos( ( link11_v.dot(link21_v) ) / (link11_v.norm() * link21_v.norm() )) ;
  double theta2=  -acos( ( link12_v.dot(link22_v) ) / (link12_v.norm() * link22_v.norm() )) ;

  //Check direction
  Eigen::Vector2d link11_vn =  link11_v.normalized();
  double side1_extended_x   = link11_vn[0]*(l11+l21);

  Eigen::Vector2d link12_vn =  link12_v.normalized();
  double side2_extended_x   = link12_vn[0]*(l12+l22);

  if ( p_EE_22d[0] <  side1_extended_x ) {
    // joint12 is outwards
    theta1 *= -1;
  }else if(p_EE_22d[0] >  side2_extended_x){
    // joint22 is outwards
    theta2 *= -1;
  }


  q[0] = JointPositionVector[0] ; 
  q[1] = JointPositionVector[1] ; //Positive joint angles opposite from MH
  q[2] = theta1 - q21_offset;
  q[3] = JointPositionVector[2] ; 
  q[4] = theta2 - q22_offset; 
}

Eigen::Matrix<double,5,1> position_controller::get_joint_angles(void){
  return q;
}

Eigen::Vector3d position_controller::get_EE_position(void){
  return pEE;
}

Eigen::Matrix3d position_controller::rotate_x(const double & th){
Eigen::Matrix3d rotm;
// rotm<<{ {1,0,0},{0, cos(th),-sin(th)},{0, sin(th),cos(th)} };
rotm<<  1,0,0,
        0, cos(th),-sin(th),
        0, sin(th),cos(th) ;

return rotm;
};


int main(int argc, char const *argv[])
{
position_controller a; 
Eigen::Vector3d d{ 1,2,3};
std::cout << "---{0,0,0}"<<std::endl; 
a.DK({0,0,0});
a.calculate_joint_angles({0,0,0});
std::cout << a.get_EE_position()<<std::endl; 
std::cout << a.get_joint_angles()<<std::endl;
std::cout << "---{1,-1,0}"<<std::endl; 
a.DK({1,-1,0});
a.calculate_joint_angles({1,-1,0});
std::cout << a.get_EE_position() <<std::endl; 
std::cout << a.get_joint_angles()<<std::endl;
std::cout << "---{0,1,1}"<<std::endl; 
a.DK({0,1,1});
a.calculate_joint_angles({0,1,1});
std::cout << a.get_EE_position()<<std::endl; 
std::cout << a.get_joint_angles()<<std::endl;
std::cout << "---{0,-1,1}"<<std::endl; 
a.DK({0,-1,1});
a.calculate_joint_angles({0,-1,1});
std::cout << a.get_EE_position()<<std::endl; 
std::cout << a.get_joint_angles()<<std::endl;
std::cout << "---{0.21,0.23,-0.32}"<<std::endl; 
a.DK({0.21,0.23,-0.32});
a.calculate_joint_angles({0.21,0.23,-0.32});
std::cout << a.get_EE_position()<<std::endl; 
std::cout << a.get_joint_angles()<<std::endl;
return 0;
}
