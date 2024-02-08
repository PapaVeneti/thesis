#include "position_controller.hpp"



 Eigen::Vector3d position_controller::DK(const Eigen::Vector3d &JointPositionVector){
    double qMH = JointPositionVector[0] ; 
    double q11 = side_sign* JointPositionVector[1] + q11_offset; //Positive joint angles opposite from MH
    double q12 = side_sign* JointPositionVector[2] + q12_offset; 

    //circle centers:
    Eigen::Vector2d p1,p2; //p1,p2 are circle centers
    Eigen::Vector2d vc;    //distance vector from `p1` and `p2` circle centers
    Eigen::Vector2d vn;    //normal to distance vector `vc`

    p1[0] = j11_Dx + l11*cos(q11);
    p1[1] = j_Dy   + l11*sin(q11);

    p2[0] = j12_Dx + l12*cos(q12);
    p2[1] = j_Dy   + l12*sin(q12);

    //2. Distance vectors and normal to distance
    vc = p2-p1;
    double d = vc.norm(); //distance of circle centers `p1` and `p2`
    vc.normalize();

    vn << -vc[1],vc[0];

    //3. Triangle solution: 
    //see https://www.petercollingridge.co.uk/tutorials/computational-geometry/circle-circle-intersections/
    double a,h;
    a = ( d*d + l21*l21 -l22*l22 )/( 2*d );
    h = sqrt(l21*l21 - a*a);

    Eigen::Vector3d p_MH{0,0,z_MH_j11j21};
    p_MH.head(2) = p1 + (a*vc) + (h*vn);

  return Eigen::Vector3d::Zero();
 };


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
  a.rotate_x(M_PI);
  return 0;
 }
 