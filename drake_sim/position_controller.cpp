#include "position_controller.hpp"
#include <cmath>

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
#define DK_TESTS false
#define IK_TESTS true

void print_nested_array(const std::array<std::array<double,2>,2> & arr){
  std::cout << "[ [" << arr[0][0] << "," ; 
  std::cout          << arr[0][1] << "],[" ; 
  std::cout          << arr[1][0] << "," ; 
  std::cout          << arr[1][1] << "] ]" <<std::endl ; 
}

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
  //5. Change to the default {MH} frame = {MH} for q1=0
  pEE = rotate_x(qMH)*p_MH;
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

//maybe change to return bool
//as IK can fail
Eigen::Vector3d position_controller::IK(const Eigen::Vector3d & pD_MH ){
  // double qMH = atan2(pD_MH[2],pD_MH[1]) - qMH_default_angle;

  //Find qMH angle:
  const double & yd = pD_MH[1];
  const double & zd = pD_MH[2];
  constexpr double r = abs(z_MH_j11j21) ; 

  double lambda = sqrt(SQUARE(zd)+SQUARE(yd)-SQUARE(r) ); //plus
  double c3,s3,determinant3;
  determinant3 = SQUARE(r) + SQUARE(lambda); 
  c3 =  (r*yd + lambda*zd) / determinant3;
  s3 =  (r*zd - lambda*yd) / determinant3;
  double qMH = atan2(s3,c3)-qMH_default_angle; 

  try {
    if ( qMH > qMH_ub || qMH< qMH_lb ) {
      throw(std::runtime_error("[IK]:Cannot rotate ab-ad to reach this possition") );
      }
  }catch(const std::exception & e){
    std::cerr << e.what() << std::endl;
    return Eigen::Vector3d{0,0,0};
  }

  //Project pD to RR plane:
  std::cout << pD_MH <<std::endl;
  std::cout << rotate_x(qMH) <<std::endl;
  Eigen::Vector3d p_MH   = rotate_x(-qMH)*pD_MH; //Rotate pDesired 

  std::cout << "Desired Position in Rotated {MH} frame:" <<std::endl;
  std::cout << p_MH <<std::endl;


  //Get all RRIK solutions
  auto SOLS_1  = RR_IK(p_MH[0]-j11_Dx,p_MH[1],true) ; //Array { [q11,q21]_a,  [q11,q21]_b }  
  auto SOLS_2  = RR_IK(p_MH[0]-j12_Dx,p_MH[1],false); //Array { [q12,q22]_a,  [q12,q22]_b } 

  print_nested_array(SOLS_1);
  print_nested_array(SOLS_2);

  //check feasibility:
  feasibility_check(SOLS_1,SOLS_2);
  //Select best
  auto best_sol_index = RR_best_sol(SOLS_1,SOLS_2);

  double & q11 = SOLS_1[best_sol_index[0]][0]; //chain1 -> first hip joint
  double & q12 = SOLS_2[best_sol_index[1]][0]; //chain1 -> first hip joint
  // return Eigen::Vector3d{qMH-qMH_default_angle,q11,q12};
  return Eigen::Vector3d{qMH,q11,q12};


}


std::array<std::array<double,2>,2> position_controller::RR_IK(const double &x,const double &y, const bool & chain1){
  std::array<double,2> solution1; //[theta1,theta2]
  std::array<double,2> solution2; //[theta1,theta2]

  double theta2;
  double r1;
  double r2;
  double theta1_offset;
  double theta2_offset;
  

  if (chain1){
    r1 = l11; r2 = l21;
    theta1_offset = q11_offset;
    theta2_offset = q21_offset;
  }else{
    r1 = l12; r2 = l22; 
    theta1_offset = q12_offset;
    theta2_offset = -q22_offset;
  }
  solution1[1] = acos(( x*x +y*y -r1*r1 -r2*r2)/(2*r1*r2));
  solution2[1] = -solution1[1];

  solution1[0] = RR_IK_system(x,y,r1,r2,solution1[1]);
  solution2[0] = RR_IK_system(x,y,r1,r2,solution2[1]);

  // solution1[0] *= -1; 
  // solution1[0] += theta1_offset;
  // solution1[0] = std::fmod(solution1[0],2*M_PI);
  // solution2[0] *= -1; 
  // solution2[0] += theta1_offset;
  // solution2[0] = std::fmod(solution2[0],2*M_PI);

  // solution1[1] *= -1; 
  // solution1[1] +=M_PI + theta2_offset;
  // solution1[1]  = std::fmod(solution1[1],2*M_PI);
  // solution2[1] *= -1; 
  // solution2[1] +=M_PI + theta2_offset;
  // solution2[1]  = std::fmod(solution2[1],2*M_PI);

  solution1[0] = theta1_offset -solution1[0] ;
  solution1[0] = std::fmod(solution1[0],2*M_PI);

  solution2[0] = theta1_offset -solution2[0] ;
  solution2[0] = std::fmod(solution2[0],2*M_PI);
  

  //theta 2
  std::cout<<solution1[1]<<std::endl;

  solution1[1]  = -theta2_offset - solution1[1] ;
  // solution1[1]  = std::fmod(solution1[1],2*M_PI);

  solution2[1]  = -theta2_offset - solution2[1] ;
  // solution2[1]  = std::fmod(solution2[1],2*M_PI);



  return std::array<std::array<double,2>,2> {solution1,solution2};
};

inline double position_controller::RR_IK_system(const double x,
                                                const double y,
                                                const double r1,
                                                const double r2,
                                                const double & theta2){
  
  double k1,k2,c1,s1,determinant;
  k1 = r1+r2*cos(theta2);
  k2 =    r2*sin(theta2);
  determinant = k1*k1 + k2*k2;

  c1 = (k1*x+k2*y)/(determinant);
  s1 = (k1*y-k2*x)/(determinant);

  return  atan2(s1,c1);
}

void  position_controller::feasibility_check( 
  const std::array<std::array<double,2>,2> & SOLS_1,
  const std::array<std::array<double,2>,2> & SOLS_2 ){


bool chain2_solutions[2];
chain2_solutions[0] = solution_in_limits(SOLS_2[0],q12_lim,q22_lim);
chain2_solutions[1] = solution_in_limits(SOLS_2[1],q12_lim,q22_lim);

//solution 1 for chain1, solutions 1,2 for chain2

if ( solution_in_limits(SOLS_1[0],q11_lim,q21_lim) ) {
  feasibility_matrix[0][0] = chain2_solutions[0];
  feasibility_matrix[0][1] = chain2_solutions[1];
}else{
  feasibility_matrix[0][0] = false;
  feasibility_matrix[0][1] = false;
}

//solution 2 for chain1, solutions 1,2 for chain2

if ( solution_in_limits(SOLS_1[1],q11_lim,q21_lim) ) {
  feasibility_matrix[0][0] = chain2_solutions[0];
  feasibility_matrix[0][1] = chain2_solutions[1];
}else{
  feasibility_matrix[0][0] = false;
  feasibility_matrix[0][1] = false;
}

};

std::array<int,2> position_controller::RR_best_sol( 
  const std::array<std::array<double,2>,2> & SOLS_1,
  const std::array<std::array<double,2>,2> & SOLS_2 ){
  std::array<int,2>  best_sol_index{0,0};
  double minimum_distance_squared = 10000;
  double current_distance = 10000;

  for (int sol_chain1=0; sol_chain1<2 ; sol_chain1++){
    for (int sol_chain2 = 0; sol_chain2 < 2 ; sol_chain2++){

      if ( feasibility_matrix[sol_chain1][sol_chain2] == 0 ) { continue;}

      double current_distance = angle_distance(SOLS_1[sol_chain1],SOLS_2[sol_chain2],q);
      if (current_distance < minimum_distance_squared){
        best_sol_index[0] = sol_chain1; best_sol_index[1] = sol_chain2; 
        minimum_distance_squared = current_distance; 
      }
    }
  }

  return best_sol_index; 
}

inline double position_controller::angle_distance(const std::array<double,2>& sol_chain1,const std::array<double,2>& sol_chain2,const Eigen::Matrix<double,5,1> qpos ){
  // double & q11,q21,q12,q22;
  // q11 = 
  return ANLGE_DIST(sol_chain1[0] - qpos[1],sol_chain1[0] - qpos[2],sol_chain2[0] - qpos[3],sol_chain2[0] - qpos[4]);
  
}

inline bool position_controller::joint_angle_in_limits(const double & q, const  std::array<double,2> & qlim){
  return (q > qlim[1] || q< qlim[0])?  false : true;
};

inline bool position_controller::solution_in_limits(const std::array<double,2> & sol, 
                                                    const std::array<double,2> & qa_lim, 
                                                    const std::array<double,2> & qb_lim)
{
  return ( joint_angle_in_limits(sol[0],qa_lim) && joint_angle_in_limits(sol[1],qb_lim) ) ? true : false; 
}



int main(int argc, char const *argv[])
{
position_controller a; 

//DK TESTS:
if (DK_TESTS){
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
} 

if (IK_TESTS){
//   double angle[3] ;
//   // std::cin >> angle[0] ;
//   // std::cin >> angle[1] ;
//   // std::cin >> angle[2] ;
//   angle[0] = 0.5;
//   angle[1] = 0; angle[2]=0;
//   std::cout << "Input angles are:" <<std::endl;
//   std::cout << angle[0] << ","<< angle[1] << ","<< angle[2] <<std::endl;

// a.DK({angle[0],angle[1],angle[2]});

// std::cout << "EE position in {MH} frame is" <<std::endl;
// std::cout <<  a.get_EE_position()  <<std::endl;

// std::cout << "IK angles are:" <<std::endl;

// std::cout << a.IK( a.get_EE_position() ) <<std::endl;
std::cout << a.IK( {0.088921,0.36555,0.13167} ) <<std::endl;
}

// std::cout<< std::fmod(-0.1,1) <<std::endl;
// std::cout<< std::fmod(-1.1,1) <<std::endl;
// std::cout<< std::fmod(1.1,1) <<std::endl;
// std::cout<< std::fmod(0.1,1) <<std::endl;

return 0;
}
