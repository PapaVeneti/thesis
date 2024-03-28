#include "leg_kinematics.hpp"
#include <cmath>

//state_estimation use pc1,pc2,p_EE_2d from current state -> thus faster for state estimation
//CANNOT BE USED FOR CONSISTENT INITIALIZATION
//I may want to know where a state gets me without going there.

//1. DK return Xw
//2. DK_update (update current cartesian position)
//3. state_estimation (must be used for consistent initial positions)
//4. update_current_joint_angles -> with conjuction of DK_update for fast state_estimation_AND_DK
// new features:
//5. IK return qd

//6. Set desired cartesian position
//7. Document functions

//explicitely generate the two IK functions:
template std::array<std::array<double,2>,2> leg_kinematics::RR_IK<1>(const Eigen::Vector2d & Pd);
template std::array<std::array<double,2>,2> leg_kinematics::RR_IK<2>(const Eigen::Vector2d & Pd);

// testing
#include <iostream>
#include <chrono>
#define DK_TESTS false
#define IK_TESTS true

double wrap_angle(double angle) {
    return std::fmod((angle + M_PI), (2 * M_PI)) - M_PI;
}

template <typename T>
void print_nested_array(const std::array<std::array<T,2>,2> & arr){
  std::cout << "[ [" << arr[0][0] << "," ; 
  std::cout          << arr[0][1] << "],[" ; 
  std::cout          << arr[1][0] << "," ; 
  std::cout          << arr[1][1] << "] ]" <<std::endl ; 
}

//Forward Kinematics and State estimation
#pragma region

//Forward Kinematics:
Eigen::Vector3d  leg_kinematics::DK(const Eigen::Vector3d &JointPositionVector){
          end_effector_2D(JointPositionVector);
  return  end_effector_MH(JointPositionVector[0]);
};
void leg_kinematics::DK(){
  pEE = DK( qm ) ;
}

//State Estimation:
Eigen::Matrix<double,5,1> leg_kinematics::state_estimation(const Eigen::Vector3d &JointPositionVector){
  end_effector_2D(JointPositionVector); //Updates `pc1`,`pc2`,`p_EE_2d`

  auto& pj11 = Pj11.segment<2>(0);
  auto& pj12 = Pj12.segment<2>(0);

  //The calculations are done in the rotated frame. 
  Eigen::Vector2d link11_v = p1c-pj11;
  Eigen::Vector2d link12_v = p2c-pj12;

  Eigen::Vector2d link21_v = p_EE_2d-p1c;
  Eigen::Vector2d link22_v = p_EE_2d-p2c;
  
  double theta1=   acos( ( link11_v.dot(link21_v) ) / (link11_v.norm() * link21_v.norm() )) ;
  double theta2=  -acos( ( link12_v.dot(link22_v) ) / (link12_v.norm() * link22_v.norm() )) ;

  //Check direction
  Eigen::Vector2d link11_vn =  link11_v.normalized();
  double side1_extended_x   = link11_vn[0]*(l11+l21);

  Eigen::Vector2d link12_vn =  link12_v.normalized();
  double side2_extended_x   = link12_vn[0]*(l12+l22);

  if ( p_EE_2d[0] <  side1_extended_x ) {
    // joint12 is outwards
    theta1 *= -1;
  }else if(p_EE_2d[0] >  side2_extended_x){
    // joint22 is outwards
    theta2 *= -1;
  }

  Eigen::Matrix<double,5,1> qvector;

  qvector[0] = JointPositionVector[0] ;
  qvector[1] = JointPositionVector[1] ; //Positive joint angles opposite from MH
  qvector[2] = theta1 - q21_offset;
  qvector[3] = JointPositionVector[2] ; 
  qvector[4] = theta2 - q22_offset; 

  return qvector;
}
void leg_kinematics::state_estimation(){
  q = state_estimation(qm);
}

// State Estimation and FK:
void leg_kinematics::state_estimation_AND_DK(const Eigen::Vector3d &JointPositionVector){
  state_estimation(JointPositionVector);
  pEE = end_effector_MH(JointPositionVector[0]);
}
void leg_kinematics::state_estimation_AND_DK(){
  state_estimation_AND_DK(qm);
}

// FK helpers:
void leg_kinematics::end_effector_2D(const Eigen::Vector3d &JointPositionVector){
  double q11,q12;

  //0. offset angles
  q11 = -config_sign*( JointPositionVector[1] -  offsets_1[joint_id::j11] );
  q12 = -config_sign*( JointPositionVector[2] -  offsets_1[joint_id::j12] );

  //1. circle centers:
  Eigen::Vector2d vc;    //distance vector from `p1c` and `p2c` circle centers
  Eigen::Vector2d vn;    //normal to distance vector `vc`

  p1c[0] =               Pj11[0] + l11*cos(q11);
  p1c[1] =  config_sign* Pj11[1] + l11*sin(q11);

  p2c[0] =               Pj12[0] + l12*cos(q12);
  p2c[1] =  config_sign* Pj12[1] + l12*sin(q12);

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
  p_EE_2d = p1c + (a*vc) + config_sign* (h*vn);
}
Eigen::Vector3d leg_kinematics::end_effector_MH(const double & qMH){
  Eigen::Vector3d p_MHrotated_EE = {0,0,z_MH_j11j21};
  p_MHrotated_EE.head(2) = p_EE_2d; 

  //Change to the default {MH} frame = {MH} for q1=0
  return  rotate_x(qMH)*p_MHrotated_EE;
}


#pragma endregion

//getters:

Eigen::Matrix<double,5,1> leg_kinematics::get_joint_angles(void){
  return q;
}
Eigen::Vector3d leg_kinematics::get_EE_position(void){
  return pEE;
}

//helpers
Eigen::Matrix3d leg_kinematics::rotate_x(const double & th){
Eigen::Matrix3d rotm;
// rotm<<{ {1,0,0},{0, cos(th),-sin(th)},{0, sin(th),cos(th)} };
rotm<<  1,0,0,
        0, cos(th),-sin(th),
        0, sin(th),cos(th) ;

return rotm;
};


// Inverse Kinematics
#pragma region


//maybe change to return bool
//as IK can fail
Eigen::Vector3d leg_kinematics::IK(const Eigen::Vector3d & pD_MH ){
  //A. Finding qmh
  const double & yd  = pD_MH[1];
  const double & zd  = pD_MH[2];
  constexpr double r = abs(z_MH_j11j21) ; 

  double temp_quant_1 = SQUARE(zd)+SQUARE(yd)-SQUARE(r); 
  //Error check 1
  if ( temp_quant_1 < 0 ){
    std::cerr << "[IK]: error in qmh -> feasibility check 1" <<std::endl;
    std::cerr << "[IK]: returning nan" <<std::endl;
  }

  double lambda = config_sign* sqrt(temp_quant_1); //plus
  double c3,s3;
  // determinant3 = SQUARE(r) + SQUARE(lambda); 
  // c3 =  (r*yd + lambda*zd) / determinant3;
  // s3 =  (r*zd - lambda*yd) / determinant3;
  c3 =  r*yd + lambda*zd;
  s3 =  r*zd - lambda*yd;
  double qMH = atan2(s3,c3);//-qMH_default_angle; 
  
  qMH -= offsets_1[joint_id::jMH];

  //Enforce limits:
  //For  the initial position, we match the y axis  of {MH} with the y axis of the RR plane ->
  //Thus offseting qMH ->
  // try {
  //   if ( qMH > qMH_ub || qMH< qMH_lb ) {
  //     throw(std::runtime_error("[IK]:Cannot rotate ab-ad to reach this possition") );
  //     }
  // }catch(const std::exception & e){
  //   std::cerr << e.what() << std::endl;
  //   return Eigen::Vector3d{0,0,0};
  // }

  //B. Transform Desired point from {MH} to {MHr}  
  //Project pD to RR plane:
  Eigen::Vector3d p_MH   = rotate_x(-qMH)*pD_MH; //Rotate pDesired 
  
  //Offset pD from joint positions -> Make IK and RR IK
  Eigen::Vector3d p_j1   = p_MH - Pj11;
  Eigen::Vector3d p_j2   = p_MH - Pj12;

  //Debug, to remove
  // std::cout << "Desired Position in {MH} frame:" <<std::endl;
  // std::cout << pD_MH <<std::endl;
  // std::cout << "Rotation Matrix:" <<std::endl;
  // std::cout << rotate_x(qMH) <<std::endl;
  //end of debug segment

  // Old working segment. BAD code: 
  // Eigen::Vector3d DP1{j11_Dx,abs( z_MH_j11j21) ,j_Dy};
  // Eigen::Vector3d DP2{j12_Dx,abs( z_MH_j11j21) ,j_Dy};
  // Eigen::Vector3d p_j1_old   = p_MH - rotate_x(-M_PI_2)*DP1;
  // Eigen::Vector3d p_j2_old   = p_MH - rotate_x(-M_PI_2)*DP2;
  // std::cout << "difference 1 is: " <<std::endl;
  // std::cout << p_j1 - p_j1_old <<std::endl;
  // std::cout << p_j2 - p_j2_old <<std::endl;
  // end of segment
  // std::cout << "Desired Position in Rotated {MH} frame:" <<std::endl;
  // std::cout << p_MH <<std::endl;
;
  //C. Get all RR IK solutions:
  // auto SOLS_1  = RR_IK(p_j1[0],p_j1[1],true); //Array { [q12,q22]_a,  [q12,q22]_b } 
  // auto SOLS_2  = RR_IK(p_j2[0],p_j2[1],false); //Array { [q12,q22]_a,  [q12,q22]_b } 

  auto SOLS_1  = RR_IK<1>(p_j1.head<2>()); //Array { [q12,q22]_a,  [q12,q22]_b } 
  auto SOLS_2  = RR_IK<2>(p_j2.head<2>()); //Array { [q12,q22]_a,  [q12,q22]_b } 

  //Debug:
  print_nested_array(SOLS_1);
  print_nested_array(SOLS_2);

  //D. check feasibility:
  feasibility_check(SOLS_1,SOLS_2);
  print_nested_array(feasibility_matrix);
  //Select best
  auto best_sol_index = RR_best_sol(SOLS_1,SOLS_2);

  double & q11 = SOLS_1[best_sol_index[0]][0]; //chain1 -> first hip joint
  double & q12 = SOLS_2[best_sol_index[1]][0]; //chain1 -> first hip joint
  // return Eigen::Vector3d{qMH-qMH_default_angle,q11,q12};
  return Eigen::Vector3d{qMH,q11,q12};


}

template <int chain>
std::array<std::array<double,2>,2> leg_kinematics::RR_IK(const Eigen::Vector2d & Pd){
  //Input references
  const double & x = Pd[0];
  const double & y = Pd[1];

  //Output defs
  std::array<double,2> solution1; //[theta1,theta2]
  std::array<double,2> solution2; //[theta1,theta2]

  //Aliases for convenience:
  double & theta1_a = solution1[0];
  double & theta2_a = solution1[1];
  double & theta1_b = solution2[0];
  double & theta2_b = solution2[1];

  //Function defs
  const double & r1 =  (chain == 1) ? l11 : l12;
  const double & r2 =  (chain == 1) ? l21 : l22;
  double theta1_offset;
  double theta2_offset;

  if constexpr (chain==1) {
    theta1_offset = offsets_1[joint_id::j11];
    theta2_offset = offsets_1[joint_id::j21];
  }else {
    theta1_offset = offsets_1[joint_id::j12];
    theta2_offset = offsets_1[joint_id::j22];
  }

  //Solve IK: 
  //a. Finding theta2:
  double temp_value = ( x*x +y*y -r1*r1 -r2*r2)/(2*r1*r2);

  if ( temp_value < -1 || temp_value > 1 ){
    std::cerr << "[IK]: error in theta2 -> feasibility check 2" <<std::endl;
    std::cerr << "[IK]: returning nan" <<std::endl;
  }

  theta2_a = acos(temp_value);
  theta2_b = -theta2_a;

  //b. Finding theta1:
  theta1_a = RR_IK_system(Pd,r1,r2,theta2_a);
  theta1_b = RR_IK_system(Pd,r1,r2,theta2_b);


  //Offsets:
  //q1i
  theta1_a =  theta1_offset -(config_sign)* theta1_a ;
  theta1_b =  theta1_offset -(config_sign)* theta1_b ;

  //q2i
  theta2_a = -theta2_offset -(config_sign)* theta2_a ;
  theta2_b = -theta2_offset -(config_sign)* theta2_b ;

  return std::array<std::array<double,2>,2> {solution1,solution2};
};


inline double leg_kinematics::RR_IK_system( const Eigen::Vector2d & Pd,
                                            const double r1,
                                            const double r2,
                                            const double & theta2){
  if (std::isnan(theta2)){ 
  // Check if theta2 is not a nan-> if nan-> return nan
    return std::nan("1");
  }                                            

  const double & x = Pd[0];
  const double & y = Pd[1];

  double k1,k2,c1,s1;
  k1 = r1+r2*cos(theta2);
  k2 =    r2*sin(theta2);
  // determinant = k1*k1 + k2*k2;
  // c1 = (k1*x+k2*y)/(determinant);
  // s1 = (k1*y-k2*x)/(determinant);

  c1 = k1*x+k2*y;
  s1 = k1*y-k2*x;

  return  atan2(s1,c1);
}

void  leg_kinematics::feasibility_check( 
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
  feasibility_matrix[1][0] = chain2_solutions[0];
  feasibility_matrix[1][1] = chain2_solutions[1];
}else{
  feasibility_matrix[1][0] = false;
  feasibility_matrix[1][1] = false;
}

};

std::array<int,2> leg_kinematics::RR_best_sol( 
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

inline double leg_kinematics::angle_distance(const std::array<double,2>& sol_chain1,const std::array<double,2>& sol_chain2,const Eigen::Matrix<double,5,1> qpos ){
  // double & q11,q21,q12,q22;
  // q11 = 
  return ANLGE_DIST(sol_chain1[0] - qpos[1],sol_chain1[0] - qpos[2],sol_chain2[0] - qpos[3],sol_chain2[0] - qpos[4]);
  
}

inline bool leg_kinematics::joint_angle_in_limits(const double & q, const  std::array<double,2> & qlim){
  return (q > qlim[1] || q< qlim[0])?  false : true;
};

inline bool leg_kinematics::solution_in_limits(const std::array<double,2> & sol, 
                                                    const std::array<double,2> & qa_lim, 
                                                    const std::array<double,2> & qb_lim)
{
  return ( joint_angle_in_limits(sol[0],qa_lim) && joint_angle_in_limits(sol[1],qb_lim) ) ? true : false; 
}

#pragma endregion
