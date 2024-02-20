#include <iostream>
#include <eigen3/Eigen/Dense>


#include "position_controller.hpp"
#include <chrono>

#define DK_TESTS false
#define IK_TESTS true

int main() {
position_controller a(1); 
auto start = std::chrono::high_resolution_clock::now();

//DK TESTS:
if (DK_TESTS){
std::cout << "---{0,0,0}"<<std::endl; 
a.qm <<0,0,0;
a.DK();                    //~20microseconds
a.calculate_joint_angles(); // ~15 microseconds
std::cout <<  a.get_EE_position() <<std::endl; 
std::cout << a.get_joint_angles()<<std::endl;
std::cout << "---{1,-1,0}"<<std::endl; 
std::cout << a.DK({1,-1,0}) <<std::endl; 
std::cout << a.calculate_joint_angles({1,-1,0}) <<std::endl;
std::cout << "---{0,1,1}"<<std::endl; 
// a.DK({0,1,1});
// a.calculate_joint_angles({0,1,1});
// std::cout << a.get_EE_position()<<std::endl; 
// std::cout << a.get_joint_angles()<<std::endl;
// std::cout << "---{0,-1,1}"<<std::endl; 
// a.DK({0,-1,1});
// a.calculate_joint_angles({0,-1,1});
// std::cout << a.get_EE_position()<<std::endl; 
// std::cout << a.get_joint_angles()<<std::endl;
// std::cout << "---{0.21,0.23,-0.32}"<<std::endl; 
// a.DK({0.21,0.23,-0.32});
// a.calculate_joint_angles({0.21,0.23,-0.32});
// std::cout << a.get_EE_position()<<std::endl; 
// std::cout << a.get_joint_angles()<<std::endl;
} 

if (IK_TESTS){
  double angle[3] ;
  // std::cin >> angle[0] ;
  // std::cin >> angle[1] ;
  // std::cin >> angle[2] ;
  angle[0] = 0.75;
  angle[1] = 0; angle[2]=0;
  std::cout << "Input angles are:" <<std::endl;
  std::cout << angle[0] << ","<< angle[1] << ","<< angle[2] <<std::endl;


Eigen::Vector3d xd = a.DK({angle[0],angle[1],angle[2]});

std::cout << "EE position in {MH} frame is" <<std::endl;
std::cout <<  xd  <<std::endl;

std::cout << "IK angles are:" <<std::endl;

// std::cout << a.IK( xd) <<std::endl;
std::cout << a.IK( {-0.11247, -0.24176, -0.05970} ) <<std::endl;


// std::cout << a.IK( {0.087188,0.18217,0.031493}) <<std::endl;




}
auto stop = std::chrono::high_resolution_clock::now();
// Calculate the duration
auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
std::cout << "Total time: " << duration.count() << " microseconds" << std::endl;

return 0;
}
