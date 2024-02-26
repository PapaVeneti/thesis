#include <iostream>
#include <leg_kinematics.hpp>
#include <chrono>

#define DK_TESTS false
#define IK_TESTS true

int main(int argc, char const *argv[])
{
leg_kinematics a(1); 
auto start = std::chrono::high_resolution_clock::now();

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
// std::cout << a.IK( {0.088921,0.36555,0.13167} ) <<std::endl;


// std::cout << a.IK( {0.087188,0.18217,0.031493}) <<std::endl;
std::cout << a.IK( {0.4,0.11,-0.2}) <<std::endl;


auto stop = std::chrono::high_resolution_clock::now();
// Calculate the duration
auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
std::cout << "IK single call takes: " << duration.count() << " microseconds" << std::endl;

}


return 0;
}
