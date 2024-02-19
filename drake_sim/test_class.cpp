#include <iostream>
#include <eigen3/Eigen/Dense>

class myClass {
  public:		
    myClass(const bool & config_1 ,const double & a): 
      is_config_1(config_1),
      Pj11(pop_j11(a,config_1))
      {
      ;
    }

  Eigen::Vector3d get_PJ() const {
    return Pj11;
  }
  Eigen::Vector3d pop_j11(const double & a,const bool & config_1){
    if (config_1) { 
         return Eigen::Vector3d(a,2*a,3*a); 
    }
    return Eigen::Vector3d(-a,-2*a,-3*a); 
  };

  private:

  // Eigen::Vector3d pop_j11(const double & a){
  //   if (is_config_1) { 
  //        return Eigen::Vector3d(a,2*a,3*a); 
  //   }
  //   return Eigen::Vector3d(-a,-2*a,-3*a); 
  // };
  

  const Eigen::Vector3d Pj11 ; 
  const bool is_config_1 ; 
  
};






int main() {
  double k = 1;
  myClass a(false,k);
  std::cout << a.get_PJ() << std::endl;
  std::cout << a.pop_j11(k,true) << std::endl;


  return 0;

}
