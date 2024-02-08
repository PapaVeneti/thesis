//Drake pre-reqs:
#include "drake/systems/framework/diagram_builder.h"

//Drake Multibody
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include <drake/multibody/tree/revolute_joint.h> //to add revolute joint
#include <drake/multibody/tree/linear_bushing_roll_pitch_yaw.h> // Add spring and bushing
#include <drake/multibody/tree/linear_spring_damper.h>
#include <drake/math/rigid_transform.h> //to set up first frame
// #include <drake/math/rotation_matrix.h> //to set up first frame


//Better in a file for typedefs
using drake_tfd       = drake::math::RigidTransform<double>;
using drake_rotMat    = drake::math::RotationMatrix<double>;
using drake_rigidBody = drake::multibody::RigidBody<double> ; 
using drake_plant     = drake::multibody::MultibodyPlant<double>;
using drake_builder   = drake::systems::DiagramBuilder<double>;

#define mb_time_step 0.0002 //in seconds!!!! -> must be as small as the simulation
enum class leg_index {fr, fl, rr, rl};

class leg_names {
  public: 
  const static std::map<leg_index,std::string> full;
  const static std::map<leg_index,std::string> suffix;

  leg_names() = delete ;
  
};

struct controllerGains {
  drake::multibody::PdControllerGains gains_MH{10,1};
  drake::multibody::PdControllerGains gains_joint11{10,1};
  drake::multibody::PdControllerGains gains_joint21{10,1};
};

struct BushingParamsStruct {
  drake::Vector3<double> torque_stiffness_constants{100,100,0} ;
  drake::Vector3<double> torque_damping_constants{20,20,0};
  drake::Vector3<double> force_stiffness_constants{100,100,0};
  drake::Vector3<double> force_damping_constant{20,20,0};
  // Eigen::Vector3d{5000,5000,0},
  // Eigen::Vector3d{200,200,0},
  // Eigen::Vector3d{20000,20000,2000},
  // Eigen::Vector3d{200,200,50} //Andreas
};

struct SpringParamsStruct {
  double free_length = 0.175;
  double stiffness   = 1;
  double damping     = 10;
};

struct leg_config{
  const leg_index       & leg_id;             //Select from {leg_index::fr,fl,rr,rl}
  const drake_rigidBody & ParentBody ;      //Parent body this leg is connected to
  const drake_tfd       & TF_B_MH;          //Transform from Parent body to MH frame
  controllerGains     Gains;                //They have default values + optional set (thus no const)
  BushingParamsStruct BushingParams;        //They have default values + optional set (thus no const)
  SpringParamsStruct  SpringParams;         //They have default values + optional set (thus no const)

  leg_config(const leg_index & index,const drake_rigidBody & Base,const drake_tfd & TF ): 
  leg_id(index),  
  ParentBody(Base),
  TF_B_MH(TF){};
};


class ntnu_leg
{
public:
  ntnu_leg( 
    drake_builder & builder,
    drake_plant & created_plant, 
    const leg_config & config);

  
  void visualize_leg_frame(); //NOT IMPLEMENTED
  
  void set_leg_gains(const controllerGains & newGains);
  void set_bushing_params(const BushingParamsStruct & newBushingParams);
  void set_spring_params(const SpringParamsStruct & newSpringParams);

private:
  const leg_index leg_id;
  
  //Drake specific:
  drake_plant & plant;
  drake::multibody::ModelInstanceIndex leg;
  
  //Simulation parameters:
  controllerGains Gains;             //They have default values
  BushingParamsStruct BushingParams; //They have default values
  SpringParamsStruct SpringParams;   //They have default values

  //Private member functions:
  inline void add_bushing_joint();
  inline void add_linear_spring();
  inline void add_actuators();

};

/// The constructor for the `ntnu_leg` class. 
/// @param[in] TF Transform defining the frame
/// @param[in] mescat_ptr raw pointer to the mescat instance.
/// @param[in] path_prefix Path prefix




