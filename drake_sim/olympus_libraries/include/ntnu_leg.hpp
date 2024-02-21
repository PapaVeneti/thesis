//Drake pre-reqs:
#include "drake/systems/framework/diagram_builder.h"

//Drake Multibody
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include <drake/multibody/tree/revolute_joint.h>                //to add revolute joint ({MH})
#include <drake/multibody/tree/linear_bushing_roll_pitch_yaw.h> // Add spring and bushing
#include <drake/multibody/tree/linear_spring_damper.h>
#include <drake/math/rigid_transform.h> //to set up first frame
#include "drake/systems/controllers/pid_controller.h"//Pid Controller



//Better in a file for typedefs
using drake_tfd       = drake::math::RigidTransform<double>;
using drake_rotMat    = drake::math::RotationMatrix<double>;
using drake_rigidBody = drake::multibody::RigidBody<double> ; 
using drake_plant     = drake::multibody::MultibodyPlant<double>;
using drake_builder   = drake::systems::DiagramBuilder<double>;

enum class leg_index {fr, fl, rr, rl};
enum leg_states {qMH, q11, q21, q12, q22, vMH, v11, v21, v12, v22};

class leg_names {
  public: 
  const static std::map<leg_index,std::string> full;
  const static std::map<leg_index,std::string> suffix;

  leg_names() = delete ;
  
};

struct controllerGains {
  Eigen::Vector3d Kp = {10,10,10};
  Eigen::Vector3d Kd = {1,1,1};
  Eigen::Vector3d Ki = {1,1,1};
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


///This struct contains the leg parameters (id, parent body, transform from parent, 
///PID gains, Bushing and Spring parameters). 
///The user MUST specify the following:
/// @param[in] leg_id from [`fr,fl,rr,rl`] in the leg_index name space (e.g. `leg_index::fr`)
/// @param[in] ParentBody  //Parent body this leg is connected to
/// @param[in] TF_B_MH //Transform from Parent body to {MH} frame (MotorHousing)
///
/// @note  The other parameters have default values. If they need to change, here it the place. The should 
///not be changed in the class
struct leg_config{
  const leg_index      leg_id;             //Select from {leg_index::fr,fl,rr,rl}
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
/// The constructor for the `ntnu_leg` class. 
/// @param[in] builder builder reference 
/// @param[in] created_plant create plant reference. This populates the `plant` attribute of the class
/// @param[in] config Leg paramters
  ntnu_leg( 
    drake_builder & builder,
    drake_plant   & created_plant, 
    const leg_config & config);

  //helper funcs 
  void visualize_leg_frame(); 

  ///This function is called to connect the leg with a Pid controller.
  ///@note must be called after `plant.Finalize()`
  void connect_PID_system(drake_builder & builder, drake_plant & plant);

  //getters
  drake_plant & get_plant();
  drake::systems::controllers::PidController<double> *  get_leg_controller();
  drake::systems::InputPortIndex  get_controller_desired_state_port();
  drake::systems::OutputPortIndex get_leg_output_state_port();
  drake::multibody::ModelInstanceIndex get_leg_model_instance();
  drake::geometry::GeometrySet get_shanks_collision();
  drake::geometry::GeometrySet get_MH_collision();

private:
  const leg_index leg_id;
  
  //Drake specific:
  drake_plant & plant; //Reference to multibody plant. 
  drake::systems::controllers::PidController<double>* controller =nullptr; //Controller system pointer (returned by `AddSystem`, so no ref as no prir existance!)
  
  //Plant entities indices: (no consts as cannot be prior initialized, no ref/ptr as initially local var)
  drake::multibody::ModelInstanceIndex leg;                      // leg model instace
  drake::systems::InputPortIndex  controller_desired_state_port; // [qMH,q11,q12,vMH,v11,v12]_d
  drake::systems::OutputPortIndex leg_output_state_port;         // [q,v]

  //Collision
  drake::geometry::GeometrySet shanks_collision_set; 
  drake::geometry::GeometrySet MotorHousing_collision_set; 

  //MISSING PLANT ELEMENTS
  //1. JointActuators
  //2. bushingJoint
  //2. linearSpring
  
  //Simulation parameters:
  const controllerGains     &Gains;         //PID gains         (const as one time use in building)
  const BushingParamsStruct &BushingParams; //BushingParameters (const as one time use in building)
  const SpringParamsStruct  &SpringParams;  //SpringParameters  (const as one time use in building)

  //Private member functions:
  inline void add_bushing_joint();
  inline void add_linear_spring();
  void add_collision_sets();
  inline void add_actuators();
  inline void add_PID_system(drake_builder & builder, drake_plant & plant);


};




