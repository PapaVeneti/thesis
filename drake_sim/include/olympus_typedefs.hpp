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