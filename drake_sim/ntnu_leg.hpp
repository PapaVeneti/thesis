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


class ntnu_leg
{
private:
  /* data */
public:
  ntnu_leg(
    drake_builder & builder,
    drake_plant & plant,
    const bool right_side, 
    const drake_rigidBody & ParentBody, 
    const drake_tfd & TF_B_MH);
  ~ntnu_leg();
};

/// The constructor for the `ntnu_leg` class. 
/// @param[in] TF Transform defining the frame
/// @param[in] mescat_ptr raw pointer to the mescat instance.
/// @param[in] path_prefix Path prefix
/// Curently using setlinesegments (maybe `SetLine` is more optimal)
// ntnu_leg::ntnu_leg(const drake_rigidBody & ParentBody , const drake_tfd & TF_B_MH)
// {
// }

ntnu_leg::~ntnu_leg()
{
}



