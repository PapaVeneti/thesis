#include "ntnu_leg.hpp" //For Legs
#include "drake/multibody/tree/ball_rpy_joint.h" //For base
#include "drake/multibody/tree/weld_joint.h" // (To be deleted) For base 

#define base_name "main_body" //name for both model instance, and body_name
#define weld_on true

using drake_rpy       = drake::math::RollPitchYawd ;


class olympus {
  public:		
    olympus(drake_builder & builder, const double & time_step_ );
    
    // void default_init(const drake::systems::System<double> & system);
    void default_init(drake::systems::Diagram<double> & diagram, drake::systems::Context<double> & context);
    
    //getters
    drake_plant & get_plant();
    std::vector<const drake::systems::InputPort<double> *> get_input_ports(drake::systems::Diagram<double> & diagram);
    std::vector<const drake::systems::OutputPort<double> *> get_output_ports(drake::systems::Diagram<double> & diagram);
    std::vector<std::string> get_joint_names();

    std::unique_ptr<ntnu_leg> fr_leg;  //pointer_as cannot be initialized initially
    std::unique_ptr<ntnu_leg> rr_leg;
    std::unique_ptr<ntnu_leg> fl_leg;
    std::unique_ptr<ntnu_leg> rl_leg;
  private:

    // robot builder functions:
    void add_body();
    void attach_legs(drake_builder & builder);
    void handle_collisions(drake::geometry::SceneGraph<double> & scene_graph);

    double time_step = 0.002;
    drake_plant * plant;
    drake::multibody::ModelInstanceIndex body_index;
    drake::geometry::GeometrySet body_collision;

    //Transforms for each leg
    drake_tfd frleg_TF;
    drake_tfd rrleg_TF;
    drake_tfd flleg_TF;
    drake_tfd rlleg_TF;

};