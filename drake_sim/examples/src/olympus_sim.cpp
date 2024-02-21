#include "olympus.hpp"
#include "drake/systems/analysis/simulator.h" //Simulation
#include "drake_helpers.hpp"

//Visualization
#include "drake/visualization/visualization_config_functions.h" //Needed for AddDefaultVisualization
#include "drake/geometry/meshcat.h" //Access the visualizer (camera, recording etc)
using meshcat_shared_ptr = std::shared_ptr<drake::geometry::Meshcat>; 

//1. DEFINE MB_TIME_STEP
//2. DEFINE base_NAME
//3. container for model instances



int main() {

    //Define simulation plant:
    drake::systems::DiagramBuilder<double> builder; // Class responsible from creating the dynamic system.
    olympus robot(builder,mb_time_step); //robot plant creation

    meshcat_shared_ptr meshcat_ptr =  std::make_shared<drake::geometry::Meshcat> (); //pointer to visualization class
    drake::visualization::AddDefaultVisualization(&builder,meshcat_ptr); //add visualization

    auto diagram = builder.Build(); //Connections before here
    
    get_system_graph(diagram.get()); 

    // Begin Simulation:

    drake::systems::Simulator sim(*diagram);
    auto& context = sim.get_mutable_context();
    sim.set_publish_every_time_step(true); // publish simulation as it happens
    sim.set_target_realtime_rate(1);

    // Set desired positions
    // Eigen::Vector3d des_angles{-M_PI_2,0,0}; 
    Eigen::Vector3d des_angles1{-M_PI_2,-3,0}; 
    Eigen::Vector3d des_angles2{ M_PI_2,-3,3}; 
    Eigen::VectorXd qd1(6);  
    Eigen::VectorXd qd2(6);  
    qd1<<  des_angles1[0],des_angles1[1],des_angles1[2],0,0,0;
    // qd2<<  des_angles2[0],des_angles2[1],des_angles2[2],0,0,0;
    qd2.setZero();
    diagram -> get_input_port(robot.fr_leg->get_controller_desired_state_port()).FixValue(&context,qd1);
    diagram -> get_input_port(robot.rr_leg->get_controller_desired_state_port()).FixValue(&context,qd2);
    diagram -> get_input_port(robot.fl_leg->get_controller_desired_state_port()).FixValue(&context,qd2);
    diagram -> get_input_port(robot.rl_leg->get_controller_desired_state_port()).FixValue(&context,qd1);



    sim.Initialize();
    double sim_time = 0; 
    double sim_update_rate = 0.01; 
    assert(sim_update_rate >= mb_time_step);
    meshcat_ptr->StartRecording();
    while( sim_time < 2.5){
    //realtime vis
    sim_time +=sim_update_rate;
    sim.AdvanceTo(sim_time);    
    }
    //playback
    meshcat_ptr->PublishRecording();

    return 0;
}


