#include "olympus.hpp"
//Simulation
// #include <drake/math/rotation_matrix.h> //to set up first frame
#include "drake/systems/analysis/simulator.h"

int main() {
    drake::systems::DiagramBuilder<double> builder; 
    meshcat_shared_ptr mescat_ptr =  std::make_shared<drake::geometry::Meshcat> ();

    olympus robot(builder,mescat_ptr,mb_time_step);

    auto diagram = builder.Build(); //Connections before here

    drake::systems::Simulator sim(*diagram);
    auto& context = sim.get_mutable_context();
    sim.set_publish_every_time_step(true); // publish simulation as it happens
    sim.Initialize();
    sim.set_target_realtime_rate(1);

    double sim_time = 0; 
    double sim_update_rate = 0.01; 
    assert(sim_update_rate >= mb_time_step);
    mescat_ptr->StartRecording();
    while( sim_time < 2.5){
    //realtime vis
    sim_time +=sim_update_rate;
    sim.AdvanceTo(sim_time);    
    }
    //playback
    mescat_ptr->PublishRecording();

    return 0;
}