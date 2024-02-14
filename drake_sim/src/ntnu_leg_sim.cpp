//Simulation
#include <drake/math/rotation_matrix.h> //to set up first frame
#include "drake/systems/analysis/simulator.h"

//Visualization
#include "drake/visualization/visualization_config_functions.h" //Needed for AddDefaultVisualization
#include "drake/geometry/meshcat.h" //Access the visualizer (camera, recording etc)
using meshcat_shared_ptr = std::shared_ptr<drake::geometry::Meshcat>; 

#include <iostream>
#include <fstream> //to export diagram

#include "ntnu_leg.hpp"

//Interface with ros:
// #include "ros/ros.h"

// #include "rosgraph_msgs/Clock.h"
// #include "sensor_msgs/JointState.h"

//control:
#include "drake/systems/controllers/pid_controller.h"

struct sim_parameters {
  double realtime_rate = 1;
  bool publish_every_time_step = true;
  double sim_time = 5;
  double sim_update_rate = 0.01;
};


// drake::systems::EventStatus publishClock(const ros::Publisher & pub, const drake::systems::Context<double> & context ){    
    
//     rosgraph_msgs::Clock clock_msg;
//     clock_msg.clock = ros::Time(context.get_time());
//     pub.publish(clock_msg);

//     return drake::systems::EventStatus::Succeeded();
// }




int main(int argc, char **argv){

// Ros: node SETUP
#pragma region
// // Node Set up 
// ros::init(argc, argv, "drake simulation");
// ros::NodeHandle n;
// ros::Publisher clock_publisher = n.advertise<rosgraph_msgs::Clock>("/clock",10);

// double loop_freq = 100;
// ros::Rate loop_rate(loop_freq); //HZ

#pragma endregion

// Drake: Plant set up
#pragma region
//0. Builder class to create the system
drake::systems::DiagramBuilder<double> builder; 

//1. Add plant and populate it with the robot instances
auto [plant,scene_graph]  = drake::multibody::AddMultibodyPlantSceneGraph(&builder,mb_time_step);

//1a. Instace of robot body (FOR NOW WORLD)
const drake_rigidBody &robot_base= plant.world_body();


//TEMPORARY: create config structs for each leg
drake_tfd frleg_TF( drake_rotMat::MakeXRotation(-M_PI/2), drake::Vector3<double>::UnitZ() ); //FR
// drake_tfd rrleg_TF( drake_rotMat::MakeXRotation(-M_PI/2), drake::Vector3<double>{-1,0,1});

leg_config config_fr(leg_index::fr, robot_base,frleg_TF);

//1b. Instance of each leg
ntnu_leg(builder,plant,config_fr); 
auto fr_leg = plant.GetModelInstanceByName("fr_leg");

// ntnu_leg(builder,plant,leg_index::rr, robot_base,rrleg_TF);

//3
//3a. Finish plant
plant.set_discrete_contact_approximation(drake::multibody::DiscreteContactApproximation::kSap);
plant.Finalize();

//3b. Add visualization (connect scene_graph and set_up meshcat)
meshcat_shared_ptr mescat_ptr =  std::make_shared<drake::geometry::Meshcat> ();
drake::visualization::AddDefaultVisualization(&builder,mescat_ptr);

//3c. Add controller
Eigen::Matrix<double,6,10> ControlProjectionMatrix ; 
ControlProjectionMatrix.setZero();
//positions
ControlProjectionMatrix(0,0) = 1;
ControlProjectionMatrix(1,1) = 1;
ControlProjectionMatrix(2,3) = 1;
//gen velocities
ControlProjectionMatrix(3,5) = 1;
ControlProjectionMatrix(4,6) = 1;
ControlProjectionMatrix(5,8) = 1;

std::cout << plant.get_state_output_port(fr_leg).size() <<std::endl;
std::cout << plant.get_actuation_input_port(fr_leg).size() <<std::endl;
std::cout << plant.get_actuation_input_port(fr_leg).get_name() <<std::endl;

// auto joint_input_port  = builder.ExportInput(plant.get_actuation_input_port(fr_leg) ,"Joint Inputs");
// auto plant_output_port = builder.ExportOutput(plant.get_state_output_port(fr_leg),"plant_state");

drake::MatrixX<double> M(ControlProjectionMatrix);
// Eigen::Vector3d Kp = {100,150,150};
// Eigen::Vector3d Kd = {3,2.7,2.7};
// Eigen::Vector3d Ki = {1,1,1};
Eigen::Vector3d Kp = {10,10,10};
Eigen::Vector3d Kd = {1,1,1};
Eigen::Vector3d Ki = {1,1,1};

drake::systems::controllers::PidController<double>* controller = 
builder.AddNamedSystem<drake::systems::controllers::PidController<double>>("controller",
                                                                            M,
                                                                            Kp,
                                                                            Kd,
                                                                            Ki);

builder.Connect(controller -> get_output_port_control(), plant.get_actuation_input_port(fr_leg) );
builder.Connect(plant.get_state_output_port(fr_leg),controller->get_input_port_estimated_state());
auto controller_desired_state = builder.ExportInput(controller -> get_input_port_desired_state(),"fr_leg_setpoint");
auto plant_output = builder.ExportOutput(plant.get_state_output_port(fr_leg),"fr_leg_state");


//3d. Finish building. Never use builder again
auto diagram = builder.Build(); //Connections before here
#pragma endregion

bool export_graph = true;
// Write graph to file:
#pragma region 
if (export_graph){
std::string GraphString = diagram -> GetGraphvizString(1);
std::string filePath = "graph.dot";

    // Create an output file stream
    std::ofstream outFile(filePath);

    // Check if the file stream is open
    if (outFile.is_open()) {
        // Write the string to the file
        outFile << GraphString;

        // Close the file stream
        outFile.close();

        std::cout << "String successfully written to file: " << filePath << std::endl;
    } else {
        std::cerr << "Error opening the file: " << filePath << std::endl;
    }
}


#pragma endregion

//4. Simulation initialization: 
drake::systems::Simulator sim(*diagram);

// //Set monitoring function
// auto bindedClockFun = std::bind(publishClock,clock_publisher,std::placeholders::_1);
// // Create a function object
// std::function<drake::systems::EventStatus(const drake::systems::Context<double> & )> monitor_func = bindedClockFun;
// // Pass the monitor function to the simulator
// simulator.set_monitor(monitor_func);

auto& context = sim.get_mutable_context();
auto& plant_context = plant.GetMyMutableContextFromRoot(&context);
auto& controller_context = controller->GetMyMutableContextFromRoot(&context);


//4. Initialization
// Set initial conditions
Eigen::VectorXd p0(5); 
p0.setZero(); 
plant.SetPositions(&plant_context, fr_leg, p0);
plant.SetVelocities(&plant_context, fr_leg, p0);

// Set desired positions
Eigen::Vector3d des_angles{-M_PI_2,0,0}; //1 desired -> opposite from my code

Eigen::VectorXd qd(6); 
qd.setZero();
qd<<  des_angles[0],des_angles[1],des_angles[2],0,0,0;
// plant.get_desired_state_input_port(fr_leg).FixValue(&plant_context,qd); //for pid joints
diagram->get_input_port(controller_desired_state).FixValue(&context,qd);
qd<<  des_angles[0],des_angles[1],des_angles[2],0,0,0;
//6.  Simulate:
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
    // (leg).FixValue(&plant_context,qd); //for pid joints
}

auto& context2 = sim.get_mutable_context();
diagram->get_input_port(controller_desired_state).FixValue(&context2,qd);
while( sim_time < 5){
    //realtime vis
    sim_time +=sim_update_rate;
    sim.AdvanceTo(sim_time);    
    // (leg).FixValue(&plant_context,qd); //for pid joints
}


//playback
mescat_ptr->PublishRecording();



  return 0;
}