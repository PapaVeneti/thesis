//Simulation
// #include <drake/math/rotation_matrix.h> //to set up first frame
#include "drake/systems/analysis/simulator.h"
#include "ntnu_leg.hpp"


//Visualization
#include "drake/visualization/visualization_config_functions.h" //Needed for AddDefaultVisualization
#include "drake/geometry/meshcat.h" //Access the visualizer (camera, recording etc)
using meshcat_shared_ptr = std::shared_ptr<drake::geometry::Meshcat>; 

//Optional includes
#include <iostream>
#include <fstream> //to export diagram
#include "drake_helpers.hpp"
#include "position_controller.hpp"

#define get_graph false
#define mb_time_step 0.0002 //in seconds!!!! -> must be as small as the simulation
#define integrator_time_step 1e-3
#define simulation_update_rate 1e-2 //GCD ( controller update rate, sensor update rate) (GREATER THAN mb_timestep)

// ros specific
#include "ros/ros.h"
// #include "std_msgs/String.h"
// #include "std_msgs/Float64.h"
// #include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h" //1st way

#include "rosgraph_msgs/Clock.h"
#include "sensor_msgs/JointState.h"

drake::systems::EventStatus publishClock(const ros::Publisher & pub, const drake::systems::Context<double> & context ){    
  rosgraph_msgs::Clock clock_msg;
  clock_msg.clock = ros::Time(context.get_time());
  pub.publish(clock_msg);

  return drake::systems::EventStatus::Succeeded();
}

struct sim_parameters {
  double realtime_rate = 1;
  bool publish_every_time_step = true;
  double sim_time = 5;
  double sim_update_rate = 0.01;
};


class simInterface{
public:
  simInterface(drake::systems::Simulator<double> & deployed_simulator,const drake::systems::InputPort<double> * system_input_port): 
  n(),
  simulator(deployed_simulator),
  input_port(system_input_port)
  {

    controllerSub  = n.subscribe("/position_controller", 1000, &simInterface::controllerCallback,this);
    encoderPub     = n.advertise<sensor_msgs::JointState>("/joint_states",10);

    // xd = InitialJointState.position[0];
  }

  inline Eigen::Vector3d get_command(void){
    std::vector<double> & v=  control_set_point.data;
    return Eigen::Vector3d({v[0],v[1],v[2]});
    }

  void encoderPublish(const sensor_msgs::JointState & CurrentJointState){
    joint_state = CurrentJointState;
    encoderPub.publish(joint_state);

  } 

  void set_Joint_states(const sensor_msgs::JointState & CurrentJointState){
    joint_state = CurrentJointState;
  }

private:

  void controllerCallback(const std_msgs::Float64MultiArray::ConstPtr& command){
    ROS_INFO("[simController]: New command: [%.3lf,%.3lf,%.3lf]", command->data[0],command->data[1],command->data[2]);
    control_set_point.data   = command->data; 
    control_set_point.layout = command->layout; 

    std::vector<double> & v=  control_set_point.data;
    // Eigen::Vector3d qd({v[0],v[1],v[2]});
    Eigen::Matrix<double,6,1> qd ;
    qd << v[0],v[1],v[2],0,0,0;

    auto& sim_context = simulator.get_mutable_context();
    input_port->FixValue(&sim_context,qd);

  }

  int num_controllers = 1;
  // std::vector< std_msgs::Float64MultiArray > control_msgs; 
  std_msgs::Float64MultiArray  control_set_point; 
  sensor_msgs::JointState joint_state;

  ros::NodeHandle n;
  ros::Subscriber controllerSub;
  ros::Publisher encoderPub;

  //drake connection
  const drake::systems::InputPort<double> * input_port;
  drake::systems::Simulator<double> & simulator;

};




int main(int argc, char **argv){

// Ros: node SETUP
#pragma region
// Node Set up 
ros::init(argc, argv, "drake_leg_simulation");
ros::NodeHandle n;
ros::Publisher clock_publisher = n.advertise<rosgraph_msgs::Clock>("/clock",10);

double loop_freq = 1/simulation_update_rate;
ros::Rate loop_rate(loop_freq); //HZ

#pragma endregion

// Drake: Plant set up
#pragma region
//0. Builder class to create the system
drake::systems::DiagramBuilder<double> builder; 

//1. Add plant and populate it with the robot instances
auto [plant,scene_graph]  = drake::multibody::AddMultibodyPlantSceneGraph(&builder,mb_time_step);

//1a. Instace of robot body (FOR NOW WORLD)
const drake_rigidBody &robot_base= plant.world_body();

  //create config structs for each leg
// drake_tfd frleg_TF( drake_rotMat::MakeXRotation(-M_PI/2), drake::Vector3<double>::UnitZ() ); //FR
drake_rotMat R_W_FR = drake_rotMat::MakeXRotation(-M_PI_2);
drake_rotMat R_FR_RR = drake_rotMat::MakeZRotation(-M_PI);
drake_tfd frleg_TF( R_W_FR* R_FR_RR, drake::Vector3<double>::UnitZ() ); //RR
leg_config config_rr(leg_index::rr, robot_base,frleg_TF);

//1b. Instance of each leg
ntnu_leg leg(builder,plant,config_rr); 
auto fr_leg = leg.get_leg_model_instance();


//1c. Finish plant
plant.set_discrete_contact_approximation(drake::multibody::DiscreteContactApproximation::kSap);
plant.Finalize();

auto collision_manager = scene_graph.collision_filter_manager();
collision_manager.Apply(
  drake::geometry::CollisionFilterDeclaration().
  ExcludeWithin(leg.get_shanks_collision())
);

//2. Wire up diagram
leg.connect_PID_system(builder,plant); //Connections happen after the plant is finalized

//2b. Add visualization (connect scene_graph and set_up meshcat)
meshcat_shared_ptr mescat_ptr =  std::make_shared<drake::geometry::Meshcat> ();
drake::visualization::AddDefaultVisualization(&builder,mescat_ptr);

//2c. Finish building. Never use builder again
auto diagram = builder.Build(); //Connections before here


if (get_graph){ get_system_graph(diagram.get());  }
#pragma endregion

//Simulation Set Up: 
#pragma region
drake::systems::Simulator sim(*diagram);

drake::systems::Simulator<double> & sim_reference  = sim; 

//a. Monitor Function:
auto bindedClockFun = std::bind(publishClock,clock_publisher,std::placeholders::_1);
// Create a function object
std::function<drake::systems::EventStatus(const drake::systems::Context<double> & )> monitor_func = bindedClockFun;
// Pass the monitor function to the simulator
sim.set_monitor(monitor_func);

//b simulation parameters
sim.set_publish_every_time_step(false); // MUST BE FALSE, ELSE OUT OF MEMORY VERY FAST
// sim.set_target_realtime_rate(2);  //no target, run as fast as possible
sim.get_mutable_integrator().set_maximum_step_size(integrator_time_step);
double sim_update_rate = simulation_update_rate; 
assert(sim_update_rate >= mb_time_step);
#pragma endregion


//Initialization
auto& context            = sim.get_mutable_context();
auto& plant_context      = diagram-> GetMutableSubsystemContext ( plant                       ,&context);
auto& controller_context = diagram-> GetMutableSubsystemContext ( *( leg.get_leg_controller()),&context);

// Set initial conditions
Eigen::VectorXd p0(5); 
Eigen::VectorXd v0(5); 
p0.setZero();
v0.setZero(); 
plant.SetPositions(&plant_context, fr_leg, p0);
plant.SetVelocities(&plant_context, fr_leg, p0);

//initial control keep it at the same position
Eigen::Matrix<double,6,1> qd;
qd.setZero();
diagram -> get_input_port(leg.get_controller_desired_state_port()).FixValue(&context,qd);



sensor_msgs::JointState JointState;
std::vector<std::string> joint_names;
for (auto joint_id : leg.get_plant().GetJointIndices(leg.get_leg_model_instance()) )
{
  joint_names.push_back( leg.get_plant().get_joint(joint_id).name() );
}
JointState.name = joint_names;
JointState.position.resize(5);
JointState.velocity.resize(5);

for (int i = 0; i < p0.size(); i++){
  JointState.position[i] = p0[i];
  JointState.velocity[i] = v0[i];
}
JointState.header.stamp = ros::Time(0);


const drake::systems::InputPort<double> & INPUT  = diagram -> get_input_port(leg.get_controller_desired_state_port());
simInterface simInterface(sim,&INPUT);
simInterface.set_Joint_states(JointState);

Eigen::Matrix<double,10,1> state_vector;

// Simulate:
sim.Initialize();
double sim_time = 0; 
while( ros::ok() ){ 
  // auto& sim_context = sim.get_mutable_context();

  //read joints
  state_vector = diagram-> get_output_port ( leg.get_leg_output_state_port() ).Eval(sim.get_context())  ;
  for (int i = 0; i < p0.size(); i++){
  JointState.position[i] = state_vector[i];
  JointState.velocity[i] = state_vector[i+5];  
  }
  JointState.header.stamp = ros::Time::now();
  simInterface.encoderPublish(JointState);

  //call callback to send commands:

  // diagram -> get_input_port(leg.get_controller_desired_state_port()).FixValue(&sim_context,qd);


  //simulation takes place:
  sim_time +=sim_update_rate;
  sim.AdvanceTo(sim_time);   

  // mescat_ptr -> Flush();

  ros::spinOnce(); //process callbacks
  if (!loop_rate.sleep()){ROS_DEBUG("Simulation is slower than realtime");} //returns false 
}

// while( sim_time < 1 ){


//     //simulation takes place:
//     sim_time +=sim_update_rate;
//     sim.AdvanceTo(sim_time);   

//     ros::spinOnce(); //process callbacks
//     if (!loop_rate.sleep()){ROS_WARN("Simulation is slower than realtime");} //returns false 
// }

// position_controller pos_controller(2) ;
// Eigen::Vector3d des_angles({1,1,1});
// qd << des_angles[0],des_angles[1],des_angles[2],0,0,0 ;
// diagram -> get_input_port(leg.get_controller_desired_state_port()).FixValue(&context,qd);
// drake_tfd T_MH_P(drake::math::RollPitchYawd(0,0,0),pos_controller.DK(des_angles));
// drake_tfd T_W_P = frleg_TF*T_MH_P; 
// auto goal1 = AddPoint(T_W_P,mescat_ptr.get(),"Goal Position");

// while( sim_time < 5 ){


//     //simulation takes place:
//     sim_time +=sim_update_rate;
//     sim.AdvanceTo(sim_time);   

//     ros::spinOnce(); //process callbacks
//     if (!loop_rate.sleep()){ROS_WARN("Simulation is slower than realtime");} //returns false 
// }

// mescat_ptr->Delete(goal1);
// while( sim_time < 7 ){


//     //simulation takes place:
//     sim_time +=sim_update_rate;
//     sim.AdvanceTo(sim_time);   

//     ros::spinOnce(); //process callbacks
//     if (!loop_rate.sleep()){ROS_WARN("Simulation is slower than realtime");} //returns false 
// }

//OLD CODE

// Set desired positions
// Eigen::Vector3d des_angles{-M_PI_2,0,0}; 
// Eigen::Vector3d des_angles{0,1.5,-1.5}; 
// Eigen::VectorXd qd(6); 
// qd<<  des_angles[0],des_angles[1],des_angles[2],0,0,0;
// diagram -> get_input_port(leg.get_controller_desired_state_port()).FixValue(&context,qd);

//Visualize the postion
// position_controller pos_controller(2) ;
// pos_controller.q = p0;
// pos_controller.DK(des_angles);

// frleg_TF -> the transform to the {MH} frame at qMH=0 (T_W_MH) 
// drake_tfd T_MH_P(drake::math::RollPitchYawd(0,0,0),pos_controller.DK(des_angles));
// drake_tfd T_W_P = frleg_TF*T_MH_P; 
// AddPoint(T_W_P,mescat_ptr.get(),"Goal Position");

// // 6.  Simulate:
// sim.Initialize();
// double sim_time = 0; 
// // mescat_ptr->StartRecording();
// while( sim_time < 2.5){
//     //realtime vis
//     sim_time +=sim_update_rate;
//     sim.AdvanceTo(sim_time);    
//     // (leg).FixValue(&plant_context,qd); //for pid joints
// }

// // auto& context2            = sim.get_mutable_context();
// // Eigen::Vector3d p_W = {0.087722,0.13204,0.63377};
// // drake_tfd T_W_Pnew(drake::math::RollPitchYawd(0,0,0),p_W);
// // AddPoint(T_W_Pnew,mescat_ptr.get(),"New Goal Position",drake::geometry::Rgba(1,0,0,1));
// // Eigen::Vector3d new_goal = T_W_MH.inverse()*p_W ; 
// // std::cout << "New goal position in {MH} frame is:" << std::endl;
// // std::cout << new_goal << std::endl;
// // des_angles = pos_controller.IK(new_goal);
// // qd<<  des_angles[0],des_angles[1],des_angles[2],0,0,0;
// // diagram -> get_input_port(leg.get_controller_desired_state_port()).FixValue(&context2,qd);

// // while( sim_time < 3){
// //     //realtime vis
// //     sim_time +=sim_update_rate;
// //     sim.AdvanceTo(sim_time);    
// //     // (leg).FixValue(&plant_context,qd); //for pid joints
// // }
// //playback
// mescat_ptr->PublishRecording();



  return 0;
}