#include <iostream>
#include <fstream> //to export diagram
//Drake pre-reqs:
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
//Drake Multibody
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include <drake/math/rigid_transform.h> //to set up first frame
#include <drake/math/rotation_matrix.h> //to set up first frame
#include <drake/multibody/tree/linear_bushing_roll_pitch_yaw.h> // Add spring and bushing
#include <drake/multibody/tree/linear_spring_damper.h>

//For controller.other systems
#include <drake/systems/primitives/constant_vector_source.h>

//Visualization
#include "drake/visualization/visualization_config_functions.h"
//Access the visualizer (camera, recording etc)
#include "drake/geometry/meshcat.h"

//Helpers
#include "drake_helpers.hpp"
#include "position_controller.hpp"

//robot class

#define export_graph false


#define mb_time_step 0.0002 //in seconds!!!! -> must be as small as the simulation
using meshcat_shared_ptr = std::shared_ptr<drake::geometry::Meshcat>; 
using drake_tfd = drake::math::RigidTransform<double>;
using drake_rotMat = drake::math::RotationMatrix<double>;

// How to use drake to have cleaner code
// gflags to set 1. sim time 2. set if print diagram





int main() {
drake::systems::DiagramBuilder<double> builder; 

// Add plant (and scene_graph)
#pragma region

auto [plant,scene_graph]  = drake::multibody::AddMultibodyPlantSceneGraph(&builder,mb_time_step);
drake::multibody::Parser parser(&plant);

//Load models
auto fl_leg = parser.AddModels("../ntnu_leg.urdf").at(0); //only one model // this adds one instance

//Add world joint (could be in the urdf instead)
//Correct version 
drake_tfd worldTF( drake_rotMat::MakeXRotation(-M_PI/2), drake::Vector3<double>::UnitZ() );

const drake::multibody::WeldJoint<double> & weld_joint =  
plant.AddJoint< drake::multibody::WeldJoint >("world_joint",
                                                plant.world_body(),
                                                worldTF,
                                                plant.GetBodyByName("MH"),
                                                drake_tfd(),
                                                drake_tfd::Identity());

//Add actuators (could be in the urdf)
auto& motor1 = plant.AddJointActuator("motor1",plant.GetJointByName("joint11"));
auto& motor2 = plant.AddJointActuator("motor2",plant.GetJointByName("joint12"));
auto& motor3 = plant.AddJointActuator("motor3",plant.GetJointByName("jointMH"));

drake::multibody::PdControllerGains motor1_gains{10,1};
drake::multibody::PdControllerGains motor2_gains{10,1};
drake::multibody::PdControllerGains motor3_gains{10,1};

plant.get_mutable_joint_actuator(motor1.index()).set_controller_gains(motor1_gains); 
plant.get_mutable_joint_actuator(motor2.index()).set_controller_gains(motor2_gains); 
plant.get_mutable_joint_actuator(motor3.index()).set_controller_gains(motor3_gains); 

//Add bushing element
double attachment_point_shank1_x =  -0.2846;
double attachment_point_shank1_y =   0.0121;
double attachment_point_shank2_x =   0.3035;
double attachment_point_shank2_y =  -0.0022;

auto& shank1_frame = plant.GetBodyByName("link21").body_frame();
auto& shank2_frame = plant.GetBodyByName("link22").body_frame();


auto& shank1_endpoint = plant.AddFrame(
    std::make_unique<drake::multibody::FixedOffsetFrame<double>>
    (
        "shank1_endpoint",
        shank1_frame,
        drake_tfd(Eigen::Translation3d{attachment_point_shank1_x,attachment_point_shank1_y,0})
    ) 
);

auto& shank2_endpoint = plant.AddFrame(
    std::make_unique<drake::multibody::FixedOffsetFrame<double>>
    (
        "shank2_endpoint",
        shank2_frame,
        drake_tfd(Eigen::Translation3d{attachment_point_shank2_x,attachment_point_shank2_y,0})
    ) 
);

auto&  bushing = plant.AddForceElement<drake::multibody::LinearBushingRollPitchYaw>(
    shank1_endpoint,
    shank2_endpoint,
    Eigen::Vector3d{100,100,0},
    Eigen::Vector3d{20,20,0},
    Eigen::Vector3d{100,100,0},
    Eigen::Vector3d{20,20,0}
    // Eigen::Vector3d{5000,5000,0},
    // Eigen::Vector3d{200,200,0},
    // Eigen::Vector3d{20000,20000,2000},
    // Eigen::Vector3d{200,200,50} //Andreas
);

// Add linear spring
auto&  spring = plant.AddForceElement<drake::multibody::LinearSpringDamper>(
    plant.GetBodyByName("link21"),
    drake::Vector3<double>::Zero(),
    plant.GetBodyByName("link22"),
    drake::Vector3<double>::Zero(),
    0.175,1,10
    //  const Body<T>& bodyA, const Vector3<double>& p_AP,
    //   const Body<T>& bodyB, const Vector3<double>& p_BQ,
    //   double free_length, double stiffness, double damping);
);


// TODO: Add constraint
// drake::systems::SystemConstraint<double> joint1_2_constr(
// ) ; 
// plant.AddExternalConstraint()

plant.set_discrete_contact_solver(drake::multibody::DiscreteContactSolver::kSap);

// Finilize plant
plant.Finalize();

#pragma endregion

// Add visualization (connect scene_graph and set_up meshcat)
meshcat_shared_ptr mescat_ptr =  std::make_shared<drake::geometry::Meshcat> ();
drake::visualization::AddDefaultVisualization(&builder,mescat_ptr);

auto diagram = builder.Build();

// Write graph to file:
#pragma region 
if (export_graph){
std::string GraphString = diagram -> GetGraphvizString(12);
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

//Simulate: 
drake::systems::Simulator sim(*diagram);
assert(  plant.get_discrete_contact_solver() ==  drake::multibody::DiscreteContactSolver::kSap );

// for (auto str : plant.GetPositionNames() )
// {
//     std::cout << str << std::endl;
// }
// for (auto joint_int : plant.GetActuatedJointIndices(fl_leg) )
// {
  
//   std::cout << plant.get_mutable_joint( joint_int).name() << std::endl;
// }

auto& context = sim.get_mutable_context();
auto& plant_context = plant.GetMyMutableContextFromRoot(&context);
// std::cout<< plant.GetPositionsAndVelocities(context) <<std::endl;
// std::cout<< plant.GetPositions(plant_context,fl_leg)<<std::endl;

// Eigen::Vector3d des_angles{1,0.32,0.7}; //1 desired -> opposite from my code
// Eigen::Vector3d des_angles{0.187721,0.32,0.7}; //1 desired -> opposite from my code
Eigen::Vector3d des_angles{0,0,M_PI/2}; //1 desired -> opposite from my code

// Set initial conditions
Eigen::VectorXd p0(5); 
p0 << des_angles[2],des_angles[0],0,des_angles[1],0;
// // plant.SetPositionsAndVelocities(plant_context,fl_leg,Eigen::VectorXd{0,0,0,M_PI,M_PI,0,0,0,0,0});
plant.SetPositions(
        &plant_context,
        fl_leg,
        p0);

//Set initial target
// plant.get_actuation_input_port(fl_leg).FixValue(&plant_context,Eigen::Vector3d{0.,0.,0.}); 
Eigen::VectorXd qd(6); qd<<  des_angles[0],des_angles[1],des_angles[2],0,0,0;
plant.get_desired_state_input_port(fl_leg).FixValue(&plant_context,qd); //for pid joints

position_controller pos_con;


//Test visualizing frames
#pragma region
// drake_tfd motor3TF = plant.GetBodyByName("motor3").body_frame().CalcPoseInWorld(plant_context);
// drake_tfd jointm3mh_childTF = plant.GetJointByName("motorHousingtomotor3").frame_on_child().CalcPoseInWorld(plant_context);
// drake_tfd jointm3mh_parentTF = plant.GetJointByName("motorHousingtomotor3").frame_on_parent().CalcPoseInWorld(plant_context);
// drake_tfd joint1_childTF = plant.GetJointByName("joint1").frame_on_child().CalcPoseInWorld(plant_context);
// drake_tfd joint1_parentTF = plant.GetJointByName("joint1").frame_on_parent().CalcPoseInWorld(plant_context);
// drake_tfd MH_TF = plant.GetBodyByName("motorHousing").body_frame().CalcPoseInWorld(plant_context);
// drake_tfd joint2_childTF = plant.GetJointByName("joint2").frame_on_child().CalcPoseInWorld(plant_context);
// drake_tfd joint2_parentTF = plant.GetJointByName("joint2").frame_on_parent().CalcPoseInWorld(plant_context);

// // drake_tfd Hip1TF = plant.GetBodyByName("hip1").body_frame().CalcPoseInWorld(plant_context);
// // drake_tfd Hip2TF = plant.GetBodyByName("hip2").body_frame().CalcPoseInWorld(plant_context);
// // drake_tfd shank1TF = plant.GetBodyByName("shank1").body_frame().CalcPoseInWorld(plant_context);
// // drake_tfd shank2TF = plant.GetBodyByName("shank2").body_frame().CalcPoseInWorld(plant_context);
// drake_tfd shank1_endTF = shank1_endpoint.CalcPoseInWorld(plant_context);


// std::cout << "Motor 3 TF: \n " << motor3TF.GetAsMatrix4() <<std::endl;
// std::cout << "Joint2 parent TF: \n " << joint1_childTF.GetAsMatrix4() <<std::endl;
// std::cout << "Joint2 parent TF: \n " << joint1_parentTF.GetAsMatrix4() <<std::endl;
// std::cout << "MH TF: \n " << MH_TF.GetAsMatrix4() <<std::endl;
// std::cout << "Hip1 TF: \n " << Hip1TF.GetAsMatrix4() <<std::endl;
// std::cout << "Hip2 TF: \n " << Hip2TF.GetAsMatrix4() <<std::endl;
// std::cout << "shank1 TF: \n " << shank1TF.GetAsMatrix4() <<std::endl;
// std::cout << "shank2 TF: \n " << shank2TF.GetAsMatrix4() <<std::endl;
// std::cout << "shank1_EE TF: \n "<< shank1_endTF.GetAsMatrix4() <<std::endl;

// VisualizeFrame(motor3TF,mescat_ptr.get(),"/Frames/motor3");
// VisualizeFrame(joint1_childTF,mescat_ptr.get(),"/Frames/joint1_child");
// VisualizeFrame(joint1_parentTF,mescat_ptr.get(),"/Frames/joint1_parent");
// VisualizeFrame(jointm3mh_childTF,mescat_ptr.get(),"/Frames/jointm3mh_child");
// VisualizeFrame(jointm3mh_parentTF,mescat_ptr.get(),"/Frames/jointm3mh_parent");
// VisualizeFrame(MH_TF,mescat_ptr.get(),"/Frames/motorHousing");
// VisualizeFrame(joint2_childTF,mescat_ptr.get(),"/Frames/joint2_child");
// VisualizeFrame(joint2_parentTF,mescat_ptr.get(),"/Frames/joint2_parent");
// // VisualizeFrame(Hip1TF,mescat_ptr.get(),"/Frames/hip1");
// // VisualizeFrame(Hip2TF,mescat_ptr.get(),"/Frames/hip2");
// // VisualizeFrame(shank1TF,mescat_ptr.get(),"/Frames/shank1");
// // VisualizeFrame(shank2TF,mescat_ptr.get(),"/Frames/shank2");
// VisualizeFrame(shank1_endTF,mescat_ptr.get(),"/Frames/shank1_endpoint");

#pragma endregion

// Add points
// drake_tfd  end_effector_transform( pos_con.DK({0,0,0}) ) ;


// drake_tfd  motor3_translation( pos_con.DK({-1,0.32,0.7}));
// drake_tfd  end_effector_transform = motor3TF*motor3_translation;
// drake_rotMat::MakeXRotation(-M_PI/2)*p_M3 + p_m3W; 

// AddPoint(end_effector_transform,mescat_ptr.get(),"estimated_end_effector"); 


// Simulate:
sim.set_publish_every_time_step(true); // publish simulation as it happens
sim.Initialize();
sim.set_target_realtime_rate(1);

double sim_time = 0; 
double sim_update_rate = 0.01; 
assert(sim_update_rate >= mb_time_step);
mescat_ptr->StartRecording();
while( sim_time < 5){
    //realtime vis
    sim_time +=sim_update_rate;
    sim.AdvanceTo(sim_time);    
    // (fl_leg).FixValue(&plant_context,qd); //for pid joints

}


//playback
mescat_ptr->PublishRecording();



return 0;
}







