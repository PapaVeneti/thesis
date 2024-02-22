#include <drake/math/rigid_transform.h> 
#include <drake/systems/framework/diagram.h>
// #include <drake/math/rotation_matrix.h> //to set up first frame

#include "drake/geometry/meshcat.h"

//graphviz
#include <iostream>
#include <fstream>
void get_system_graph(const drake::systems::Diagram<double> * diagram,const std::string & filePath = "graph.dot" );

/// This helper function visualizes a frame, that is passed as a `RigidTransformd`
/// @param[in] TF Transform defining the frame
/// @param[in] mescat_ptr raw pointer to the mescat instance.
/// @param[in] path_prefix Path prefix
/// Curently using setlinesegments (maybe `SetLine` is more optimal)
void VisualizeFrame(const drake::math::RigidTransformd &TF , drake::geometry::Meshcat* mescat_ptr, const std::string & path_prefix){
double scale_axis_param = 0.25;

mescat_ptr->SetLineSegments(
  path_prefix+"/x_axis",
  TF.translation(),
  TF.translation()+TF.rotation().col(0)*scale_axis_param,
  1,
  drake::geometry::Rgba(1,0,0,1)
);

mescat_ptr->SetLineSegments(
  path_prefix+"/y_axis",
  TF.translation(),
  TF.translation()+TF.rotation().col(1)*scale_axis_param,
  1,
  drake::geometry::Rgba(0,1,0,1)
);

mescat_ptr->SetLineSegments(
  path_prefix+"/z_axis",
  TF.translation(),
  TF.translation()+TF.rotation().col(2)*scale_axis_param,
  1,
  drake::geometry::Rgba(0,0,1,1)
);
// meshcat_shared_ptr mescat_ptr =  std::make_shared<drake::geometry::Meshcat> ();



}

std::string AddPoint(
  const drake::math::RigidTransformd &TF , 
  drake::geometry::Meshcat* mescat_ptr, 
  const std::string & point_name,
  const drake::geometry::Rgba &RGBA =  drake::geometry::Rgba(0,0,1,1),
  const double & radius = 0.025){
  // * Maybe return the path -> easier to delete afterwards
  std::string path_name = "/Helper symbols/Points/" + point_name;

  auto sphere = std::make_unique<drake::geometry::Sphere>(0.1);


  mescat_ptr->SetObject( path_name, drake::geometry::Sphere(radius),RGBA);
  mescat_ptr->SetTransform(path_name,TF);
  std::cout << "Hello1" << std::endl;

  return path_name;
}


void get_system_graph(const drake::systems::Diagram<double> * diagram,const std::string & filePath){
    std::string GraphString = diagram -> GetGraphvizString(1);
    // std::string filePath = "graph.dot";

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
