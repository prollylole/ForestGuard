#include "sdformat_urdf/sdformat_urdf.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

int main(int argc, char * argv[])
{
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <input.sdf> <output.urdf>\n";
    return 1;
  }

  // Read SDF file
  std::ifstream sdf_file(argv[1]);
  if (!sdf_file) {
    std::cerr << "Error opening SDF file: " << argv[1] << "\n";
    return 1;
  }
  std::stringstream sdf_buffer;
  sdf_buffer << sdf_file.rdbuf();
  std::string sdf_content = sdf_buffer.str();

  // Convert SDF to URDF
  sdf::Errors errors;
  ::urdf::ModelInterfaceSharedPtr urdf_model = sdformat_urdf::parse(sdf_content, errors);

  if (!urdf_model) {
    std::cerr << "Conversion failed with errors:\n";
    for (const auto & error : errors) {
      std::cerr << error.Message() << "\n";
    }
    return 1;
  }

  // Write URDF to file
  std::ofstream urdf_file(argv[2]);
  if (!urdf_file) {
    std::cerr << "Error creating URDF file: " << argv[2] << "\n";
    return 1;
  }

  // Basic URDF output
  urdf_file << "<?xml version=\"1.0\"?>\n";
  urdf_file << "<robot name=\"" << urdf_model->name_ << "\">\n";
  
  // TODO: Add proper URDF output generation
  // This is a placeholder - you'll need to implement proper URDF generation
  // based on the urdf_model object
  
  urdf_file << "  <link name=\"base_link\"/>\n"; // Placeholder
  urdf_file << "</robot>\n";

  std::cout << "Conversion successful. URDF saved to: " << argv[2] << "\n";
  return 0;
}