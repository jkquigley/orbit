// Copyright [2023] James Keane Quigley

#include "Parser.h"
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Eigen>


/*
 Convert a YAML::Node to OutputParams.
 */
void operator >> (const YAML::Node &node, OutputParams &params) {
  try {
    params.filename = node["filename"].as<std::string>();
    params.save = true;
  }
  catch (YAML::Exception &e) {
    params.save = false;
    params.filename = "";
  }

  params.height = node["height"].as<int>();
  params.width = node["width"].as<int>();
  params.fps = node["fps"].as<int>();
}


/*
 Convert a YAML::Node to SimulationParams.
 */
void operator >> (const YAML::Node &node, SimulationParams &params) {
  params.timestep = node["timestep"].as<float>();
  params.nframes = node["nframes"].as<int>();
  params.steps_per_frame = node["steps_per_frame"].as<int>();
}


/*
 Convert a YAML::Node to OrbitalBodyParams.
 */
void operator >> (const YAML::Node &node, BodyParams &params) {
  params.mass = node["mass"].as<double>();
  params.radius = node["radius"].as<double>();
  auto x = node["position"][0].as<double>();
  auto y = node["position"][1].as<double>();
  params.position = Eigen::Vector2d(x, y);
  auto r = node["colour"][0].as<float>();
  auto g = node["colour"][1].as<float>();
  auto b = node["colour"][2].as<float>();
  params.colour = cv::Scalar(b, g, r);
}


/*
 Load a system from a given yaml file.
 */
bool Parser::load(const char* filename, OutputParams* output_params,
                  SimulationParams* sim_params,
                  std::vector<BodyParams>* bodies_params) {
  // get the main node of the yaml file
  YAML::Node head_node;
  try {
    head_node = YAML::LoadFile(filename);
  }
  catch (YAML::BadFile &) {
    return false;
  }

  // get the output parameters
  head_node["output"] >> *output_params;

  // get the simulation parameters
  head_node["simulation"] >> *sim_params;

  // get the body parameters from the yaml file
  auto bodies_node = head_node["bodies"];

  // for every body add the parameters to the vector
  for (auto && i : bodies_node) {
    BodyParams body_params;
    i >> body_params;
    bodies_params->push_back(body_params);
  }

  return true;
}
