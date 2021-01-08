#include "Parser.h"
#include <yaml-cpp/yaml.h>
#include <fstream>


void operator >> (const YAML::Node &node, SimulationParams &params)
/*
 Convert a YAML::Node to SimulationParams.
 */
{
    params.timestep = node["timestep"].as<float>();
    params.nframes = node["nframes"].as<int>();
    params.steps_per_frame = node["steps_per_frame"].as<int>();
}


void operator >> (const YAML::Node &node, CentralBodyParams &params)
/*
 Convert a YAML::Node to CentralBodyParams.
 */
{
    params.name = node["name"].as<std::string>();
    params.mass = node["mass"].as<double>();
    params.radius = node["radius"].as<double>();
    float r = node["colour"][0].as<float>();
    float g = node["colour"][1].as<float>();
    float b = node["colour"][2].as<float>();
    params.colour = cv::Scalar(b, g, r);
}


void operator >> (const YAML::Node &node, RocketParams &params)
/*
 Convert a YAML::Node to RocketParams.
 */
{
    params.name = node["name"].as<std::string>();
    params.mass = node["mass"].as<double>();
    params.fuel_mass = node["fuel_mass"].as<double>();
    params.burn_rate = node["burn_rate"].as<double>();
    params.thrust_magnitude = node["thrust_magnitude"].as<double>();
    params.angular_launch_position = node["angular_launch_position"].as<float>();
    params.radius = node["radius"].as<double>();
    float r = node["colour"][0].as<float>();
    float g = node["colour"][1].as<float>();
    float b = node["colour"][2].as<float>();
    params.colour = cv::Scalar(b, g, r);
}


void operator >> (const YAML::Node &node, OrbitalBodyParams &params)
/*
 Convert a YAML::Node to OrbitalBodyParams.
 */
{
    params.name = node["name"].as<std::string>();
    params.mass = node["mass"].as<double>();
    params.radius = node["radius"].as<double>();
    params.orbital_radius = node["orbital_radius"].as<double>();
    float r = node["colour"][0].as<float>();
    float g = node["colour"][1].as<float>();
    float b = node["colour"][2].as<float>();
    params.colour = cv::Scalar(b, g, r);

    try
    {
        for (const auto &rocket_node: node["rockets"])
        {
            RocketParams rocket_params;
            rocket_node >> rocket_params;
            params.rockets.push_back(rocket_params);
        }
    }
    catch (YAML::KeyNotFound)
    { }
}





CentralBody Parser::createCentralBody(const CentralBodyParams &params)
/*
 Create a central body from a CentralBodyParams object.
 */
{
    return { params.name, params.mass, params.radius, params.colour };
}


OrbitalBody Parser::createOrbitalBody(const OrbitalBodyParams &params, const double &central_mass)
/*
 Create an orbital body from a OrbitalBodyParams object.
 */
{
    return { params.name, params.mass, central_mass, params.orbital_radius, params.radius, params.colour };
}


Rocket Parser::createRocket(const RocketParams &params, const Body &origin)
/*
 Create an orbital body from a OrbitalBodyParams object.
 */
{
    return { params.name, params.mass, params.fuel_mass, params.burn_rate, params.thrust_magnitude, origin, params.angular_launch_position, params.radius, params.colour };
}


System Parser::load(const char *filename)
/*
 Load a system from a given yaml file.
 */
{
    // get the main node of the yaml file
    auto head_node = YAML::LoadFile(filename);

    // get the simulation parameters
    SimulationParams sim_params;
    head_node["simulation"] >> sim_params;

    // get the bodies from the yaml file
    std::vector<Body> bodies;

    // first add the central body
    // the mass is needed to calculate the orbital bodies' velocity
    CentralBodyParams central_body_params;
    head_node["central_body"] >> central_body_params;
    bodies.push_back(this->createCentralBody(central_body_params));

    // get the central mass
    double central_mass = central_body_params.mass;

    // get the node for the orbital bodies
    auto orbital_bodies_node = head_node["orbital_bodies"];

    // for every orbital body create the body and add it to the bodies vector
    for (int i = 0; i < orbital_bodies_node.size(); i++)
    {
        OrbitalBodyParams orbital_body_params;
        orbital_bodies_node[i] >> orbital_body_params;
        auto body = this->createOrbitalBody(orbital_body_params, central_mass);
        bodies.push_back(body);

        // for every rocket from the body add the rocket to the bodies vector
        for (const auto &rocket_params: orbital_body_params.rockets)
        {
            bodies.push_back(this->createRocket(rocket_params, body));
        }
    }

    // return the system from the yaml file
    return { bodies, sim_params.timestep, sim_params.nframes, sim_params.steps_per_frame };
}
