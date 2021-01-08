#ifndef ORBIT_PARSER_H
#define ORBIT_PARSER_H


#include <fstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "Body.h"
#include "System.h"


struct SimulationParams
/*
 Data structure to store the simulation parameters.
 */
{
    float timestep;
    int nframes;
    int steps_per_frame;
    char* outfile;
};


struct CentralBodyParams
/*
 Data structure to store the central body's parameters.
 */
{
    std::string name;
    double mass;
    double radius;
    cv::Scalar colour;
};


struct RocketParams
/*
 Data structure to store a rocket's parameters.
 */
{
    std::string name;
    double mass;
    double fuel_mass;
    double burn_rate;
    double thrust_magnitude;
    float angular_launch_position;
    double radius;
    cv::Scalar colour;
};


struct OrbitalBodyParams
/*
 Data struture to store an orbital body's parameters.
 */
{
    std::string name;
    double mass;
    double radius;
    double orbital_radius;
    cv::Scalar colour;
    std::vector<RocketParams> rockets;
};


class Parser {
public:
    Parser() = default;

    System load(const char *filename);

private:
    static CentralBody createCentralBody(const CentralBodyParams &params);
    static OrbitalBody createOrbitalBody(const OrbitalBodyParams &params, const double &central_mass);
    static Rocket createRocket(const RocketParams &params, const Body &origin);
};


#endif //ORBIT_PARSER_H
