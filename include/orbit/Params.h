// Copyright [2023] James Keane Quigley

#ifndef ORBIT_ORBIT_INCLUDE_PARAMS_H_
#define ORBIT_ORBIT_INCLUDE_PARAMS_H_

#include <eigen3/Eigen/Eigen>
#include <string>
#include <opencv2/opencv.hpp>


/*
 * Data structure to store the output parameters.
 */
struct OutputParams {
  bool display;
  bool save;
  std::string filename;
  int width;
  int height;
  int fps;
};


/*
 Data structure to store the simulation parameters.
 */
struct SimulationParams {
  float timestep;
  int nframes;
  int steps_per_frame;
};


/*
 Data structure to store a body's parameters.
 */
struct BodyParams {
  double mass;
  double radius;
  Eigen::Vector2d position;
  cv::Scalar colour;
};

#endif  // ORBIT_ORBIT_INCLUDE_PARAMS_H_
