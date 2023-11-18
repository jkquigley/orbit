// Copyright [2023] James Keane Quigley

#ifndef ORBIT_INCLUDE_SYSTEM_H_
#define ORBIT_INCLUDE_SYSTEM_H_


#include <eigen3/Eigen/Eigen>
#include <vector>
#include <fstream>
#include "Body.h"
#include "Params.h"


class System {
 public:
  System(const OutputParams &output_params, const SimulationParams &sim_params,
         const std::vector<BodyParams> &bodies_params);

  void run();

  double getTotalKineticEnergy();
  double getTotalPotentialEnergy();
  double getTotalEnergy();

 private:
  void updateForces();
  void update();

  double G = 6.67408e-11;
  std::vector<Body> bodies;
  const OutputParams output_params;
  const SimulationParams sim_params;
};


#endif  // ORBIT_INCLUDE_SYSTEM_H_
