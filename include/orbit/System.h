// Copyright [2023] James Keane Quigley

#ifndef ORBIT_ORBIT_INCLUDE_SYSTEM_H_
#define ORBIT_ORBIT_INCLUDE_SYSTEM_H_


#include <eigen3/Eigen/Eigen>
#include <vector>
#include <fstream>
#include "Body.h"
#include "Params.h"
#include "indicators/indicators.h"


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

  indicators::ProgressBar bar {
    indicators::option::BarWidth{50},
    indicators::option::Start{"["},
    indicators::option::Fill{"="},
    indicators::option::Lead{">"},
    indicators::option::Remainder{" "},
    indicators::option::End{"]"},
    indicators::option::ShowPercentage{true},
    indicators::option::PostfixText{"Running Simulation"},
    indicators::option::ForegroundColor{indicators::Color::green},
    indicators::option::FontStyles{
      std::vector<indicators::FontStyle>{indicators::FontStyle::bold}}
  };
};


#endif  // ORBIT_ORBIT_INCLUDE_SYSTEM_H_
