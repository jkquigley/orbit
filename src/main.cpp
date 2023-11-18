// Copyright [2023] James Keane Quigley

#include <iostream>
#include "orbit/Parser.h"
#include "orbit/System.h"


void help() {
  std::cout << "[Usage]\n"
            << "./orbit <cfg file>\n";
}


int main(int argc, char** argv) {
  if (argc == 2) {
    OutputParams output_params;
    SimulationParams sim_params;
    std::vector<BodyParams> bodies_params;
    if (!Parser::load(argv[1], &output_params, &sim_params, &bodies_params)) {
      std::cout << "Error loading file: " << argv[1] << "\n";
      return -1;
    }

    System system(output_params, sim_params, bodies_params);
    system.run();

    return 0;
  } else {
    help();
    return -1;
  }
}
