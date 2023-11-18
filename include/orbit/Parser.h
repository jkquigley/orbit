// Copyright [2023] James Keane Quigley

#ifndef ORBIT_ORBIT_INCLUDE_PARSER_H_
#define ORBIT_ORBIT_INCLUDE_PARSER_H_


#include <vector>
#include "Params.h"


class Parser {
 public:
    Parser() = default;

    static bool load(const char *filename, OutputParams *output_params,
                     SimulationParams *sim_params,
                     std::vector<BodyParams> *bodies_params);
};


#endif  // ORBIT_ORBIT_INCLUDE_PARSER_H_
