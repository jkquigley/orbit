#ifndef ORBIT_SYSTEM_H
#define ORBIT_SYSTEM_H


#include <vector>
#include <fstream>
#include <eigen3/Eigen/Eigen>
#include "Body.h"


class System {
public:
    System(const std::vector<Body> &bodies, const float &timestep, const int &nframes, const int &steps_per_frame=1);

    void run();
    void run(const char* energy_filename, const bool &animate=false);
    void run(const char* energy_filename, const char* pos_filename, const bool &animate=false);

    double getTotalKineticEnergy();
    double getTotalPotentialEnergy();
    double getTotalEnergy();

private:
    void updateForces();
    void update();

    void writeMiscData(std::ofstream &file);
    void writePositionData(std::ofstream &file);
    void writeEnergyData(std::ofstream &file);

    double G = 6.67408e-11;
    std::vector<Body> bodies;
    const float timestep;
    const int nframes;
    const int steps_per_frame;
};


#endif //ORBIT_SYSTEM_H
