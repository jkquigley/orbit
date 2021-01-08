#ifndef ORBITS_BODY_H
#define ORBITS_BODY_H


#include <vector>
#include <string>
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>


class Body {
public:
    Body(const std::string &name, const double &mass, const Eigen::Vector2d &position, const Eigen::Vector2d &velocity,
         const double &radius, const cv::Scalar &colour);

    void updateForce(const Eigen::Vector2d &new_force);
    virtual void update(float const &timestep);

    std::string getName() const;
    virtual double getMass() const;
    Eigen::Vector2d getPosition() const;
    double x() const;
    double y() const;
    double getDistance() const;
    Eigen::Vector2d getVelocity() const;
    std::vector<Eigen::Vector2d> getAccelerations() const;
    Eigen::Vector2d getForce() const;
    double getRadius() const;
    cv::Scalar getColour() const;
    double getKineticEnergy() const;
    virtual void printInfo() const;

protected:
    virtual void updateAcceleration();
    void updatePosition(const float &timestep);
    void updateVelocity(const float &timestep);

    const std::string name;
    const double mass;
    Eigen::Vector2d position;
    Eigen::Vector2d velocity;
    std::vector<Eigen::Vector2d> accelerations;
    Eigen::Vector2d force;
    const double radius;
    const cv::Scalar colour;
};


class CentralBody
: public Body
{
public:
    CentralBody(const std::string &name, const double &mass, const double &radius, const cv::Scalar &colour);
};


class OrbitalBody
: public Body
{
public:
    OrbitalBody(const std::string &name, const double &mass, const double &central_mass, const double &orbital_radius,
                const double &radius, const cv::Scalar &colour);

    double getCentralMass() const;
    float getOrbitalPeriod() const;

    void printInfo() const;

private:
    void updateOrbitalPeriod();
    void update(const float &timestep);

    double central_mass;
    float orbital_period;
};


class Rocket
: public Body
{
public:
    Rocket(const std::string &name, const double &mass, const double &fuel_mass, const double &burn_rate,
           const double &thrust_magnitude, const Body origin, const float &angular_launch_position, const double &radius,
           const cv::Scalar &colour);

    double getMass() const;
    double getBurnRate() const;
    Eigen::Vector2d getThrust() const;

    void printInfo() const;

private:
    void propel(const float &timestep);
    void updateAcceleration();
    void update(const float &timestep);

    cv::Scalar true_colour;
    double fuel_mass;
    double burn_rate;
    Eigen::Vector2d thrust;
};


#endif //ORBITS_BODY_H
