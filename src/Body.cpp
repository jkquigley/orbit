#include "Body.h"
#include <iostream>


// ----------
// Body class
// ----------


Body::Body(const std::string &name, const double &mass, const Eigen::Vector2d &position,
           const Eigen::Vector2d &velocity, const double &radius, const cv::Scalar &colour)
: name(name)
, mass(mass)
, position(position)
, velocity(velocity)
, accelerations({ Eigen::Vector2d(0, 0), Eigen::Vector2d(0, 0), Eigen::Vector2d(0, 0) })
, force(Eigen::Vector2d(0, 0))
, radius(radius)
, colour(colour)
{ }


void Body::updateForce(const Eigen::Vector2d &new_force)
/*
 Update the force on the body from an input.
 */
{
    this->force = new_force;
}


void Body::updateAcceleration()
/*
 Update the accelerations on the body.
 */
{
    // remove the oldest acceleration , add the latest one and shift the rest down
    this->accelerations[0] = this->accelerations[1];
    this->accelerations[1] = this->accelerations[2];
    this->accelerations[2] = (1 / this->mass) * this->force;
}


void Body::updatePosition(const float &timestep)
/*
 Update the position of the body using the Beeman algorithm.
 */
{
    this->position = this->position + this->velocity * timestep + timestep * timestep * (4 * this->accelerations[1] - this->accelerations[0]) / 6;
}

void Body::updateVelocity(const float &timestep)
/*
 Update the velocity of the body using the Beeman algorithm.
 */
{
    this->velocity = this->velocity + timestep * (2 * this->accelerations[2] + 5 * this->accelerations[1] - this->accelerations[0]) / 6;
}


void Body::update(const float &timestep)
/*
 Update the body's state.
 */
{
    this->updateAcceleration();
    this->updatePosition(timestep);
    this->updateVelocity(timestep);
}


std::string Body::getName() const
{
    return this->name;
}


double Body::getMass() const
{
    return this->mass;
}


Eigen::Vector2d Body::getPosition() const
{
    return this->position;
}


double Body::x() const
/*
 Get the x component of the position.
 */
{
    return this->position(0);
}


double Body::y() const
/*
 Get the y component of the position.
 */
{
    return this->position(1);
}


double Body::getDistance() const
/*
 Get the distance relative to the origin.
 */
{
    return this->position.norm();
}


Eigen::Vector2d Body::getVelocity() const
{
    return this->velocity;
}


std::vector<Eigen::Vector2d> Body::getAccelerations() const
{
    return this->accelerations;
}


Eigen::Vector2d Body::getForce() const
{
    return this->force;
}


double Body::getRadius() const
{
    return this->radius;
}


cv::Scalar Body::getColour() const
{
    return this->colour;
}


double Body::getKineticEnergy() const
/*
 Get the kinetic energy of the body.
 */
{
    double kinetic_energy = 0.5 * this->mass * this->velocity.dot(this->velocity);
    return kinetic_energy;
}


void Body::printInfo() const
/*
 Print all the body's information.
 */
{
    std::cout << this->name << "\n"
              << "    m        = " << this->mass << " kg\n"
              << "    p(t)     = " << this->position.x() << " m\n"
              << "               " << this->position.y() << " m\n"
              << "    v(t)     = " << this->velocity.x() << " m/s\n"
              << "               " << this->velocity.y() << " m/s\n"
              << "    a(t-2dt) = " << this->accelerations[0].x() << " m/s^2\n"
              << "               " << this->accelerations[0].y() << " m/s^2\n"
              << "    a(t-dt)  = " << this->accelerations[1].x() << " m/s^2\n"
              << "               " << this->accelerations[1].y() << " m/s^2\n"
              << "    a(t)     = " << this->accelerations[2].x() << " m/s^2\n"
              << "               " << this->accelerations[2].y() << " m/s^2\n"
              << "    F        = " << this->force.x() << " N\n"
              << "               " << this->force.y() << " N\n"
              << "    r        = " << this->radius << " m\n"
              << "    RGB        " << this->colour[2] << " " << this->colour[1] << " " << this->colour[0] << "\n"
              << "\n";
}


// ------------------
// Central Body Class
// ------------------


CentralBody::CentralBody(const std::string &name, const double &mass, const double &radius, const cv::Scalar &colour)
: Body(name, mass, {0, 0}, {0, 0}, radius, colour)
{ }


// ------------------
// Orbital Body Class
// ------------------


OrbitalBody::OrbitalBody(const std::string &name, const double &mass, const double &central_mass,
                         const double &orbital_radius, const double &radius, const cv::Scalar &colour)
: Body(name, mass, {orbital_radius, 0}, {0, std::sqrt(6.67408e-11 * central_mass / orbital_radius)}, radius, colour)
, central_mass(central_mass)
, orbital_period(2 * M_PI * sqrt(pow(orbital_radius, 3) / (6.67408e-11 * central_mass)))
{ }


void OrbitalBody::update(const float &timestep)
/*
 Update the body's state.
 */
{
        this->updateAcceleration();
        this->updatePosition(timestep);
        this->updateVelocity(timestep);
        this->updateOrbitalPeriod();
}


void OrbitalBody::updateOrbitalPeriod()
/*
 Update the orbital period of the body.
 */
{
    this->orbital_period = 2 * M_PI * sqrt(pow(this->position.norm(), 3) / (6.67408e-11 * this->central_mass));
}


double OrbitalBody::getCentralMass() const
/*
 Get the central body's mass.
 */
{
    return this->central_mass;
}


float OrbitalBody::getOrbitalPeriod() const
/*
 Get the orbital period of the body.
 */
{
    return this->orbital_period;
}


void OrbitalBody::printInfo() const
/*
 Print all the body's information.
 */
{
    std::cout << this->name << "\n"
              << "    m        = " << this->mass << " kg\n"
              << "    p(t)     = " << this->position.x() << " m\n"
              << "               " << this->position.y() << " m\n"
              << "    v(t)     = " << this->velocity.x() << " m/s\n"
              << "               " << this->velocity.y() << " m/s\n"
              << "    a(t-2dt) = " << this->accelerations[0].x() << " m/s^2\n"
              << "               " << this->accelerations[0].y() << " m/s^2\n"
              << "    a(t-dt)  = " << this->accelerations[1].x() << " m/s^2\n"
              << "               " << this->accelerations[1].y() << " m/s^2\n"
              << "    a(t)     = " << this->accelerations[2].x() << " m/s^2\n"
              << "               " << this->accelerations[2].y() << " m/s^2\n"
              << "    F        = " << this->force.x() << " N\n"
              << "               " << this->force.y() << " N\n"
              << "    r        = " << this->radius << " m\n"
              << "    T        = " << this->orbital_period << " s\n"
              << "    RGB        " << this->colour[2] << " " << this->colour[1] << " " << this->colour[0] << "\n"
              << "\n";
}


// ------------
// Rocket Class
// ------------


Rocket::Rocket(const std::string &name, const double &mass, const double &fuel_mass, const double &burn_rate,
               const double &thrust_magnitude, const Body origin, const float &angular_launch_position,
               const double &radius, const cv::Scalar &colour)
: Body(name, mass,{cos(angular_launch_position) * origin.getRadius() + origin.getPosition().x(),sin(angular_launch_position) * origin.getRadius() + origin.getPosition().y()},origin.getVelocity(), radius, colour)
, true_colour(colour)
, fuel_mass(fuel_mass)
, burn_rate(burn_rate)
, thrust(thrust_magnitude * (this->position - origin.getPosition()))
{ }


double Rocket::getMass() const
{
    // include the fuel mass
    return this->mass + this->fuel_mass;
}


double Rocket::getBurnRate() const
{
    return this->burn_rate;
}


Eigen::Vector2d Rocket::getThrust() const
{
    return this->thrust;
}


void Rocket::propel(const float &timestep)
/*
 Accelerate the rocket using the rocket fuel.
 */
{
    // if there's no fuel do nothing
    if (this->fuel_mass <= 0) return;

    // remove the fuel used
    this->fuel_mass = this->burn_rate * timestep;

    // add the thrust to the acceleration
    this->accelerations[2] += this->thrust;
}


void Rocket::updateAcceleration()
/*
 Update the accelerations on the body.
 */
{
    // remove the oldest acceleration , add the latest one and shift the rest down
    this->accelerations[0] = this->accelerations[1];
    this->accelerations[1] = this->accelerations[2];
    this->accelerations[2] = (1 / (this->mass + this->fuel_mass)) * this->force;
}


void Rocket::update(const float &timestep)
/*
 Update the body's state.
 */
{
    this->updateAcceleration();
    this->propel(timestep);
    this->updatePosition(timestep);
    this->updateVelocity(timestep);
}


void Rocket::printInfo() const
/*
 Print all the body's information.
 */
{
    std::cout << this->name << "\n"
              << "    m        = " << this->mass << " kg\n"
              << "    m_fuel   = " << this->fuel_mass << " kg\n"
              << "    p(t)     = " << this->position.x() << " m\n"
              << "               " << this->position.y() << " m\n"
              << "    v(t)     = " << this->velocity.x() << " m/s\n"
              << "               " << this->velocity.y() << " m/s\n"
              << "    a(t-2dt) = " << this->accelerations[0].x() << " m/s^2\n"
              << "               " << this->accelerations[0].y() << " m/s^2\n"
              << "    a(t-dt)  = " << this->accelerations[1].x() << " m/s^2\n"
              << "               " << this->accelerations[1].y() << " m/s^2\n"
              << "    a(t)     = " << this->accelerations[2].x() << " m/s^2\n"
              << "               " << this->accelerations[2].y() << " m/s^2\n"
              << "    F_g      = " << this->force.x() << " N\n"
              << "               " << this->force.y() << " N\n"
              << "    r        = " << this->radius << " m\n"
              << "    F_A      = " << this->thrust[0] << " N\n"
              << "               " << this->thrust[1] << "\n"
              << "    RGB        " << this->colour[2] << " " << this->colour[1] << " " << this->colour[0] << "\n"
              << "\n";
}
