// Copyright [2023] James Keane Quigley

#include "Body.h"
#include <iostream>


// ----------
// Body class
// ----------


Body::Body(const double &mass, const Eigen::Vector2d &position,
           const Eigen::Vector2d &velocity, const double &radius,
           const cv::Scalar &colour)
: mass(mass)
, position(position)
, velocity(velocity)
, accelerations({ Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(),
                  Eigen::Vector2d::Zero()})
, force(Eigen::Vector2d::Zero())
, radius(radius)
, colour(colour) {
  // empty constructor
}


/*
 Update the force on the body from an input.
 */
void Body::updateForce(const Eigen::Vector2d &new_force) {
  this->force = new_force;
}


/*
 Update the accelerations on the body.
 */
void Body::updateAcceleration() {
  // remove the oldest acceleration , add the latest one and shift the rest down
  this->accelerations[0] = this->accelerations[1];
  this->accelerations[1] = this->accelerations[2];
  this->accelerations[2] = (1 / this->mass) * this->force;
}


/*
 Update the position of the body using the Beeman algorithm.
 */
void Body::updatePosition(const float &timestep) {
  this->position = this->position + this->velocity * timestep + timestep
      * timestep * (4 * this->accelerations[1] - this->accelerations[0]) / 6;
}


/*
 Update the velocity of the body using the Beeman algorithm.
 */
void Body::updateVelocity(const float &timestep) {
  this->velocity = this->velocity + timestep * (2 * this->accelerations[2]
      + 5 * this->accelerations[1] - this->accelerations[0]) / 6;
}


/*
 Update the body's state.
 */
void Body::update(const float &timestep) {
  this->updateAcceleration();
  this->updatePosition(timestep);
  this->updateVelocity(timestep);
}


double Body::getMass() const {
  return this->mass;
}


Eigen::Vector2d Body::getPosition() const {
  return this->position;
}


/*
 Get the x component of the position.
 */
double Body::x() const {
  return this->position(0);
}


/*
 Get the y component of the position.
 */
double Body::y() const {
  return this->position(1);
}


/*
 Get the distance relative to the origin.
 */
double Body::getDistance() const {
  return this->position.norm();
}


/*
 Get the velocity.
 */
Eigen::Vector2d Body::getVelocity() const {
  return this->velocity;
}


/*
 Get the acceleration.
 */
std::vector<Eigen::Vector2d> Body::getAccelerations() const {
  return this->accelerations;
}


/*
 Get the net force acting on the body.
 */
Eigen::Vector2d Body::getForce() const {
  return this->force;
}


/*
 Get the body's radius.
 */
double Body::getRadius() const {
  return this->radius;
}


/*
 Get the body's colour.
 */
cv::Scalar Body::getColour() const {
  return this->colour;
}


/*
 Get the kinetic energy of the body.
 */
double Body::getKineticEnergy() const {
  return 0.5 * this->mass * this->velocity.dot(this->velocity);
}


/*
 Print all the body's information.
 */
void Body::printInfo() const {
  std::cout << "---------------" << "\n"
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
            << "    RGB        " << this->colour[2] << " " << this->colour[1]
            << " " << this->colour[0] << "\n"
            << "\n";
}


// ------------------
// Central Body Class
// ------------------


CentralBody::CentralBody(const double &mass, const double &radius,
                         const Eigen::Vector2d &position,
                         const cv::Scalar &colour)
: Body(mass, position, Eigen::Vector2d::Zero(), radius, colour) {
  // empty constructor
}


// ------------------
// Orbital Body Class
// ------------------


OrbitalBody::OrbitalBody(const double &mass, const double &central_mass,
                         const double &radius, const Eigen::Vector2d &position,
                         const cv::Scalar &colour)
: Body(mass, position,
       (std::sqrt(6.67408e-11 * central_mass / position.norm())) *
       Eigen::Vector2d(position.y(), - position.x()).normalized(),
       radius, colour)
, central_mass(central_mass) {
  this->updateOrbitalPeriod();
}


/*
 Update the body's state.
 */
void OrbitalBody::update(const float &timestep) {
  this->updateAcceleration();
  this->updatePosition(timestep);
  this->updateVelocity(timestep);
  this->updateOrbitalPeriod();
}


/*
 Update the orbital period of the body.
 */
void OrbitalBody::updateOrbitalPeriod() {
  this->orbital_period = 2 * M_PI * sqrt(pow(this->position.norm(), 3) /
      (6.67408e-11 * this->central_mass));
}


/*
 Get the central body's mass.
 */
double OrbitalBody::getCentralMass() const {
  return this->central_mass;
}


/*
 Get the orbital period of the body.
 */
float OrbitalBody::getOrbitalPeriod() const {
  return this->orbital_period;
}


/*
 Print all the body's information.
 */
void OrbitalBody::printInfo() const {
  std::cout << "----------------" << "\n"
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
            << "    RGB        " << this->colour[2] << " " << this->colour[1]
            << " " << this->colour[0] << "\n"
            << "\n";
}
