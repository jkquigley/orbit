// Copyright [2023] James Keane Quigley

#ifndef ORBIT_ORBIT_INCLUDE_BODY_H_
#define ORBIT_ORBIT_INCLUDE_BODY_H_


#include <eigen3/Eigen/Eigen>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>


class Body {
 public:
  Body(const double &mass, const Eigen::Vector2d &position,
       const Eigen::Vector2d &velocity, const double &radius,
       const cv::Scalar &colour);

  void updateForce(const Eigen::Vector2d &new_force);
  virtual void update(float const &timestep);

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

  const double mass;
  Eigen::Vector2d position;
  Eigen::Vector2d velocity;
  std::vector<Eigen::Vector2d> accelerations;
  Eigen::Vector2d force;
  const double radius;
  const cv::Scalar colour;
};


class CentralBody
: public Body {
 public:
  CentralBody(const double &mass, const double &radius,
              const Eigen::Vector2d &position, const cv::Scalar &colour);
};


class OrbitalBody
: public Body {
 public:
  OrbitalBody(const double &mass, const double &central_mass,
              const double &radius, const Eigen::Vector2d &position,
              const cv::Scalar &colour);

  double getCentralMass() const;
  float getOrbitalPeriod() const;

  void printInfo() const;

 private:
  void updateOrbitalPeriod();
  void update(const float &timestep);

  double central_mass;
  float orbital_period;
};


#endif  // ORBIT_ORBIT_INCLUDE_BODY_H_
