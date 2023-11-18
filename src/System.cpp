// Copyright [2023] James Keane Quigley

#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "System.h"


/*
Convert a number to scientific notation.
*/
inline std::string scientificNotation(const double &number) {
  std::stringstream stream;
  stream.precision(5);
  stream << std::scientific << number;
  return stream.str();
}


System::System(const OutputParams &output_params,
               const SimulationParams &sim_params,
               const std::vector<BodyParams> &bodies_params)
: output_params(output_params)
, sim_params(sim_params) {
  // find the largest body to be the central body
  auto compare_mass = [](const BodyParams &a, const BodyParams &b)
      {return a.mass < b.mass;};
  auto central_body_it = std::max_element(bodies_params.begin(),
                                          bodies_params.end(),
                                          compare_mass);

  // add the central body to the list of bodies
  this->bodies.push_back(CentralBody(central_body_it->mass,
                                     central_body_it->radius,
                                     Eigen::Vector2d::Zero() ,
                                     central_body_it->colour));

  // add the other bodies to the list with the central body as the origin
  for (auto it = bodies_params.begin(); it != bodies_params.end(); it++) {
    if (it != central_body_it) {
      this->bodies.push_back(
          OrbitalBody(it->mass, central_body_it->mass, it->radius,
                      it->position - central_body_it->position, it->colour));
    }
  }
}


/*
 Update the forces for every body.
 */
void System::updateForces() {
  for (int i = 0; i < this->bodies.size(); i++) {
    auto net_force = Eigen::Vector2d(0, 0);
    auto m1 = this->bodies[i].getMass();
    auto r1 = this->bodies[i].getPosition();

    // calculate and add the forces from all other bodies.
    for (int j = 0; j < this->bodies.size(); j++) {
      if (i != j) {
        auto m2 = this->bodies[j].getMass();
        auto r2 = this->bodies[j].getPosition();

        // get the relative position and direction between the two bodies
        auto relative_position = r2 - r1;
        auto direction = (1 / relative_position.norm()) * relative_position;

        // add the force to the net force
        net_force += (this->G * m1 * m2 /
            relative_position.dot(relative_position)) * direction;
      }
    }

    // update the force on the body
    this->bodies[i].updateForce(net_force);
  }
}


/*
 Update the system for one frame.
 */
void System::update() {
  for (int i = 0; i < this->sim_params.steps_per_frame; i++) {
    this->updateForces();

    for (auto &body : this->bodies) {
      body.update(this->sim_params.timestep);
    }
  }
}


/*
 Run the simulation and animate it.
 */
void System::run() {
  // set up the image conditions
  auto image_size = cv::Size(this->output_params.width,
                             this->output_params.height);
  const int half_image_width = image_size.width / 2;
  const int half_image_length = image_size.height / 2;
  cv::Mat background(image_size.width , this->output_params.height,
                     CV_8UC3, cv::Scalar(0, 0, 0));

  // set up a video writer
  cv::VideoWriter video(this->output_params.filename,
                        cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                        this->output_params.fps, image_size);

  // find the initial max distance to scale te animation
  double max_dist = 0;
  for (const auto &body : this->bodies) {
    auto distance = body.getDistance();
    if (distance > max_dist) max_dist = distance;
  }
  // add a buffer to the max distance
  max_dist *= 1.2;

  // update the system for each frame and display it
  // write the energy data to file
  for (int i = 0; i < this->sim_params.nframes; i++) {
    this->update();

    // copy the background image
    cv::Mat frame;
    background.copyTo(frame);

    for (const auto &body : this->bodies) {
      // calculate the pixel positions of the center of the circle / body
      int x_pixel = body.x() / max_dist * half_image_width + half_image_width;
      int y_pixel = - body.y() / max_dist * half_image_length +
          half_image_length;

      cv::Point circle_center(x_pixel, y_pixel);
      // scale the radius
      int circle_radius = body.getRadius() / max_dist * half_image_width;
      // add the circle to the background image
      cv::circle(frame, circle_center, circle_radius, body.getColour(),
                 cv::FILLED);
    }

    // add the frame count and energy values to the image
    std::string frame_text = "Frame: " + std::to_string(i) + "/" +
        std::to_string(this->sim_params.nframes);
    cv::putText(frame, frame_text, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX,
                0.35, cv::Scalar(255, 255, 255), 1);

    std::string energy_text = "Total Energy: " +
        scientificNotation(this->getTotalEnergy()) + " J";
    cv::putText(frame, energy_text, cv::Point(10, image_size.width - 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 255, 255), 1);

    // display the image
    if (this->output_params.save) video.write(frame);
    cv::imshow("Simulation", frame);
    if (cv::waitKey(1) == 27) return;
  }
}


/*
 Get the total kinetic energy in the system.
 */
double System::getTotalKineticEnergy() {
  double kinetic_energy = 0;

  // calculate and add the kinetic energy from all bodies.
  for (const auto &body : this->bodies) {
    kinetic_energy += body.getKineticEnergy();
  }

  return kinetic_energy;
}


/*
 Get the total potential energy in the system.
 */
double System::getTotalPotentialEnergy() {
  double potential_energy = 0;

  // for every pair of bodies find the potential between them
  // this is similar to calculating the forces between all the bodies
  for (int i = 0; i < this->bodies.size(); i++) {
    auto m1 = this->bodies[i].getMass();
    auto r1 = this->bodies[i].getPosition();

    for (int j = 0; j < this->bodies.size(); j++) {
      if (i != j) {
        auto m2 = this->bodies[j].getMass();
        auto r2 = this->bodies[j].getPosition();

        auto relative_posiion = r2 - r1;
        auto distance = relative_posiion.norm();

        potential_energy += - this->G * m1 * m2 / distance;
      }
    }
  }

  return potential_energy / 2;
}


/*
 Add the kinetic and potential energy.
 */
double System::getTotalEnergy() {
  return this->getTotalKineticEnergy() + this->getTotalPotentialEnergy();
}
