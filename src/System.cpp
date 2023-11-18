#include <iostream>
#include <opencv2/opencv.hpp>
#include "System.h"


System::System(const std::vector<Body> &bodies, const float &timestep, const int &nframes, const int &steps_per_frame)
: bodies(bodies)
, timestep(timestep)
, nframes(nframes)
, steps_per_frame(steps_per_frame)
{ }


void System::updateForces()
/*
 Update the forces for every body.
 */
{
    for (int i = 0; i < this->bodies.size(); i++)
    {
        auto net_force = Eigen::Vector2d(0, 0);
        auto m1 = this->bodies[i].getMass();
        auto r1 = this->bodies[i].getPosition();

        // for every other body in the system calculate the force and add it to the net force.
        for (int j = 0; j < this->bodies.size(); j++)
        {
            if (i != j)
            {
                auto m2 = this->bodies[j].getMass();
                auto r2 = this->bodies[j].getPosition();

                // get the relative position and direction vector between the two bodies
                auto relative_position = r2 - r1;
                auto direction = (1 / relative_position.norm()) * relative_position;

                // add the force to the net force
                net_force += (this->G * m1 * m2 / relative_position.dot(relative_position)) * direction;
            }
        }

        // update the force on the body
        this->bodies[i].updateForce(net_force);
    }
}


void System::update()
/*
 Update the system for one frame.
 */
{
    for (int i = 0; i < this->steps_per_frame; i++)
    {
        this->updateForces();

        for (auto &body: this->bodies)
        {
            body.update(this->timestep);
        }
    }
}


void System::run()
/*
 Run the simulation and animate it.
 */
{
    // get the initial image conditions
    const int image_width = 500;
    const int image_length = 500;
    const int half_image_width = image_width / 2;
    const int half_image_length = image_length / 2;
    cv::Mat background(image_width, image_length, CV_8UC3, cv::Scalar(0, 0, 0));

    // find the initial max distance to scale te animation
    // the max distance needs to be a little bigger than calculated to contain all bodies
    double max_dist = -1;
    for (const auto &body: this->bodies) {
        auto distance = body.getDistance();
        if (distance > max_dist) max_dist = distance * 1.2;
    }

    // update the system for each frame and display it
    // write the energy data to file
    for (int i = 0; i < this->nframes; i++) {
        this->update();

        // copy the background image
        cv::Mat frame;
        background.copyTo(frame);

        for (const auto &body: this->bodies) {
            // calculate the pixel positions of the center of the circle / body
            int x_pixel = (body.x() / max_dist) * half_image_width + half_image_width;
            int y_pixel = - (body.y() / max_dist) * half_image_length + half_image_length;
            cv::Point circle_center(x_pixel, y_pixel);
            // scale the radius
            int circle_radius = body.getRadius() / max_dist * half_image_width;
            // add the circle to the background image
            cv::circle(frame, circle_center, circle_radius, body.getColour(), cv::FILLED);
        }

        // display the image
        cv::imshow("Simulation", frame);
        if (cv::waitKey(1) == 27) return;
    }
}


void System::run(const char* filename, const bool &animate)
/*
 Run the simulation
 Animate if set.
 Write energy data to file.
 */
{
    // open the file
    std::ofstream file(filename);

    if (animate)
    {
        // get the initial image conditions
        const int image_width = 500;
        const int image_length = 500;
        const int half_image_width = image_width / 2;
        const int half_image_length = image_length / 2;
        cv::Mat background(image_width, image_length, CV_8UC3, cv::Scalar(0, 0, 0));

        // find the initial max distance to scale te animation
        // the max distance needs to be a little bigger than calculated to contain all bodies
        double max_dist = 0;
        for (const auto &body: this->bodies) {
            auto distance = body.getDistance();
            if (distance > max_dist) max_dist = distance;
        }

        // Add padding for the image
        max_dist *= 1.2;

        // update the system for each frame and display it
        // write the energy data to file
        for (int i = 0; i < this->nframes; i++) {
            this->update();

            // copy the background image
            cv::Mat frame;
            background.copyTo(frame);

            for (const auto &body: this->bodies) {
                // calculate the pixel positions of the center of the circle / body
                int x_pixel = (body.x() / max_dist) * half_image_width + half_image_width;
                int y_pixel = - (body.y() / max_dist) * half_image_length + half_image_length;
                cv::Point circle_center(x_pixel, y_pixel);
                // scale the radius
                int circle_radius = body.getRadius() / max_dist * half_image_width;
                // add the circle to the background image
                cv::circle(frame, circle_center, circle_radius, body.getColour(), cv::FILLED);
            }

            this->writeEnergyData(file);

            // display the image
            cv::imshow("Simulation", frame);
            if (cv::waitKey(1) == 27) return;
        }
    }

    else
    {
        for (int i = 0; i < this->nframes; i++) {
            this->update();
            this->writeEnergyData(file);
        }
    }

    file.close();
}


void System::run(const char *energy_filename, const char *position_filename, const bool &animate)
/*
 Run the simulation.
 Animate if set.
 Write energy and position data to file.
 */
{
    // open the files
    std::ofstream energy_file(energy_filename);
    std::ofstream position_file(position_filename);

    // add the miscellaneous data for every body to the position data file.
    this->writeMiscData(position_file);

    if (animate)
    {// get the initial image conditions
        const int image_width = 500;
        const int image_length = 500;
        const int half_image_width = image_width / 2;
        const int half_image_length = image_length / 2;
        cv::Mat background(image_width, image_length, CV_8UC3, cv::Scalar(0, 0, 0));

        // find the initial max distance to scale te animation
        // the max distance needs to be a little bigger than calculated to contain all bodies
        double max_dist = -1;
        for (const auto &body: this->bodies) {
            auto distance = body.getDistance();
            if (distance > max_dist) max_dist = distance * 1.2;
        }

        // update the system for each frame and display it
        // write the energy data to file
        for (int i = 0; i < this->nframes; i++) {
            this->update();

            // copy the background image
            cv::Mat frame;
            background.copyTo(frame);

            for (const auto &body: this->bodies) {
                // calculate the pixel positions of the center of the circle / body
                int x_pixel = (body.x() / max_dist) * half_image_width + half_image_width;
                int y_pixel = - (body.y() / max_dist) * half_image_length + half_image_length;
                cv::Point circle_center(x_pixel, y_pixel);
                // scale the radius
                int circle_radius = body.getRadius() / max_dist * half_image_width;
                // add the circle to the background image
                cv::circle(frame, circle_center, circle_radius, body.getColour(), cv::FILLED);
            }

            // record the energy and position data
            this->writeEnergyData(energy_file);
            this->writePositionData(position_file);

            // display the image
            cv::imshow("Simulation", frame);
            if (cv::waitKey(1) == 27) return;
        }
    }

    else
    {
        for (int i = 0; i < this->nframes; i++)
        {
            this->update();
            this->writeEnergyData(energy_file);
            this->writePositionData(position_file);
        }
    }

    energy_file.close();
    position_file.close();
}


double System::getTotalKineticEnergy()
/*
 Get the total kinetic energy in the system.
 */
{
    double kinetic_energy = 0;

    // for every body get the kinetic energy and add it to the system kinetic energy.
    for (const auto &body: this->bodies)
    {
        kinetic_energy += body.getKineticEnergy();
    }

    return kinetic_energy;
}


double System::getTotalPotentialEnergy()
/*
 Get the total potential energy in the system.
 */
{
    double potential_energy = 0;

    // for every pair of bodies find the potential between them
    // this is similar to calculating the forces between all the bodies
    for (int i = 0; i < this->bodies.size(); i++)
    {
        auto m1 = this->bodies[i].getMass();
        auto r1 = this->bodies[i].getPosition();

        for (int j = 0; j < this->bodies.size(); j++)
        {
            if (i != j)
            {
                auto m2 = this->bodies[j].getMass();
                auto r2 = this->bodies[j].getPosition();

                auto relative_posiion = r2 - r1;
                auto distance = relative_posiion.norm();

                potential_energy += -0.5 * this->G * m1 * m2 / distance;
            }
        }
    }

    return potential_energy;
}


double System::getTotalEnergy()
/*
 Add the kinetic and potential energy.
 */
{
    return this->getTotalKineticEnergy() + this->getTotalPotentialEnergy();
}


void System::writeMiscData(std::ofstream &file)
/*
 Write the miscellaneous data to the data file.
 */
{
    for (const auto &body: this->bodies)
    {
        auto body_colour = body.getColour();
        file << "~" << " " << body.getName() << " "
             << std::to_string(body.getRadius()) << " "
             << body_colour[2] << " " << body_colour[1] << " " << body_colour[0] << "\n";
    }

    file << "\n";
}


void System::writeEnergyData(std::ofstream &file)
/*
 Write a line of energy data to the data file.
 */
{
    file << this->getTotalKineticEnergy() << " " << this->getTotalPotentialEnergy() <<  " " << this->getTotalEnergy() << "\n";
}


void System::writePositionData(std::ofstream &file)
/*
 Write the data for the step to the data file.
 */
{
    for (const auto &body: this->bodies)
    {
        file << body.x() << " " << body.y() << "\n";
    }

    file << "\n";
}
