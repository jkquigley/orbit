from sys import argv
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np


class Grapher:
    def __init__(self, filename):
        self.load(filename)

    def load(self, filename):
        all_kinetic = []
        all_potential = []
        all_total = []

        with open(filename, 'r') as file:
            for line in file:
                first_char = line[0]

                if first_char == '#':
                    continue

                elif first_char == '\n':
                    continue

                else:
                    line_data = line.split(' ')

                    kinetic = float(line_data[0])
                    all_kinetic.append(kinetic)

                    potential = float(line_data[1])
                    all_potential.append(potential)

                    total = float(line_data[2])
                    all_total.append(total)

        self.data = (all_kinetic, all_potential, all_total)

    def plot(self):
        # extract the kinetic, potential and total energy data to plot
        kinetic = self.data[0]
        potential = self.data[1]
        total = self.data[2]

        # plot the energy data on three seperate plots
        # plt.subplot(1, 3, 1)
        # plt.plot(kinetic)
        # plt.title("Kinetic Energy (J) of the System")
        # plt.xlabel("Frame number")
        # plt.ylabel("Energy (J)")
        #
        # plt.subplot(1, 3, 2)
        # plt.plot(potential)
        # plt.title("Potential Energy (J) of the System")
        # plt.xlabel("Frame number")
        # plt.ylabel("Energy (J)")
        #
        # plt.subplot(1, 3, 3)
        # plt.plot(total)
        # plt.title("Total Energy (J) of the System")
        # plt.xlabel("Frame number")
        # plt.ylabel("Energy (J)")

        # plot the data
        energy_ax = plt.axes()
        energy_ax.set_title("Total Energy of the System")
        energy_ax.set_xlabel("Frame number")
        energy_ax.set_ylabel("Energy (J)")

        energy_ax.plot(kinetic, label='Kinetic Energy (J)')
        energy_ax.plot(potential, label='Potentail Energy (J)')
        energy_ax.plot(total, label='Total Energy (J)')
        energy_ax.legend()

        # plt.show()


class Animator:
    def __init__(self, filename):
        self.count = 0
        self.load(filename)

    def load(self, filename):
        self.data = []
        self.misc = []

        # load the data from file
        with open(filename, 'r') as file:
            # for every step add the position data to this list to be added to self.position_data
            step_position_data = []
            for line in file:
                first_char = line[0]

                # treat lines beginning with '#' as comments
                if first_char == '#':
                    continue

                # skip empty lines
                elif first_char == '\n' and len(step_position_data) == 0:
                    continue

                # if an empty line is reached and step_position_data contains data
                # add it to self.position_data and clear it
                elif first_char == '\n' and len(step_position_data) != 0:
                    self.data.append(step_position_data)
                    step_position_data = []

                # lines beginning with '~' are miscellaneous data
                elif first_char == '~':
                    # remove the new line to avoid errors
                    if line[-1] == '\n':
                        line = line[:-1]

                    body_data = line.split(' ')

                    # get the RGB colour value
                    body_colour_r = float(body_data[3]) / 255
                    body_colour_g = float(body_data[4]) / 255
                    body_colour_b = float(body_data[5]) / 255
                    body_colour = (body_colour_r, body_colour_g, body_colour_b)

                    self.misc.append({'name': body_data[1], 'radius': float(body_data[2]), 'colour': body_colour})

                # add the position data to step_position_data
                else:
                    body_position_data = line.split(' ')
                    x = float(body_position_data[0])
                    y = float(body_position_data[1])
                    r = np.array([x, y])

                    step_position_data.append(r)

        self.nframes = len(self.data)

    def animInit(self):
        """
        Initiate the animation.
        """
        return self.patches

    def animate(self, i):
        """
        Animate the simulation.
        """
        # get the position data for the current step being animated
        step_data = self.data[self.count]

        # update the patches' positions
        for idx in range(len(step_data)):
            body_data = step_data[idx]
            self.patches[idx].center = (body_data[0], body_data[1])

        self.count += 1

        return self.patches

    def run(self):
        """
        Run the simulation of the given data
        """
        step_data = self.data[0]

        # set the axis limit to a little bit more than the initial max distance of any body
        ax_lim = max([np.linalg.norm(r) for r in step_data]) * 1.2

        # set up the animation plot
        fig = plt.figure()
        ax = plt.axes()
        ax.axis('scaled')
        ax.set_title("Simulation")
        ax.set_xlim(-ax_lim, ax_lim)
        ax.set_ylim(-ax_lim, ax_lim)

        # initiate a list to store the patches and create the initial patches
        self.patches = []

        for idx in range(len(step_data)):
            body_pos_data = step_data[idx]
            body_misc_data = self.misc[idx]
            self.patches.append(plt.Circle((body_pos_data[0], body_pos_data[1]),
                                           body_misc_data['radius'], color=body_misc_data['colour'],
                                           animated=True))

        for patch in self.patches:
            ax.add_patch(patch)

        self.count += 1

        # animate the data
        anim = FuncAnimation(fig, self.animate, init_func=self.animInit, frames=self.nframes-1, repeat=False, interval=1, blit=True)

        # plt.show()


def main():
    if len(argv) in [2, 3]:
        energy_data_file = argv[1]

        graph = Grapher(energy_data_file)
        graph.plot()

        if len(argv) == 4:
            position_data_file = argv[2]

            animation = Animator(position_data_file)
            animation.run()

        plt.show()

        return

    raise IOError("Input not valid.")


if __name__ == '__main__':
    main()
