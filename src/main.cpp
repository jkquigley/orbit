#include <iostream>
#include "Parser.h"


void help()
{
    std::cout << "[Usage]\n"
              << "./orbit animate <cfg file>\n"
              << "./orbit animate <cfg file> <energy data file>\n"
              << "./orbit animate <cfg file> <energy data file> <position data file>\n"
              << "./orbit record <cfg file> <energy data file>\n"
              << "./orbit record <cfg file> <energy data file> <position data file>\n";
}


int main(int argc, char** argv)
{
    Parser parser;

    if (argc == 1)
    {
        if (strcmp(argv[1], "help") == 0)
        {
            help();
            return 0;
        }
    }

    else if (strcmp(argv[1], "animate") == 0)
    {
        if (argc == 3)
        {
            const char *infile = argv[2];

            auto simulation = parser.load(infile);

            simulation.run();

            return 0;
        }

        else if (argc == 4)
        {
            const char *infile = argv[2];
            const char *energy_outfile = argv[3];

            auto simulation = parser.load(infile);

            simulation.run(energy_outfile, true);

            return 0;
        }

        else if (argc == 5)
        {
            const char *infile = argv[2];
            const char *energy_outfile = argv[3];
            const char *position_outfile = argv[4];

            auto simulation = parser.load(infile);

            simulation.run(energy_outfile, position_outfile, true);

            return 0;
        }
    }

    else if (strcmp(argv[1], "record") == 0)
    {
        if (argc == 4)
        {
            const char *infile = argv[2];
            const char *energy_outfile = argv[3];

            auto simulation = parser.load(infile);

            simulation.run(energy_outfile, false);

            return 0;
        }

        else if (argc == 5)
        {
            const char *infile = argv[2];
            const char *energy_outfile = argv[3];
            const char *position_outfile = argv[4];

            auto simulation = parser.load(infile);

            simulation.run(energy_outfile, position_outfile, false);

            return 0;
        }
    }

    help();
    return -1;
}
