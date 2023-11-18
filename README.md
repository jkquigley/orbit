# orbit
Planetry orbit simulation.

## Building
To build the simulation, `cmake` and `make` are required. To install these on a Debian/Ubuntu machine run
```bash
sudo apt install cmake make
```

To build, `cd` into the repositories top directory and run
```bash
mkdir build
cd build
cmake ..
make
cd ..
```

You should now have an executable called `orbit`.

## Usage
To run the simulation, use the command
```bash
./orbit <path to .yml config>
```

## Config Files
The config files are `.yml`. Have a look at the default file `cfg/solar.yml`. There you can see the important parameters and how to add bodies to the simulation.
