# Parallel Ripple Search

Project by Gavin Gray, Lorenzo Liso, Dario Mylonopoulos, Ishaan Shamanna and Lucas Weitzendorf for the course Design of Parallel and High-Performance Computing HS 2021 at ETH Zurich


## Build
The project requires CMake version 3.16 (or newer).
The repository and all the submodules can be cloned with the following command:
```bash
git clone --recursive <url>
```
Due to the long compilation time of oneTBB it was not included as a build dependency, if a version of oneTBB is not already present in the system it can be built and installed with the following commands (see [here](https://github.com/oneapi-src/oneTBB/blob/master/INSTALL.md) for the complete installation reference):
```bash
cd oneTBB
mkdir build && cd build
cmake ..
make
sudo make install
```

Parallel ripple search and the other binaries can then be built with the following commands from the root directory of the repository:
```bash
mkdir build && cd build
cmake ..
make
```

## Run

The build process described above produces the following binaries in the `build` directory
```bash
./bench # Runs the benchmarks for each algorithm and outputs measurements into <algorithm>.lsb
./graph_view <path> <source> <goal> # Runs the algorithms and displays results
./high  # Generates the high level graph and displays it on top of the original graph
```

## Project Structure
The source code is structured as follows:
- `src/benchmark`: utilities for loading benchmark data
- `src/bin`: source code for the output binaries
- `src/graph`: graph and map data structures
- `src/reference`: source code of reference implementations of pathfinding algorithms
- `src/ripple`: Parallel Ripple Search implementation
- `src/utility`: utilities for data structures, parsing, and performance measurement

## Dependencies
- [oneTBB](https://oneapi-src.github.io/oneTBB/) - concurrent queues
- [LibSciBench](https://spcl.inf.ethz.ch/Research/Performance/LibLSB/) - performance measurement
- [GLM](https://glm.g-truc.net/0.9.9/index.html) - math utils
- [Raylib](https://www.raylib.com/) - ui and visualization

## References
[1] Sandy Brand and Rafael Bidarra, “Parallel ripple search – scalable and efficient pathfinding for multi-core architectures”,  in *Proceedings of the 4th International Conference on Motion in Games*, Berlin, Heidelberg, 2011,MIG’11, p. 290–303, Springer-Verlag.

[2] Nathan  R.  Sturtevant, “Benchmarks  for  grid-basedpathfinding”, *IEEE Transactions on Computational Intelligence and AI in Games*, vol. 4, no. 2, pp. 144–148,2012.

[3] Yngvi Bjornsson, Markus Enzenberger, Robert Holte and Jonathan Schaeffer,  “Fringe search:  Beating a* atpathfinding on game maps.”, 01 2005