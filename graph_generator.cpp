#include <iostream>
#include <fstream>
#include <random>
#include <cstdlib>

#define WALL_PERCENTAGE 10

int main(int argc, char const *argv[]) {
  if (argc != 3) {
    std::cout << "Usage: ./graph_generator <x_dim> <y_dim>" << std::endl;
    return -1;
  }

  std::random_device r;
  std::default_random_engine e1(r());
  std::uniform_int_distribution<int> uniform_dist(0, 99);

  long x = strtol(argv[1], nullptr, 10), y = strtol(argv[2], nullptr, 10);

  std::ofstream file;
  file.open ("graph.in");
  file << x << " " << y << std::endl;

  for (int i = 0; i < x; i++) {
    for (int j = 0; j < y; j++) {
      file << (uniform_dist(e1) >= WALL_PERCENTAGE) << " ";
    }
    file << std::endl;
  }
  
  file.close();

  return 0;
}