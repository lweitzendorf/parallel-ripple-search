
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>

int main(int argc, char const *argv[]) {
  const char* file_name = argv[1];
  std::ifstream graph_file(file_name);

  int x_size, y_size;
  graph_file >> x_size >> y_size;

  std::vector<char> map(x_size * y_size);
  for(int y = 0; y < y_size; y++) {
    for(int x = 0; x < x_size; x++) {
        graph_file >> map[y * x_size + x];
    }  
  }

  int x_samples = x_size / 10; 
  int y_samples = y_size / 10;
  int samples = x_samples * y_samples;
  
  std::vector<std::pair<int, int>> nodes(samples);
  for(int i = 0; i < samples; i++) {
      int x = rand() % x_size;
      int y = rand() % y_size;

      nodes[i] = std::make_pair(x, y);
  }

}