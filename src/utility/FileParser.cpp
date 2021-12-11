#include "FileParser.h"
#include <fstream>

FileParser::FileParser(std::string file_path) {
  this->file_path = std::move(file_path);
}

bool BitMapParser::build_graph(WeightedGraph& g) {
  std::ifstream graph_file(file_path);

  if (!graph_file.is_open())
    return false;

  int x_size, y_size;
  graph_file >> x_size >> y_size;

  std::vector<bool> is_path(x_size);

  for (int y = 0; y < y_size; y++) {
    for (int x = 0; x < x_size; x++) {
      bool current_is_path;
      graph_file >> current_is_path;

      vertex_t vertex_number = g.add_vertex(Point(x, y));

      if (current_is_path) {
        if (x > 0 && is_path.at(x - 1))
          g.add_edge(vertex_number - 1, vertex_number, 1);

        if (y > 0 && is_path.at(x))
          g.add_edge(vertex_number - x_size, vertex_number, 1);
      }
      is_path.at(x) = current_is_path;
    }
  }

  graph_file.close();
  return true;
}

bool DotParser::build_graph(WeightedGraph& g) {
  std::ifstream graph_file(file_path);

  std::string block;
  graph_file >> block;

  if (block != "graph") {
    std::cout << "Expected graph, aborting." << std::endl;
    graph_file.close();
    return false;
  }

  while (block != "{") {
    graph_file >> block;
  }

  while (true) {
    std::string node_name, label_str, pos_str;
    graph_file >> node_name;

    if (node_name == "edge") {
      block = node_name;
      break;
    }

    graph_file >> label_str >> pos_str >> block;

    int x = stoi(str_between(pos_str, '"', ','));
    int y = stoi(str_between(pos_str, ',', '"'));

    g.add_vertex(Point(x, y));
  }

  graph_file >> block >> block;

  while (true) {
    std::string node_1, direction, node_2;
    graph_file >> node_1;

    if (node_1 == "}")
      break;

    graph_file >> direction >> node_2 >> block >> block;

    vertex_t node_nr_1 = stoi(str_after(node_1, "node"));
    vertex_t node_nr_2 = stoi(str_between(node_2, "node", ";"));

    g.add_edge(node_nr_1, node_nr_2, 1);
  }

  graph_file.close();
  return true;
}

std::string FileParser::str_between(const std::string& str, char pre, char post) {
  std::string s = str.substr(str.find(pre) + 1);
  return s.substr(0, s.find(post));
}

std::string FileParser::str_between(const std::string& str, const std::string& pre, const std::string& post) {
  std::string s = str.substr(str.find(pre) + pre.length());
  return s.substr(0, s.find(post));
}

std::string FileParser::str_after(const std::string& str, const std::string& pre) {
  return str.substr(str.find(pre) + pre.length());
}
