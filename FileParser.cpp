#include "FileParser.h"
#include <fstream>
#include <utility>

FileParser::FileParser(std::string file_path) {
  this->file_path = std::move(file_path);
}

bool BitMapParser::build_graph(WeightedGraph& g) {
  std::ifstream graph_file(file_path);

  int x_size, y_size;
  graph_file >> x_size >> y_size;

  std::vector<bool> is_path(x_size);

  for (int y = 0; y < y_size; y++) {
    for (int x = 0; x < x_size; x++) {
      bool current_is_path;
      graph_file >> current_is_path;

      int vertex_number = g.add_vertex(x, y);

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

  if (block != "digraph") {
    std::cout << "Expected digraph, aborting." << std::endl;
    graph_file.close();
    return false;
  }

  while (block != "{") {
    graph_file >> block;
  }

  while (true) {
    std::string node_name, label_str, pos_str;
    graph_file >> node_name;

    if (node_name == "subgraph") {
      block = "subgraph";
      break;
    }

    graph_file >> label_str >> pos_str;

    int x = stoi(str_between(pos_str, '"', ','));
    int y = stoi(str_between(pos_str, ',', '"'));

    g.add_vertex(x, y);
  }

  while (block != "{") {
    graph_file >> block;
  }
  graph_file >> block;

  if (block != "edge") {
    std::cout << "Expected edge, aborting." << std::endl;
    graph_file.close();
    return false;
  }

  std::string edge_direction;
  graph_file >> edge_direction;

  if (str_between(edge_direction, '=', ']') != "none") {
    std::cout << "Only undirected edges supported." << std::endl;
    graph_file.close();
    return false;
  }

  while (true) {
    std::string node_1, direction, node_2;
    graph_file >> node_1;

    if (node_1 == "}")
      break;

    graph_file >> direction >> node_2;

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
