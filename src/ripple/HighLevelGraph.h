#pragma once

#include <vector>

#include <glm/gtx/norm.hpp>
#include <glm/glm.hpp>

#include "graph/Map.h"

// Old knn based approach
Path<Node> create_high_level_path(Map& map, Node source, Node goal);


// New grid based approach
class GridHighLevelGraph {
    Map& map;
    glm::vec2 cell;

public:
    Map grid;
    std::vector<std::vector<int>> adj;

    // Creates the high level path for a given map
    GridHighLevelGraph(Map& map);

    // High level path from 'source' to 'goal' that uses up to 'num' points
    Path<Node> create_high_level_path(Node source, Node goal, int num);



    // Search implementation
    std::vector<int> &neighbours(int i) { return adj[i]; }
    double cost(int i, int j);
    int distance(int i, int j);
    size_t size() { return grid.size(); }



    // Debug utils

    // Full high level path from 'source' to 'goal'
    std::vector<Point> get_full_path(Point source, Point goal);

    // Refine the high level path so that it has at most num nodes
    std::vector<Point> refine_high_level_path(const std::vector<Point>& path, int num);

    // Return the closest point to 'point' that exists on the grid
    Point get_closest_point_on_grid(Point point);

    // Transform coordinates of a point on the grid to a point on the map
    Point grid_to_map(Point p);
    glm::vec2 grid_to_map(glm::vec2 p);
};
