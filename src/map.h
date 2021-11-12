#pragma once

#include <vector>

typedef int Node;
#define INVALID_NODE -1

struct Point {
    int x;
    int y;
};

class Map {
public:
    std::vector<char> data;
    int width, height;

    char get(int x, int y);
    void load_from_image_file(const char* path);
    
    Point node_to_point(Node i);
    Node point_to_node(Point p);
    int distance(Node a, Node b);
};


int distance(Point a, Point b);