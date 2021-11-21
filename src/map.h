#pragma once

#include <vector>

typedef int Node;
#define INVALID_NODE (-1)

struct Point {
    int x;
    int y;

    Point(){}
    
    Point(int x, int y){
        this-> x = x;
        this-> y = y;
    }
    inline bool operator== (const Point& p1) const {
        return this->x == p1.x && this->y == p1.y;
    }

    inline bool operator!= (const Point& p1) const {
        return this->x != p1.x || this->y != p1.y;
    }

    inline bool operator<(const Point& p1) const {
        return false;
    }
};

class Map;

class MapIterator {
private:
    Map& map;
    Point p;
    int index;

public:      
    MapIterator& operator++();
    bool operator!=(MapIterator& other);
    int get_cost();
    int get_node();
};

class MapNeighbours {
    Map& map;
    Node node;

public:
    MapIterator begin();
    MapIterator end();    
};

class Map {
public:
    Map(int width = 0, int height = 0);

    std::vector<char> data;
    int width, height;

    char get(int x, int y);
    void set(int x, int y, char c);

    void load_from_image_file(const char* path);
    
    Point node_to_point(Node i);
    Node point_to_node(Point p);
    int distance(Node a, Node b);

    MapNeighbours neighbours(Node i);
};


int distance(Point a, Point b);

extern Point neighbour_offsets[4];