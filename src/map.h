#pragma once

#include <vector>

typedef int Node;
typedef std::vector<Node> Path;

#define INVALID_NODE (-1)

struct Point {
    int x;
    int y;

    Point(){}
    
    Point(int x, int y){
        this->x = x;
        this->y = y;
    }

    inline Point operator+(Point& other) {
        return Point(x + other.x, y + other.y);
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
    Point center;
    size_t index;

    Node current_node;
    
    void update_current();

public:
    MapIterator(Map& map, Point p, size_t idx);

    MapIterator& operator++();
    bool operator!=(MapIterator& other) const;
    Node operator*() const;
};

class MapNeighbours {
    Map& map;
    Point point;

public:
    MapNeighbours(Map& map, Node node);
    MapIterator begin();
    MapIterator end();    
};

class Map {
private:
  std::vector<char> data;

public:
    Map(int width = 0, int height = 0);
    int width, height;

    char get(int n);
    char get(int x, int y);

    void set(int n, char c);
    void set(int x, int y, char c);

    size_t size() const;

    void load_from_image_file(const char* path);
    
    Point node_to_point(Node i) const;
    Node point_to_node(Point p) const;
    int distance(Node a, Node b) const;
    double cost(Node from, Node to);

    MapNeighbours neighbours(Node i);
};


int distance(Point a, Point b);

extern Point neighbour_offsets[4];