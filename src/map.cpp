#include "map.h"

#include <raylib.h>
#include <cmath>

Point neighbour_offsets[4] = {
    {1, 0},
    {0, 1},
    {-1, 0},
    {0, -1},
};

constexpr int NEIGHBOURS_COUNT = sizeof(neighbour_offsets) / sizeof(neighbour_offsets[0]);

Map::Map(int width, int height) {
  this->width = width;
  this->height = height;
  this->data.resize(width * height);
}

char Map::get(int n) {
    return data[n];
}

char Map::get(int x, int y) {
  return data[y * width + x];
}

void Map::set(int n, char c) {
  data[n] = c;
}

void Map::set(int x, int y, char c) {
  data[y * width + x] = c;
}

size_t Map::size() const {
    return width * height;
}

void Map::load_from_image_file(const char* path) {
    Image img = LoadImage(path);

    this->width = img.width;
    this->height = img.height;
    this->data.resize(img.width * img.height);
    for(int y = 0; y < img.height; y++) {
        for(int x = 0; x < img.width; x++) {
          set(x, y, GetImageColor(img, x, y).r != 0);
        }
    }

    UnloadImage(img);
}

Point Map::node_to_point(Node i) const {
    Point p;
    p.x = i % width;
    p.y = i / width;

    return p;
};

Node Map::point_to_node(Point i) const {
    return i.y * width + i.x;
};

double Map::cost(Node from, Node to) {
    return 1;
}

int Map::distance(Node a, Node b) const {
    return ::distance(node_to_point(a), node_to_point(b));
}


int distance(Point a, Point b) {
    return abs(b.x - a.x) + abs(b.y - a.y);
};

// Iterator for generic path finding algorithms
MapNeighbours Map::neighbours(Node i) {
    return {*this, i };
}

MapNeighbours::MapNeighbours(Map& map, Node node) : map(map) {
    this->point = map.node_to_point(node);
}

MapIterator MapNeighbours::begin() {
    return MapIterator(map, point, 0);
}

MapIterator MapNeighbours::end() {
    return MapIterator(map, point, NEIGHBOURS_COUNT);
}

MapIterator::MapIterator(Map& map, Point p, size_t idx): 
    map(map), center(p), index(idx) {

    update_current();
}

MapIterator& MapIterator::operator++() {
    this->index++;
    update_current();
    return *this;
}

void MapIterator::update_current() {
    while(index < NEIGHBOURS_COUNT) {
        //Check that neighbour at index exists
        Point p = center + neighbour_offsets[index];
        if(p.x >= 0 && p.x < map.width && p.y >= 0 && p.y < map.height) {
            // If it does set current node
            if(map.get(p.x, p.y)) {
                this->current_node = map.point_to_node(p);
                break;
            }
        }

        // Otherwise try next neighbour
        index++;
    }
}

bool MapIterator::operator!=(MapIterator& other) const {
    return this->index != other.index;
}

Node MapIterator::operator*() const {
    return current_node;
}