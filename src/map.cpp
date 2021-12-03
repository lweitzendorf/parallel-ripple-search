#include "map.h"

#include <raylib.h>
#include <cmath>

Point Map::neighbour_offsets[] = {
        { -1, -1 },
        { -1,  1 },
        {  1, -1 },
        {  1,  1 },

        { -1,  0 },
        {  0, -1 },
        {  1,  0 },
        {  0,  1 },
};

Map::Map(int width, int height) {
  this->width_ = width;
  this->height_ = height;
  this->data.resize(width * height);
}

char Map::get(Point p) {
  return data[point_to_node(p)];
}

void Map::set(Point p, char c) {
  data[point_to_node(p)] = c;
}

bool Map::in_bounds(Point p) const {
  return p.x >= 0 && p.x < width_ && p.y >= 0 && p.y < height_;
}

void Map::load_from_image_file(const char* path) {
    Image img = LoadImage(path);

    this->width_ = img.width;
    this->height_ = img.height;
    this->data.resize(img.width * img.height);
    for(int y = 0; y < img.height; y++) {
        for(int x = 0; x < img.width; x++) {
          set(Point(x, y), GetImageColor(img, x, y).r != 0);
        }
    }

    UnloadImage(img);
}

Point Map::node_to_point(Node i) const {
    Point p;
    p.x = i % width_;
    p.y = i / width_;

    return p;
};

Node Map::point_to_node(Point i) const {
    return i.y * width_ + i.x;
};

double Map::cost(Node from, Node to) {
    return 1;
}

float Map::distance(Node a, Node b) const {
    return distance(node_to_point(a), node_to_point(b));
}

float Map::distance(Point a, Point b) {
  //return abs(b.x - a.x) + abs(b.y - a.y);
  Point diff(b.x - a.x, b.y - a.y);
  return sqrtf(diff.x * diff.x + diff.y * diff.y) * 0.5f;
}

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
    return MapIterator(map, point, Map::NEIGHBOURS_COUNT);
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
    while(index < Map::NEIGHBOURS_COUNT) {
        //Check that neighbour at index exists
        Point p = center + Map::neighbour_offsets[index];
        if(map.in_bounds(p) && map.get(p)) {
            // If it does set current node
            this->current_node = map.point_to_node(p);
            break;
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