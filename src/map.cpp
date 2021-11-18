#include "map.h"

#include <raylib.h>
#include <cmath>


Point neighbour_offsets[4] = {
    {1, 0},
    {0, 1},
    {-1, 0},
    {0, -1},
};

char Map::get(int x, int y) {
    return data[y * width + x];
}

void Map::load_from_image_file(const char* path) {
    Image img = LoadImage(path);

    this->width = img.width;
    this->height = img.height;
    this->data.resize(img.width * img.height);
    for(int y = 0; y < img.height; y++) {
        for(int x = 0; x < img.width; x++) {
            this->data[y * img.width + x] = GetImageColor(img, x, y).r != 0;
        }
    }

    UnloadImage(img);
}

Point Map::node_to_point(Node i) {
    Point p;
    p.x = i % width;
    p.y = i / width;

    return p;
};


Node Map::point_to_node(Point i){
    return i.y * width + i.x;
};

int Map::distance(Node a, Node b) {
    return ::distance(node_to_point(a), node_to_point(b));
}


int distance(Point a, Point b) {
    return abs(b.x - a.x) + abs(b.y - a.y);
};