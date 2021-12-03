#pragma once

#include <vector>

typedef int Node;
template <typename T> using Path = std::vector<T>;

#define INVALID_NODE (-1)

struct Point {
  int x;
  int y;

  Point() {}

  Point(int x, int y) {
    this->x = x;
    this->y = y;
  }

  inline Point operator+(Point &other) {
    return Point(x + other.x, y + other.y);
  }

  inline bool operator==(const Point &p1) const {
    return this->x == p1.x && this->y == p1.y;
  }

  inline bool operator!=(const Point &p1) const {
    return this->x != p1.x || this->y != p1.y;
  }

  inline bool operator<(const Point &p1) const { return false; }
};

class Map;

class MapIterator {
private:
  Map &map;
  Point center;
  size_t index;

  Node current_node;

  void update_current();

public:
  MapIterator(Map &map, Point p, size_t idx);

  MapIterator &operator++();
  bool operator!=(MapIterator &other) const;
  Node operator*() const;
};

class MapNeighbours {
  Map &map;
  Point point;

public:
  MapNeighbours(Map &map, Node node);
  MapIterator begin();
  MapIterator end();
};

class Map {
private:
  int width_, height_;
  std::vector<char> data;

public:
  explicit Map(int width = 0, int height = 0);

  static Point neighbour_offsets[8];
  static constexpr int NEIGHBOURS_COUNT =
      sizeof(neighbour_offsets) / sizeof(neighbour_offsets[0]);

  int width() const { return width_; }
  int height() const { return height_; }
  size_t size() const { return data.size(); }

  char get(Point p);
  void set(Point p, char c);

  bool in_bounds(Point p) const;

  void load_from_image_file(const char *path);

  Point node_to_point(Node i) const;
  Node point_to_node(Point p) const;
  float distance(Node a, Node b) const;
  static float distance(Point a, Point b);

  double cost(Node from, Node to);

  MapNeighbours neighbours(Node i);
};
