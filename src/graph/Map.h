#pragma once

#include "Point.h"
#include <stdint.h>
#include <vector>

typedef uint8_t MapType;
typedef int32_t Node;
template <typename T> using Path = std::vector<T>;

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
  bool operator!=(const MapIterator &other) const;
  bool operator==(const MapIterator &other) const;
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
  std::vector<MapType> data;

public:
  explicit Map(int width = 0, int height = 0);

  static constexpr int NEIGHBOURS_COUNT = 8;
  static Point neighbour_offsets[NEIGHBOURS_COUNT];

  static std::function<bool(Node)> node_eq_predicate(Node n1) {
    return [=](Node n2) -> bool { return n1 == n2; };
  };

  int width() const { return width_; }
  int height() const { return height_; }
  size_t size() const { return data.size(); }
  MapType *data_ptr() { return data.data(); }

  MapType get(Node n);
  MapType get(Point p);

  void set(Point p, MapType c);

  bool in_bounds(Point p) const;

  void load_from_image_file(const char *path);

  Point node_to_point(Node i) const;
  Node point_to_node(Point p) const;
  float distance(Node a, Node b) const;
  static float distance(Point a, Point b);

  float cost(Node from, Node to);

  MapNeighbours neighbours(Node i);
};

namespace std {
  template<>
  struct iterator_traits<MapIterator> {
    typedef ptrdiff_t            difference_type;
    typedef Node                 value_type;
    typedef Node*                pointer;
    typedef Node&                reference;
    typedef forward_iterator_tag iterator_category;
  };
}
