#pragma once

#include <vector>
#include <boost/functional/hash.hpp>

struct Point {
  int x;
  int y;

  Point() {}

  Point(int x, int y) {
    this->x = x;
    this->y = y;
  }

  inline Point operator+(Point &other) const {
    return {x + other.x, y + other.y};
  }

  inline bool operator==(const Point &p1) const {
    return this->x == p1.x && this->y == p1.y;
  }

  inline bool operator!=(const Point &p1) const {
    return this->x != p1.x || this->y != p1.y;
  }

  inline bool operator<(const Point &p1) const { return false; }
};

namespace std {
  template<>
  struct hash<Point> {
    size_t operator()(const Point &p) const {
      std::size_t seed = 0;
      boost::hash_combine(seed, p.x);
      boost::hash_combine(seed, p.y);

      return seed;
    }
  };
}

namespace Map {
  static constexpr int NEIGHBOURS_COUNT = 8;
  static const Point neighbour_offsets[NEIGHBOURS_COUNT] = {
          {-1, -1}, {-1, 1}, {1, -1}, {1, 1},
          {-1, 0},  {0, -1}, {1, 0},  {0, 1},
  };
}

