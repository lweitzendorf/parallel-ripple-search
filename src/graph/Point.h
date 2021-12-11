#pragma once

#include <iostream>
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
