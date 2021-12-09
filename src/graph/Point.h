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