#pragma once
#include <vector>
#include <utility>
#include "math/Point2D.h"

class World;
struct Agent {
public:
  explicit Agent() = default;

  virtual Point2D Move(World*) = 0;

  std::vector<Point2D> generatePath(World* w);
};