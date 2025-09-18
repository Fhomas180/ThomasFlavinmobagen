#include "JohnConway.h"

#include <unordered_map>

// Reference: https://playgameoflife.com/info
void JohnConway::Step(World& world) {
  // todo: implement
  std::pmr::unordered_map<Point2D, Point2D> map;
  for (int x = 0; x < world.SideSize(); x++) {
    for (int y = 0; y < world.SideSize(); y++) {
      Point2D point = {x,y};
      if (world.Get(point)) {
        map.insert({point, point});
      }
    }
  }
  for (int x = 0; x < world.SideSize(); x++) {
    for (int y = 0; y < world.SideSize(); y++) {
      Point2D point = {x, y};
      bool isAlive = map.count(point) > 0;
      int neighbors = CountNeighbors(world, point);

      bool nextState = false;
      if (isAlive && (neighbors == 2 || neighbors == 3)) {
        nextState = true;
      } else if (!isAlive && neighbors == 3) {
        nextState = true;
      }

      world.SetNext(point, nextState);
    }
  }


}

int JohnConway::CountNeighbors(World& world, Point2D point) {
  // todo: implement
  int count = 0;
  for (int dx = -1; dx <= 1; dx++) {
    for (int dy = -1; dy <= 1; dy++) {
      if (dx == 0 && dy == 0) continue;
      Point2D neighbor = {point.x + dx, point.y + dy};
      if (world.Get(neighbor)) {
        count++;
      }
    }
  }
  return count;
}
