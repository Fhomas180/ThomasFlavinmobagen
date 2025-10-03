#include "../World.h"
#include "Random.h"
#include "RecursiveBacktrackerExample.h"
#include <climits>
bool RecursiveBacktrackerExample::Step(World* w) {
  // todo: implement this
  Point2D chosenNeighbor;
  if (stack.empty() && visited.empty()) {
    stack.push_back({0, 0});

    return true;
  }

  // Check if done
  if (stack.empty()) {
    return false;
  }

  auto start = stack.back();
  stack.pop_back();
  auto getNeighbor = getVisitables(w, start);

    if (getNeighbor.size() > 0) {
      visited[start.x][start.y] = true;
      if (getNeighbor.size() == 1) {
        chosenNeighbor = getNeighbor[0];
      }else {
        chosenNeighbor = getNeighbor[Random::Range(0,getNeighbor.size())];
      }if (chosenNeighbor.y < start.y) {
        // neighbor is NORTH
        w->SetNorth(start, false);
        w->SetSouth(chosenNeighbor, false);
      } else if (chosenNeighbor.x > start.x) {
        // neighbor is EAST
        w->SetEast(start, false);
        w->SetWest(chosenNeighbor, false);
      }else if (chosenNeighbor.y > start.y) {
        // neighbor is SOUTH
        w->SetSouth(start,false);
        w->SetNorth(chosenNeighbor, false);
      }else if (chosenNeighbor.x < start.x) {
        // neighbor is WEST
        w->SetWest(start, false);
        w->SetEast(chosenNeighbor, false);
        // remove west wall from start and east wall from neighbor
      }
      stack.push_back(chosenNeighbor);

    }
    return false;
  }


void RecursiveBacktrackerExample::Clear(World* world) {
  visited.clear();
  stack.clear();
  auto sideOver2 = world->GetSize() / 2;

  for (int i = -sideOver2; i <= sideOver2; i++) {
    for (int j = -sideOver2; j <= sideOver2; j++) {
      visited[i][j] = false;
    }
  }
}

Point2D RecursiveBacktrackerExample::randomStartPoint(World* world) {
  auto sideOver2 = world->GetSize() / 2;

  // todo: change this if you want
  for (int y = -sideOver2; y <= sideOver2; y++)
    for (int x = -sideOver2; x <= sideOver2; x++)
      if (!visited[y][x]) return {x, y};
  return {INT_MAX, INT_MAX};
}

std::vector<Point2D> RecursiveBacktrackerExample::getVisitables(World* w, const Point2D& p) {
  auto sideOver2 = w->GetSize() / 2;
  std::vector<Point2D> visitables;
  Point2D upNeighbor = {p.x, p.y - 1};
  Point2D downNeighbor = {p.x, p.y + 1};
  Point2D leftNeighbor = {p.x - 1, p.y};
  Point2D rightNeighbor = {p.x + 1, p.y};
  if (upNeighbor.x >= -sideOver2 && upNeighbor.x <= sideOver2 &&
     upNeighbor.y >= -sideOver2 && upNeighbor.y <= sideOver2) {
    if (!visited[upNeighbor.x][upNeighbor.y]) {
      visitables.push_back(upNeighbor);
    }
  }if (rightNeighbor.x >= -sideOver2 && rightNeighbor.x <= sideOver2 && rightNeighbor.y >= -sideOver2 && rightNeighbor.y <= sideOver2) {
    if (!visited[rightNeighbor.x][rightNeighbor.y]) {
      visitables.push_back(rightNeighbor);

    }
  }if (downNeighbor.x >= -sideOver2 && downNeighbor.x <= sideOver2 && downNeighbor.y >= -sideOver2 && downNeighbor.y <= sideOver2) {
    if (!visited[downNeighbor.x][downNeighbor.y]) {
      visitables.push_back(downNeighbor);
    }
  }if (leftNeighbor.x >= -sideOver2 && leftNeighbor.x <= sideOver2 && leftNeighbor.y >= -sideOver2 && leftNeighbor.y <= sideOver2) {
    if (!visited[leftNeighbor.x][leftNeighbor.y]) {
      visitables.push_back(leftNeighbor);

    }
  }

    // todo: implement this

    return visitables;

}
