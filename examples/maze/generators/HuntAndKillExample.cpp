#include "HuntAndKillExample.h"
#include "../World.h"
#include "Random.h"
#include <climits>
bool HuntAndKillExample::Step(World* w) {
  // todo: code this
  if (stack.empty() && visited.empty()) {
    stack.push_back({0, 0});
    return true;
  }if (!stack.empty()) {
    auto current = stack.back();
    auto neighbors = getVisitables(w, current);
    if (neighbors.size() > 0) {
      visited[current.x][current.y] = true;
      Point2D currentNeighbor;
      if (neighbors.size() == 1) {
        currentNeighbor = neighbors[0];

      }else {
        currentNeighbor = neighbors[Random::Range(0, neighbors.size())];
      }if (currentNeighbor.y < current.y) {
        // neighbor is NORTH
        w->SetNorth(current, false);
        w->SetSouth(currentNeighbor, false);
      } else if (currentNeighbor.x > current.x) {
        // neighbor is EAST
        w->SetEast(current, false);
        w->SetWest(currentNeighbor, false);
      } else if (currentNeighbor.y > current.y) {
        // neighbor is SOUTH
        w->SetSouth(current, false);
        w->SetNorth(currentNeighbor, false);
      } else if (currentNeighbor.x < current.x) {
        // neighbor is WEST
        w->SetWest(current, false);
        w->SetEast(currentNeighbor, false);
      }
      stack.push_back(currentNeighbor);
    }else {
      stack.pop_back();
    }
  }else {
    Point2D huntCell = randomStartPoint(w);

    if (huntCell.x == INT_MAX) {
      return false; // Done
    }

    auto visitedNeighbors = getVisitedNeighbors(w, huntCell);

    if (visitedNeighbors.size() > 0) {
      visited[huntCell.x][huntCell.y] = true;

      // Pick a visited neighbor
      Point2D neighbor = visitedNeighbors[Random::Range(0, visitedNeighbors.size())];

      // Remove wall between huntCell and neighbor
      if (neighbor.y < huntCell.y) {
        w->SetNorth(huntCell, false);
        w->SetSouth(neighbor, false);
      } else if (neighbor.x > huntCell.x) {
        w->SetEast(huntCell, false);
        w->SetWest(neighbor, false);
      } else if (neighbor.y > huntCell.y) {
        w->SetSouth(huntCell, false);
        w->SetNorth(neighbor, false);
      } else if (neighbor.x < huntCell.x) {
        w->SetWest(huntCell, false);
        w->SetEast(neighbor, false);
      }

      stack.push_back(huntCell);
    }
  }
  return true;
}
void HuntAndKillExample::Clear(World* world) {
  visited.clear();
  stack.clear();
  auto sideOver2 = world->GetSize() / 2;

  for (int i = -sideOver2; i <= sideOver2; i++) {
    for (int j = -sideOver2; j <= sideOver2; j++) {
      visited[i][j] = false;
    }
  }
}
Point2D HuntAndKillExample::randomStartPoint(World* world) {
  // Todo: improve this if you want

  auto sideOver2 = world->GetSize() / 2;

  for (int y = -sideOver2; y <= sideOver2; y++)
    for (int x = -sideOver2; x <= sideOver2; x++)
      if (!visited[y][x]) return {x, y};
  return {INT_MAX, INT_MAX};
}

std::vector<Point2D> HuntAndKillExample::getVisitables(World* w, const Point2D& p) {
  auto sideOver2 = w->GetSize() / 2;
  std::vector<Point2D> visitables;

  // todo: code this
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


  return visitables;
}
std::vector<Point2D> HuntAndKillExample::getVisitedNeighbors(World* w, const Point2D& p) {
  std::vector<Point2D> deltas = {{-1, 0}, {0, -1}, {1, 0}, {0, 1}};
  auto sideOver2 = w->GetSize() / 2;
  std::vector<Point2D> neighbors;

  // todo: code this
  for (const auto& delta : deltas) {
    Point2D theNeighbor = {p.x + delta.x, p.y + delta.y};
    if (theNeighbor.x >= -sideOver2 && theNeighbor.x <= sideOver2 && theNeighbor.y >= -sideOver2 && theNeighbor.y <= sideOver2) {
      if (visited[theNeighbor.x][theNeighbor.y]) {
        neighbors.push_back(theNeighbor);
      }
    }


  }


  return neighbors;
}
