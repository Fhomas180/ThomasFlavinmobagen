#include "PrimExample.h"
#include "../World.h"
#include "Random.h"

bool PrimExample::Step(World* w) {
  int sideOver2 = w->GetSize() / 2;

  // todo: code this
  if (!initialized) {
    w->SetNodeColor({0, 0}, Color::White);

    auto neighbors = getVisitables(w, {0, 0});
    for (const auto& n : neighbors) {
      toBeVisited.push_back(n);
    }

    initialized = true;
    return true;
  }

  if (toBeVisited.empty()) {
    return false;
  }
  int randomIndex = Random::Range(0, toBeVisited.size());
  Point2D current = toBeVisited[randomIndex];
  toBeVisited.erase(toBeVisited.begin() + randomIndex);
  auto visitedNeighbors = getVisitedNeighbors(w, current);

  if (visitedNeighbors.size() > 0) {

    Point2D neighbor = visitedNeighbors[Random::Range(0, visitedNeighbors.size())];
    // Remove wall between current and neighbor
    if (neighbor.y < current.y) {
      // neighbor is NORTH
      w->SetNorth(current, false);
      w->SetSouth(neighbor, false);
    } else if (neighbor.x > current.x) {
      // neighbor is EAST
      w->SetEast(current, false);
      w->SetWest(neighbor, false);
    } else if (neighbor.y > current.y) {
      // neighbor is SOUTH
      w->SetSouth(current, false);
      w->SetNorth(neighbor, false);
    } else if (neighbor.x < current.x) {
      // neighbor is WEST
      w->SetWest(current, false);
      w->SetEast(neighbor, false);
    }

    w->SetNodeColor(current, Color::White);

    auto unvisitedNeighbors = getVisitables(w, current);
    for (const auto& n : unvisitedNeighbors) {
      toBeVisited.push_back(n);
    }
  }

  return true;
}
void PrimExample::Clear(World* world) {
  toBeVisited.clear();
  initialized = false;
}

std::vector<Point2D> PrimExample::getVisitables(World* w, const Point2D& p) {
  auto sideOver2 = w->GetSize() / 2;
  std::vector<Point2D> visitables;
  auto clearColor = Color::DarkGray;

  // todo: code this
  Point2D upNeighbor = {p.x, p.y - 1};
  Point2D downNeighbor = {p.x, p.y + 1};
  Point2D leftNeighbor = {p.x - 1, p.y};
  Point2D rightNeighbor = {p.x + 1, p.y};
  if (upNeighbor.x >= -sideOver2 && upNeighbor.x <= sideOver2 &&
     upNeighbor.y >= -sideOver2 && upNeighbor.y <= sideOver2) {
    if (w->GetNodeColor(upNeighbor) == clearColor) {  // Not visited yet
      visitables.push_back(upNeighbor);
    }
     }if (rightNeighbor.x >= -sideOver2 && rightNeighbor.x <= sideOver2 && rightNeighbor.y >= -sideOver2 && rightNeighbor.y <= sideOver2) {
       if (w->GetNodeColor(rightNeighbor) == clearColor) {  // Not visited yet
         visitables.push_back(rightNeighbor);
       }
     }if (downNeighbor.x >= -sideOver2 && downNeighbor.x <= sideOver2 && downNeighbor.y >= -sideOver2 && downNeighbor.y <= sideOver2) {
       if (w->GetNodeColor(downNeighbor) == clearColor) {  // Not visited yet
         visitables.push_back(downNeighbor);
       }
     }if (leftNeighbor.x >= -sideOver2 && leftNeighbor.x <= sideOver2 && leftNeighbor.y >= -sideOver2 && leftNeighbor.y <= sideOver2) {
       if (w->GetNodeColor(leftNeighbor) == clearColor) {  // Not visited yet
         visitables.push_back(leftNeighbor);
       }
     }


  return visitables;
}

std::vector<Point2D> PrimExample::getVisitedNeighbors(World* w, const Point2D& p) {
  std::vector<Point2D> deltas = {Point2D::UP, Point2D::DOWN, Point2D::LEFT, Point2D::RIGHT};
  auto sideOver2 = w->GetSize() / 2;
  std::vector<Point2D> neighbors;
  auto clearColor = Color::DarkGray;

  // todo: code this
  for (const auto& delta : deltas) {
    Point2D theNeighbor = {p.x + delta.x, p.y + delta.y};
    if (theNeighbor.x >= -sideOver2 && theNeighbor.x <= sideOver2 &&
        theNeighbor.y >= -sideOver2 && theNeighbor.y <= sideOver2) {
      if (w->GetNodeColor(theNeighbor) != clearColor) {
        neighbors.push_back(theNeighbor);
      }
        }
    return neighbors;
  }
}
