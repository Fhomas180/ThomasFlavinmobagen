#include "Agent.h"
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include "World.h"

#include <algorithm>
using namespace std;
std::vector<Point2D> Agent::generatePath(World* w) {
  unordered_map<Point2D, Point2D> cameFrom;  // to build the flowfield and build the path
  queue<Point2D> frontier;                   // to store next ones to visit
  unordered_set<Point2D> frontierSet;        // OPTIMIZATION to check faster if a point is in the queue
  unordered_map<Point2D, bool> visited;      // use .at() to get data, if the element dont exist [] will give you wrong results

  // bootstrap state
  auto catPos = w->getCat();
  frontier.push(catPos);
  frontierSet.insert(catPos);
  Point2D borderExit = Point2D::INFINITE;  // if at the end of the loop we dont find a border, we have to return random points
  int sideSize = w->getWorldSideSize();
  int halfSize = sideSize / 2;
  while (!frontier.empty()) {
    // get the current from frontier
    // remove the current from frontierset
    // mark current as visited
    // getVisitableNeightbors(world, current) returns a vector of neighbors that are not visited, not cat, not block, not in the queue
    // iterate over the neighs:
    // for every neighbor set the cameFrom
    // enqueue the neighbors to frontier and frontierset
    // do this up to find a visitable border and break the loop
    Point2D current = frontier.front();
    frontier.pop();

    // remove the current from frontierset
    frontierSet.erase(current);

    // mark current as visited
    visited[current] = true;

    // check if we reached border
    if (abs(current.x) == halfSize || abs(current.y) == halfSize) {
      borderExit = current;
      break;
    }

    // get visitable neighbors
    auto neighbors = w->getVisitableNeighbors(catPos, current);

    // iterate over the neighbors
    for (const auto& neighbor : neighbors) {
      // skip if already visited
      if (visited.count(neighbor) && visited[neighbor]) continue;

      // skip if already in frontier
      if (frontierSet.count(neighbor)) continue;

      // set the cameFrom
      cameFrom[neighbor] = current;

      // enqueue the neighbor
      frontier.push(neighbor);
      frontierSet.insert(neighbor);
    }
  }

  // if the border is not infinity, build the path from border to the cat
  if (borderExit != Point2D::INFINITE) {
    vector<Point2D> path;
    Point2D current = borderExit;

    while (current != catPos) {
      path.push_back(current);
      current = cameFrom[current];
    }

    // reverse to get path from cat to border
    std::reverse(path.begin(), path.end());
    return path;
  }

  // if the border is not infinity, build the path from border to the cat using the camefrom map
  // if there isnt a reachable border, just return empty vector
  // if your vector is filled from the border to the cat, the first element is the catcher move, and the last element is the cat move
  return vector<Point2D>();
}