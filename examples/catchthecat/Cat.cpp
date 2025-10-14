#include "Cat.h"
#include "World.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include <vector>
#include <stdexcept>
struct Node {
  std::pair<int, int> pos;
  float fScore;
  bool operator>(const Node& other) const {
    return fScore > other.fScore;
  }
};
struct PairHash {
  size_t operator()(const std::pair<int, int>& p) const {
    return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second)<< 1);

  }
};
float heuristicBorder(std::pair<int, int> pos, int sideSize) {
  int halfSide = sideSize / 2;
  int distX = halfSide - abs(pos.first);
  int distY = halfSide - abs(pos.second);
  return std::min(distX, distY);
}
static std::vector<std::pair<int, int>> getHexNeighbors(std::pair<int, int> pos) {
  int x = pos.first;
  int y = pos.second;
  std::vector<std::pair<int, int>> neighbors;
  neighbors.push_back({x + 1, y});              // E
  neighbors.push_back({x - 1, y});              // W
  neighbors.push_back({x + (y % 2), y - 1});    // NE
  neighbors.push_back({x - 1 + (y % 2), y - 1}); // NW
  neighbors.push_back({x + (y % 2), y + 1});    // SE
  neighbors.push_back({x - 1 + (y % 2), y + 1});
  return neighbors;
}

static bool isOnBorder(std::pair<int, int> pos, int sideSize) {
  int halfSize = sideSize / 2;
  return abs(pos.first) == halfSize || abs(pos.second) == halfSize;
}

// Check if position is valid
static bool isValidPosition(std::pair<int, int> pos, int sideSize) {
  int halfSize = sideSize / 2;
  return pos.first >= -halfSize && pos.first <= halfSize &&
         pos.second >= -halfSize && pos.second <= halfSize;
}

// Convert (x, y) to world vector index
static int getIndex(std::pair<int, int> pos, int sideSize) {
  return (pos.second + sideSize / 2) * sideSize + (pos.first + sideSize / 2);
}
std::pair<int, int> Cat::move(const std::vector<bool>& world,
                               std::pair<int, int> catPos,
                               int sideSize) {

    // A* pathfinding setup
    std::unordered_map<std::pair<int, int>, std::pair<int, int>, PairHash> cameFrom;
    std::unordered_map<std::pair<int, int>, float, PairHash> gScore;
    std::unordered_set<std::pair<int, int>, PairHash> visited;

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> frontier;
    std::unordered_set<std::pair<int, int>, PairHash> frontierSet;

    // Initialize
    gScore[catPos] = 0;
    float h = heuristicBorder(catPos, sideSize);
    frontier.push({catPos, h});
    frontierSet.insert(catPos);

    std::pair<int, int> borderExit = {INT_MAX, INT_MAX};

    // A* main loop
    while (!frontier.empty()) {
        Node currentNode = frontier.top();
        std::pair<int, int> current = currentNode.pos;
        frontier.pop();
        frontierSet.erase(current);

        // Skip if already visited
        if (visited.count(current)) continue;
        visited.insert(current);

        // Check if we reached border
        if (isOnBorder(current, sideSize)) {
            borderExit = current;
            break;
        }

        // Explore neighbors
        auto neighbors = getHexNeighbors(current);
        for (const auto& neighbor : neighbors) {
            // Skip invalid positions
            if (!isValidPosition(neighbor, sideSize)) continue;

            // Skip blocked cells
            int idx = getIndex(neighbor, sideSize);
            if (world[idx]) continue;

            // Skip visited
            if (visited.count(neighbor)) continue;

            // Calculate scores
            float tentativeG = gScore[current] + 1;

            if (gScore.find(neighbor) == gScore.end() || tentativeG < gScore[neighbor]) {
                cameFrom[neighbor] = current;
                gScore[neighbor] = tentativeG;
                float h = heuristicBorder(neighbor, sideSize);
                float f = tentativeG + h;

                if (!frontierSet.count(neighbor)) {
                    frontier.push({neighbor, f});
                    frontierSet.insert(neighbor);
                }
            }
        }
    }

    // Build path from border back to cat
    if (borderExit.first != INT_MAX) {
        std::vector<std::pair<int, int>> path;
        std::pair<int, int> current = borderExit;

        while (current != catPos) {
            path.push_back(current);
            current = cameFrom[current];
        }

        // Reverse to get path from cat to border
        std::reverse(path.begin(), path.end());

        // Return first move (next step toward border)
        if (!path.empty()) {
            return path[0];
        }
    }

    // No path found - try any valid neighbor
    auto neighbors = getHexNeighbors(catPos);
    for (const auto& neighbor : neighbors) {
        if (isValidPosition(neighbor, sideSize)) {
            int idx = getIndex(neighbor, sideSize);
            if (!world[idx]) {
                return neighbor;
            }
        }
    }

    // Stuck - stay in place
    return catPos;
}

// For local testing with World class
Point2D Cat::Move(World* world) {
  // Extract data from World
  std::vector<bool> worldState;
  int sideSize = world->getWorldSideSize();

  // Build world vector (you may need to add a getter in World.h)
  for (int y = -sideSize/2; y <= sideSize/2; y++) {
    for (int x = -sideSize/2; x <= sideSize/2; x++) {
      worldState.push_back(world->getContent(x, y));
    }
  }

  Point2D catPos = world->getCat();
  std::pair<int, int> catPosPair = {catPos.x, catPos.y};

  // Call the real move function
  auto result = move(worldState, catPosPair, sideSize);

  return Point2D(result.first, result.second);
}