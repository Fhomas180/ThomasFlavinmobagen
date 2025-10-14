#include "Catcher.h"
#include "World.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include <vector>
struct Node {
  std::pair<int, int> pos;
  float fScore;

  bool operator>(const Node& other) const {
    return fScore > other.fScore;
  }
};

struct PairHash {
  size_t operator()(const std::pair<int, int>& p) const {
    return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
  }
};

static float heuristicToBorder(std::pair<int, int> pos, int sideSize) {
  int halfSize = sideSize / 2;
  int distX = halfSize - abs(pos.first);
  int distY = halfSize - abs(pos.second);
  return std::min(distX, distY);
}

static std::vector<std::pair<int, int>> getHexNeighbors(std::pair<int, int> pos) {
  int x = pos.first;
  int y = pos.second;
  std::vector<std::pair<int, int>> neighbors;

  neighbors.push_back({x + 1, y});
  neighbors.push_back({x - 1, y});
  neighbors.push_back({x + (y % 2), y - 1});
  neighbors.push_back({x - 1 + (y % 2), y - 1});
  neighbors.push_back({x + (y % 2), y + 1});
  neighbors.push_back({x - 1 + (y % 2), y + 1});

  return neighbors;
}
static bool isOnBorder(std::pair<int, int> pos, int sideSize) {
  int halfSize = sideSize / 2;
  return abs(pos.first) == halfSize || abs(pos.second) == halfSize;
}

static bool isValidPosition(std::pair<int, int> pos, int sideSize) {
  int halfSize = sideSize / 2;
  return pos.first >= -halfSize && pos.first <= halfSize &&
         pos.second >= -halfSize && pos.second <= halfSize;
}

static int getIndex(std::pair<int, int> pos, int sideSize) {
  return (pos.second + sideSize / 2) * sideSize + (pos.first + sideSize / 2);
}
std::vector<std::pair<int, int>> findCatPath(const std::vector<bool>& world,std::pair<int, int> catPos,int sideSize) {
    std::unordered_map<std::pair<int, int>, std::pair<int, int>, PairHash> cameFrom;
    std::unordered_map<std::pair<int, int>, float, PairHash> gScore;
    std::unordered_set<std::pair<int, int>, PairHash> visited;

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> frontier;
    std::unordered_set<std::pair<int, int>, PairHash> frontierSet;

    gScore[catPos] = 0;
    float h = heuristicToBorder(catPos, sideSize);
    frontier.push({catPos, h});
    frontierSet.insert(catPos);

    std::pair<int, int> borderExit = {INT_MAX, INT_MAX};

    while (!frontier.empty()) {
        Node currentNode = frontier.top();
        std::pair<int, int> current = currentNode.pos;
        frontier.pop();
        frontierSet.erase(current);

        if (visited.count(current)) continue;
        visited.insert(current);

        if (isOnBorder(current, sideSize)) {
            borderExit = current;
            break;
        }

        auto neighbors = getHexNeighbors(current);
        for (const auto& neighbor : neighbors) {
            if (!isValidPosition(neighbor, sideSize)) continue;

            int idx = getIndex(neighbor, sideSize);
            if (world[idx]) continue;

            if (visited.count(neighbor)) continue;

            float tentativeG = gScore[current] + 1;

            if (gScore.find(neighbor) == gScore.end() || tentativeG < gScore[neighbor]) {
                cameFrom[neighbor] = current;
                gScore[neighbor] = tentativeG;
                float h = heuristicToBorder(neighbor, sideSize);
                float f = tentativeG + h;

                if (!frontierSet.count(neighbor)) {
                    frontier.push({neighbor, f});
                    frontierSet.insert(neighbor);
                }
            }
        }
    }

    // Build path
    std::vector<std::pair<int, int>> path;
    if (borderExit.first != INT_MAX) {
        std::pair<int, int> current = borderExit;
        while (current != catPos) {
            path.push_back(current);
            current = cameFrom[current];
        }
        std::reverse(path.begin(), path.end());
    }

    return path;
}

// MAIN FUNCTION - Catcher blocking strategy
std::pair<int, int> Catcher::move(const std::vector<bool>& world,
                                   std::pair<int, int> catPos,
                                   int sideSize) {

    // Strategy: Find the cat's shortest path and block it
    auto catPath = findCatPath(world, catPos, sideSize);

    // If cat has a path, block the first move (closest to cat)
    if (!catPath.empty()) {
        // Block the cat's next intended move
        return catPath[0];
    }

    // If no path found, cat might already be trapped
    // Or block a random valid neighbor of the cat
    auto neighbors = getHexNeighbors(catPos);
    for (const auto& neighbor : neighbors) {
        if (isValidPosition(neighbor, sideSize)) {
            int idx = getIndex(neighbor, sideSize);
            if (!world[idx]) {
                return neighbor;
            }
        }
    }

    // Last resort: block any valid empty cell
    int halfSize = sideSize / 2;
    for (int y = -halfSize; y <= halfSize; y++) {
        for (int x = -halfSize; x <= halfSize; x++) {
            std::pair<int, int> pos = {x, y};
            if (pos != catPos && isValidPosition(pos, sideSize)) {
                int idx = getIndex(pos, sideSize);
                if (!world[idx]) {
                    return pos;
                }
            }
        }
    }

    // Should never reach here - return cat position as invalid move
    return catPos;
}

// For local testing with World class
Point2D Catcher::Move(World* world) {
    // Extract data from World
    std::vector<bool> worldState;
    int sideSize = world->getWorldSideSize();

    // Build world vector
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