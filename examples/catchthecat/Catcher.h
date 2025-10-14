#ifndef CATCHER_H
#define CATCHER_H

#include "Agent.h"
#include "IAgent.h"
class Catcher : public Agent, public IAgent {
public:
  explicit Catcher() : Agent(){};
  Point2D Move(World*) override;
  virtual std::pair<int,int> move(const std::vector<bool>& world, std::pair<int,int> catPos, int sideSize ) override;
};

#endif  // CATCHER_H