#ifndef CAT_H
#define CAT_H

#include "Agent.h"
#include "IAgent.h"

class Cat : public Agent, public IAgent {
public:
  explicit Cat() : Agent(){};
  Point2D Move(World*) override;
  virtual std::pair<int,int> move(const std::vector<bool>& world, std::pair<int,int> catPos, int sideSize ) override;
};

#endif  // CAT_H