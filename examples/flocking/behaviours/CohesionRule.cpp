#include "CohesionRule.h"
#include "../gameobjects/Boid.h"

Vector2f CohesionRule::computeForce(const std::vector<Boid*>& neighborhood, Boid* boid) {
  Vector2f cohesionForce;

  // todo: add your code here to make a force towards the center of mass
  // hint: iterate over the neighborhood
  if (!neighborhood.empty()) {
    Vector2f centerOfMass;

    // find center of mass
    for (int i = 0; i < neighborhood.size(); i++) {
      Boid* neighbor = neighborhood[i];
      centerOfMass += neighbor->transform.position;
    }

    centerOfMass /= (float)neighborhood.size();

    // force is direction towards the center of mass
    cohesionForce = centerOfMass - boid->transform.position;
    cohesionForce = Vector2f::normalized(cohesionForce);
  }
  // find center of mass

  return cohesionForce;
}