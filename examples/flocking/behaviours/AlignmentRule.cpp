#include "AlignmentRule.h"
#include "../gameobjects/Boid.h"

Vector2f AlignmentRule::computeForce(const std::vector<Boid*>& neighborhood, Boid* boid) {
  // Try to match the heading of neighbors = Average velocity
  Vector2f averageVelocity = Vector2f::zero();

  // todo: add your code here to align each boid in a neighborhood
  // hint: iterate over the neighborhood
//iterating the neighborhood by using a for loop to get the size of neighbors
for(int i = 0; i < neighborhood.size(); i++) {
  //Using boid to call neighbor to which equals to neighborhood to get the array of the size.
Boid* neighbor = neighborhood[i];
  Vector2f force = Vector2f::zero();
  //While getting the averageVelocity I add it to neighbor to get the size then use the arrow operator to get the Velocity of all neighbors.
  averageVelocity += neighbor->getVelocity();

}
  return Vector2f::normalized(averageVelocity);
}