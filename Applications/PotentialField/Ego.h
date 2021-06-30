#pragma once

#include "ECSCore/Types.hpp"
#include "Components/Tag.h"
#include "PotentialFieldTypes.h"
#include "GoalAndObstacle.hpp"

#include <vector>
#include <unordered_map>

class Ego {

public:
    explicit Ego(const float& scan_radius = 1.0F, const int& num_directions = 6, 
                 const p_field::Position& position = p_field::Position());

    void CalculatePossibleMoves();
    std::pair<p_field::Position, float> ComputeBestMove(const std::unordered_map<lazyECS::Entity, Obstacle>& obstacleActors, 
                                       const  std::unordered_map<lazyECS::Entity,Goal>& goalActors);

    void UpdateGoalReached(const std::unordered_map<lazyECS::Entity,Goal>& goalActors);

    // lazyECS::Tag mTag; // which Tag group is this Actor a part of
    p_field::Position position_; // simple 2D position of the robot
    bool goalReached_; // if one of the goals have been reached

private:
    float scanRadius_; // radius of the search space (circle) where robot calculates possible costs
    int numPossibleDirections_; // defines the resolution of the search space (circle)
    std::vector<p_field::Position> possibleMoves_; // possible future points in the search radius
    float headingIncrement_;
};