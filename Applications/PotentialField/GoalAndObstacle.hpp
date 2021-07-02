#pragma once

#include "PotentialFieldTypes.h"
#include "lazyECS/ECSCore/Types.hpp"

#include <cmath>
#include <reactphysics3d/reactphysics3d.h>

class Obstacle {

public:
    explicit Obstacle(const float& sigma = 1, const p_field::Position& position = p_field::Position()) 
        : position_(position), sigma_(sigma) {}

    // Calculate the repulsion force caused by this Obstacle, at the given targetPos
    // Repulsion Force is calculated based on Normal Distribution, where the mean of the distribution is the position of the obstacle
    float GetRepulsionForce(const p_field::Position& targetPos) const {
        float repulsion_force = (1.0F / (sigma_ * std::sqrt(2*rp3d::PI))) 
                                        * std::exp(-0.5 * position_.CalculateDistanceSquared(targetPos) / (sigma_*sigma_));

        return repulsion_force;
    }

    p_field::Position position_; // simple 2D position of the Obstacle -> also mean (mu) of the Normal distribution

private:

    float sigma_; // standard deviation of Normal distribution
};

class Goal {

public:
    explicit Goal(const float& sigma = 1, const p_field::Position& position = p_field::Position()) 
        : position_(position), sigma_(sigma) {}

    // Calculate the attraction force caused by this Goal, at the given targetPos
    // Attraction Force is calculated based on Normal Distribution, where the mean of the distribution is the position of the Goal
    float GetAttractionForce(const p_field::Position& targetPos) const {
        float attraction_force = -(1.0F / (sigma_ * std::sqrt(2*rp3d::PI))) 
                                        * std::exp(-0.5 * position_.CalculateDistanceSquared(targetPos) / (sigma_*sigma_)); // just opposite of repulsion

        return attraction_force;
    }

    p_field::Position position_; // simple 2D position of the Obstacle -> also mean (mu) of the Normal distribution
    
private:

    float sigma_; // standard deviation of Normal distribution

};