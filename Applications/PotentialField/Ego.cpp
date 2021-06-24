#include "Ego.h"

#include <cmath>
#include <reactphysics3d/reactphysics3d.h>
#include <limits>
#include <vector>

Ego::Ego(const float& scan_radius, const int& num_directions, const p_field::Position& position) : 
         scanRadius_(scan_radius), numPossibleDirections_(num_directions), position_(position) {}

void Ego::CalculatePossibleMoves() {
    float heading_increment = (2.0F * rp3d::PI) / static_cast<float>(this->numPossibleDirections_);
    float heading(0); // starting angle

    possibleMoves_.clear();

    for(int i = 0; i < numPossibleDirections_; i++) {
        possibleMoves_.emplace_back(p_field::Position(scanRadius_ * std::cos(heading) + position_.x,
                                                      scanRadius_ * std::sin(heading) + position_.y));
        heading += heading_increment;
    }
}

// Repulsion = Obstacle is (+) and Attraction = Goal is (-)
p_field::Position Ego::ComputeBestMove(const std::unordered_map<lazyECS::Entity, Obstacle>& obstacleActors, 
                                       const  std::unordered_map<lazyECS::Entity,Goal>& goalActors) {
    float min_action_value = std::numeric_limits<float>::infinity();
    p_field::Position best_move = possibleMoves_.at(0); // arbitrary initialization of best move

    // Find the position in the search circle with the least resultant force (meaning pulled the most by the Goals)
    for(const auto& possible_move : possibleMoves_) {
        float action_value = 0.0F;
        // Add all the forces at this ego position, based on all the surrounding Goals and Obstacles
        for(const auto& goal_actor : goalActors) {
            action_value += goal_actor.second.GetAttractionForce(possible_move);
            for(const auto& obst_actor : obstacleActors) {
                action_value += obst_actor.second.GetRepulsionForce(possible_move);
            }
        }

        // Now evaluate how good is it to be in this position
        if(action_value < min_action_value) {
            min_action_value = action_value;
            best_move = possible_move;
        }           
    }

    return best_move;
}