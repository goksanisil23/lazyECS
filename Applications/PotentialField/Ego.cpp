#include "Ego.h"
#include "PotentialFieldTypes.h"

#include <cmath>
#include <reactphysics3d/reactphysics3d.h>
#include <limits>
#include <utility>
#include <vector>

Ego::Ego(const float& scan_radius, const int& num_directions, const p_field::Position& position) : 
         scanRadius_(scan_radius), numPossibleDirections_(num_directions), position_(position), goalReached_(false), headingIncrement_(0.0) {}

void Ego::CalculatePossibleMoves() {
    this->headingIncrement_ = (2.0F * rp3d::PI) / static_cast<float>(this->numPossibleDirections_);
    float heading(0); // starting angle -> we measure heading from z axis, incrementing CCW (since y is pointing up)
                     // z-axis = heading 0, x-axis = heading 90

    possibleMoves_.clear();

    for(int i = 0; i < numPossibleDirections_; i++) {
        possibleMoves_.emplace_back(p_field::Position(scanRadius_ * std::sin(heading) + position_.x,   // lateral
                                                      scanRadius_ * std::cos(heading) + position_.z)); // longitudinal
        heading += headingIncrement_;
    }
}

void Ego::UpdateGoalReached(const std::unordered_map<lazyECS::Entity,Goal>& goalActors) {
    for(const auto& goal_actor : goalActors) {
        if(goal_actor.second.position_.CalculateDistance(this->position_) < 0.2) {
            goalReached_ = true;
            break;
        }
    }
}

// Repulsion = Obstacle is (+) and Attraction = Goal is (-)
std::pair<p_field::Position, float> Ego::ComputeBestMove(const std::unordered_map<lazyECS::Entity, Obstacle>& obstacleActors, 
                                       const std::unordered_map<lazyECS::Entity,Goal>& goalActors) {
                                           
    // calculate possible moves with the current position information
    CalculatePossibleMoves();

    float min_action_value = std::numeric_limits<float>::infinity();
    p_field::Position best_move = possibleMoves_.at(0); // arbitrary initialization of best move

    UpdateGoalReached(goalActors);

    int best_move_idx = 0;

    if(!goalReached_) {
        // Find the position in the search circle with the least resultant force (meaning pulled the most by the Goals)
        // for(const auto& possible_move : possibleMoves_) {
        for(int i = 0; i < possibleMoves_.size(); i++) {
            float action_value = 0.0F;
            // Add all the forces at this ego position, based on all the surrounding Goals and Obstacles
            for(const auto& goal_actor : goalActors) {
                action_value += goal_actor.second.GetAttractionForce(possibleMoves_.at(i));
            }
            for(const auto& obst_actor : obstacleActors) {
                action_value += obst_actor.second.GetRepulsionForce(possibleMoves_.at(i));
            }        

            // Now evaluate how good is it to be in this position
            if((action_value < min_action_value)) {
                min_action_value = action_value;
                best_move = possibleMoves_.at(i);
                best_move_idx = i;
            }           
        }
    }
    else {
        best_move = position_; // keep the current position
    }

    return std::make_pair(best_move, headingIncrement_ * static_cast<float>(best_move_idx));
}