#pragma once

#include <reactphysics3d/mathematics/mathematics_functions.h>
#include <thread>
#include <vector>
#include <unordered_map>
#include <random>
#include <chrono>

#include "RenderingSystem.hpp"
#include "PhysicsSystem.hpp"
#include "RigidBody3D.hpp"
#include "TagSystem.h"
#include "ECSCore/Orchestrator.hpp"

#include "WaveFrontTypes.h"
#include "Ego.h"
#include "GoalAndObstacle.hpp"
#include "GridCell.hpp"

extern lazyECS::Orchestrator gOrchestrator;

class WaveFront {
    
public:
    // ----------------- Member functions ----------------- //
    WaveFront(bool isFullscreen, int windowWidth, int windowHeight);

    void init();

    void main_loop();

    void SetupGridCells();

    void UpdateGridOccupancy();

    void ShowGridValues() const;

    void CalculateShortestPath();

    std::pair<uint16_t, uint16_t> GetGoalPosition();

    // ----------------- Member variables ----------------- //
    std::shared_ptr<lazyECS::RenderingSystem> renderSys; // pointer to the rendering system to be accessed from draw_contents
    std::shared_ptr<lazyECS::PhysicsSystem> physicsSys;
    std::shared_ptr<lazyECS::TagSystem> tagSys;    

private:

    std::unordered_map<lazyECS::Entity, Ego> egoActors_;
    std::unordered_map<lazyECS::Entity, Obstacle> obstacleActors_;
    std::unordered_map<lazyECS::Entity, Goal> goalActors_;

    std::vector<GridCell> gridCells_;

    float grid_size_x_, grid_size_z_, grid_resolution_;
    std::pair<float, float> grid_x_limit_, grid_z_limit_;
    uint16_t num_cells_x_, num_cells_z_;


};