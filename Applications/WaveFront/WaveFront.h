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

    void CreateGridCells();

    void UpdateGridOccupancy();

    void ResetGridStatus();

    void ResetActorPositions();

    void HighlightPath(const std::deque<uint16_t>& ego_path);

    void ShowGridValues() const;

    void RunWavePropagation();

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

    // Application parameters
    float grid_size_x_, grid_size_z_, grid_resolution_;
    std::pair<float, float> grid_x_limit_, grid_z_limit_;
    uint16_t num_cells_x_, num_cells_z_;
    float app_step_time_;
    
    bool goalsReached_;
    std::random_device rand_dev_;
    std::default_random_engine rand_eng_;
    std::uniform_int_distribution<int> rand_dist_;

    std::chrono::_V2::system_clock::time_point prevAppTime_;
    float deltaTime_;
    float timeAccumulator_;     


};