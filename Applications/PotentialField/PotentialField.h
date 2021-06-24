#pragma once

#include <thread>
#include <vector>
#include <unordered_map>

#include "RenderingSystem.hpp"
#include "PhysicsSystem.hpp"
#include "TagSystem.h"
#include "ECSCore/Orchestrator.hpp"

#include "PotentialFieldTypes.h"
#include "Ego.h"
#include "GoalAndObstacle.hpp"

extern lazyECS::Orchestrator gOrchestrator;

class PotentialField {

public:
    // ----------------- Member functions ----------------- //

    PotentialField(bool isFullscreen, int windowWidth, int windowHeight);

    void init();

    void main_loop();

    // ----------------- Member variables ----------------- //
    std::shared_ptr<lazyECS::RenderingSystem> renderSys; // pointer to the rendering system to be accessed from draw_contents
    std::shared_ptr<lazyECS::PhysicsSystem> physicsSys;
    std::shared_ptr<lazyECS::TagSystem> tagSys;

private:

    std::unordered_map<lazyECS::Entity, Ego> egoActors_;
    std::unordered_map<lazyECS::Entity, Obstacle> obstacleActors_;
    std::unordered_map<lazyECS::Entity, Goal> goalActors_;

};