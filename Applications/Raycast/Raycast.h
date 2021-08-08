#pragma once

#include <random>

#include "BaseApp.h"
#include "Ego.h"

extern lazyECS::Orchestrator gOrchestrator;

class Raycast : public BaseApp {
    
public:
    // ----------------- Member functions ----------------- //
    Raycast();

    void init();

    void app_func() override;

private:

    std::random_device rand_dev_;
    std::default_random_engine rand_eng_;
    std::uniform_int_distribution<int> rand_dist_;

    std::unordered_map<lazyECS::Entity, Ego> egoActors_;

};