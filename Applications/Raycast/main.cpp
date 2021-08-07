#include <chrono>
#include <iostream>
#include <fstream>

#include "Orchestrator.hpp"
#include "Raycast.h"


lazyECS::Orchestrator gOrchestrator; // Orchestrator is a global variable that can be accessed by all systems
                                     // and will exist throughout the lifetime of the program


json launch_obj; // launch_obj is a global json object that will be populated in the app after parsing the launch file
                 // and will be accessed by systems and Orchestrator to setup entity and system parameters

int main() {

    // ---- Parse the launch file for scene & entity setup ---- //
    std::ifstream ifs("scene.json");
    launch_obj = json::parse(ifs);

    // ---------- NANOGUI -------------- //
    nanogui::init();
    {
        std::shared_ptr<Raycast> raycast_app = std::make_shared<Raycast>();
        raycast_app->lazyECS_loop();
    }
    nanogui::shutdown();
    return 0;
}