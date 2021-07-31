#include <chrono>
#include <iostream>
#include <fstream>

#include "Orchestrator.hpp"
#include "WaveFront.h"


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
        nanogui::ref<WaveFront> wavefront_app = new WaveFront(false, 1200, 1200); // WaveFront(isFullscreen, windowWidth, windowHeight)
        wavefront_app->main_loop(); // render update interval in sec.
    }
    nanogui::shutdown();
    return 0;
}