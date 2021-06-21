#include "Orchestrator.hpp"

#include <chrono>
#include <iostream>
#include <fstream>

#include "Minimal3D.h"


lazyECS::Orchestrator gOrchestrator; // Orchestrator is a global variable that can be accessed by all systems
                                     // and will exist throughout the lifetime of the program


json launch_obj; // launch_obj is a global json object that will be populated in the app after parsing the launch file
                 // and will be accessed by systems and Orchestrator to setup entity and system parameters

int main() {

    // ---- Parse the launch file for scene & entity setup ---- //
    std::ifstream ifs("/home/goksan/Work/lazyECS/Applications/Minimal3D/launch.json");
    launch_obj = json::parse(ifs);

    // ---------- NANOGUI -------------- //
    nanogui::init();

    {
        nanogui::ref<Minimal3D> minimal3d_app = new Minimal3D(false, 1200, 1200); // Minimal3D(isFullscreen, windowWidth, windowHeight)
        // minimal3d_app->set_visible(true);
        
        // nanogui::mainloop(10); // calls nanogui::Screen::draw_all --> 
                                                                    // draw_setup();
                                                                    // draw_contents();
                                                                    // draw_widgets();
                                                                    // draw_teardown();

        minimal3d_app->main_loop(1.0f/60.0f); // render update interval in sec.

    }

    nanogui::shutdown();


    return 0;
}