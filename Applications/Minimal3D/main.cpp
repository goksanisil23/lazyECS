#include "Orchestrator.hpp"

#include <chrono>
#include <iostream>

#include "Minimal3D.h"


lazyECS::Orchestrator gOrchestrator; // Orchestrator is a global variable that can be accessed by all systems
                                     // and will exist throughout the lifetime of the program


int main() {

    // ---------- NANOGUI -------------- //
    nanogui::init();

    {
        nanogui::ref<Minimal3D> minimal3d_app = new Minimal3D(false, 800, 600); // Minimal3D(isFullscreen, windowWidth, windowHeight)
        // minimal3d_app->set_visible(true);
        
        nanogui::mainloop(); // calls nanogui::Screen::draw_all --> 
                                                                    // draw_setup();
                                                                    // draw_contents();
                                                                    // draw_widgets();
                                                                    // draw_teardown();

    }

    nanogui::shutdown();


    return 0;
}