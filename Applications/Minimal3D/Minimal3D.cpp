#include "Minimal3D.h"
#include "Spawner.hpp"

bool Minimal3D::lazyECS_mainloop_active{false};
extern json launch_obj;

Minimal3D::Minimal3D(bool isFullscreen, int windowWidth, int windowHeight) {

    // --------------- lAZYECS -------------- //

    gOrchestrator.Init(); // initializes the ECS managers

    // Register the components
    gOrchestrator.RegisterComponentType<lazyECS::Transform3D>();
    gOrchestrator.RegisterComponentType<lazyECS::RigidBody3D>();
    gOrchestrator.RegisterComponentType<lazyECS::Mesh>();

    // Register the systems (calls Constructors during registration)
    physicsSys = gOrchestrator.RegisterSystem<lazyECS::PhysicsSystem>();
    renderSys = gOrchestrator.RegisterSystem<lazyECS::RenderingSystem>(isFullscreen, windowWidth, windowHeight, "Minimal 3D"); 

    renderSys->SetupSignature(); // entities below will be matched with systems so signature of the system is set
    physicsSys->SetupSignature();

    // Create the entities and assign components

    json entities_json = launch_obj.at("entities");

    lazyECS::Spawner::CreatePhysicsEntities(entities_json);
    lazyECS::Spawner::CreateTerrainEntity(entities_json);
    lazyECS::Spawner::CreateRenderOnlyEntities(entities_json);
   
    // Init requires the entities to be initialized and entities requires system's signature to be set
    renderSys->Init();
    physicsSys->Init();
}

void Minimal3D::main_loop() {

    auto& nanogui_screens = renderSys->GetScreen();
    if(lazyECS_mainloop_active)
        throw std::runtime_error("Main loop is already running!");

    auto main_loop_step = [&nanogui_screens, this]() {

        // ------------- 1) Apply ego control, NPC AI, kinematic control, etc. -------------  // 
        for(auto& entity : this->physicsSys->m_entities) {
            auto& rigidBody = gOrchestrator.GetComponent<lazyECS::RigidBody3D>(entity);
            if(rigidBody.rp3d_bodyType == rp3d::BodyType::DYNAMIC) // set force for the dynamic bodies
                rigidBody.rp3d_rigidBody->applyForceToCenterOfMass(rp3d::Vector3(1.0,0,1.0));
            else if (rigidBody.rp3d_bodyType == rp3d::BodyType::KINEMATIC) // set velocity for the kinematic body
                rigidBody.rp3d_rigidBody->setLinearVelocity(rp3d::Vector3(0.2,0.0,0.0));
        }

        // Update render only entities
        for(auto& entity : this->renderSys->m_entities) {
            if (!gOrchestrator.CheckComponentExistsInEntity<lazyECS::RigidBody3D>(entity)) { // if render only entity
                auto & trans = gOrchestrator.GetComponent<lazyECS::Transform3D>(entity);
                trans.rp3d_transform.setPosition(trans.rp3d_transform.getPosition() + rp3d::Vector3(0,0,0.02));  
            }
        }

        // ------------- 2) Update physics ------------- //    
        this->physicsSys->Update();

        // ------------- 3) Update graphics ------------- //
        for(auto& entity : this->renderSys->m_entities) {
            auto& transform = gOrchestrator.GetComponent<lazyECS::Transform3D>(entity);
            // transform.rp3d_transform.setPosition(transform.rp3d_transform.getPosition() + rp3d::Vector3(0.1,0.0,0.0)); // teleport
            transform.ConvertRP3DToOpenglTransform(); // convert the physics transform to graphics transform
        }

        int num_nanogui_screens = 0;
        for(auto screen_pair : nanogui_screens) {
            nanogui::Screen* screen = screen_pair.second;
            if(!screen->visible()) {
                continue;
            }
            else if(glfwWindowShouldClose(screen->glfw_window())) {
                screen->set_visible(false);
                continue;
            }
            else{
                screen->draw_all(); // only draws if m_redraw==True, sets m_redraw=False after iteration
                num_nanogui_screens++;
            }

            if(num_nanogui_screens == 0) {
                lazyECS_mainloop_active = false;
                return;
            }

            // Blocks next iteration until:
            // a) A keyboard/mouse event is made
            // b) render_timer_thread below periodically interrupts it by a glfwPostEmptyEvent call for steady render rate
            glfwWaitEvents();
        }
    };

    lazyECS_mainloop_active = true;

    std::thread render_timer_thread;
    std::chrono::microseconds quantum;
    quantum = std::chrono::microseconds((int64_t)(RENDER_TIME_STEP*1'000'000));
    auto render_timer_thread_func = [quantum, nanogui_screens]() {
        while(true) {
            if(!lazyECS_mainloop_active)
                return;
            else {                
                std::this_thread::sleep_for(quantum); // since no other operation happens in this thread, enough to just sleep for FPS rate
                // This is the main event interrupt which allows steady render rate (irrespective of mouse/keyboard callbacks)
                // after sleeping for RENDER_TIME_STEP
                // redraw() calls glfwPostEmptyEvent() which allows the iteration to proceed from the blocked glfwWaitEvents() state
                // and also sets m_redraw=True, for allowing draw_all to do rendering
                for(auto screen : nanogui_screens)
                    screen.second->redraw();                
            }
        }
    };
    render_timer_thread = std::thread(render_timer_thread_func);

    try {
        while(lazyECS_mainloop_active) {
            main_loop_step();
        }
        
        // Process events one final time
        glfwPollEvents();
    }
    catch (const std::exception& e) {
        std::cerr << "Caught exception in lazyECS main loop: " << e.what() << std::endl;
        lazyECS_mainloop_active=false;
    }

    render_timer_thread.join();
}
