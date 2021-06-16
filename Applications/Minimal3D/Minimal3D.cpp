#include "Minimal3D.h"

bool Minimal3D::lazyECS_mainloop_active{false};

Minimal3D::Minimal3D(bool isFullscreen, int windowWidth, int windowHeight)
{
    
    // --------------- lAZYECS -------------- //

    gOrchestrator.Init(); // initializes the ECS managers

    // Register the components
    gOrchestrator.RegisterComponentType<lazyECS::Transform3D>();
    gOrchestrator.RegisterComponentType<lazyECS::RigidBody3D>();
    gOrchestrator.RegisterComponentType<lazyECS::Mesh>();

    // Register the systems
    physicsSys = gOrchestrator.RegisterSystem<lazyECS::PhysicsSystem>();
    // Rendering system is registered and initialized here since it has dependency on OpenGL context to be initialized first
    // which is happening during nanogui::Screen initialization
    renderSys = gOrchestrator.RegisterSystem<lazyECS::RenderingSystem>(isFullscreen, windowWidth, windowHeight); // constructs the System

    // renderSys->SetupSignature(); // entities below will be matched with systems so signature of the system is set

    // Create the entities and assign components
    std::vector<lazyECS::Entity> entities(10);
    float boxSize[3] = {2.0, 2.0, 2.0};                   
    const int box_x_pos = 3.0;

    for (int i = 0; i < entities.size(); i++) {
        auto entity = entities.at(i);
        entity = gOrchestrator.CreateEntity();

        // Transform component (Initial position)
        reactphysics3d::Vector3 spawn_pos(box_x_pos + i*5.0, box_x_pos + i*5.0, box_x_pos + i*5.0);
        reactphysics3d::Quaternion spawn_rot(reactphysics3d::Quaternion::identity());
        lazyECS::Transform3D spawn_trans;
        spawn_trans.rp3d_transform = reactphysics3d::Transform(spawn_pos, spawn_rot);
        spawn_trans.mScalingMatrix = openglframework::Matrix4(  boxSize[0]*0.5f, 0, 0, 0,
                                                                0, boxSize[1]*0.5f, 0, 0,
                                                                0, 0, boxSize[2]*0.5f, 0,
                                                                0, 0, 0, 1);
        spawn_trans.ConvertRP3DToOpenglTransform(0.9); // temporary interpolation factor
        gOrchestrator.AddComponent<lazyECS::Transform3D>(entity,spawn_trans);


        // Mesh componenet (for rendering)
        lazyECS::Mesh mesh;
        gOrchestrator.AddComponent<lazyECS::Mesh>(entity, mesh);

        // Rigid Body component for physical motion
        lazyECS::RigidBody3D rigid_body; // will be initialized in PhysicsSystem
        gOrchestrator.AddComponent<lazyECS::RigidBody3D>(entity, rigid_body);
    }

    // Init requires the entities to be initialized and entities requires system's signature to be set
    renderSys->Init("/home/goksan/Work/lazyECS/Applications/meshes/cube.obj");
    renderSys->set_visible(true); // nanogui set visibility

    physicsSys->Init();
}

void Minimal3D::main_loop(const float& update_rate) {

    auto& nanogui_screens = renderSys->GetScreen();

    if(lazyECS_mainloop_active)
        throw std::runtime_error("Main loop is already running!");

    auto main_loop_iter = [nanogui_screens](){
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
                screen->draw_all();
                num_nanogui_screens++;
            }

            if(num_nanogui_screens == 0) {
                lazyECS_mainloop_active = false;
                return;
            }

            glfwWaitEvents();

        }
    };

    lazyECS_mainloop_active = true;

    std::thread update_thread;
    std::chrono::microseconds quantum;
    size_t quantum_count = 1;
    // We divide the update interval to approx. 50ms. chunks, since GUI elements better be drawn in this rate
    // If the  update interval is already faster than 50ms., no chunking happens
    quantum = std::chrono::microseconds((int64_t)(update_rate*1'000));
    while(quantum.count() > 50'000) {
        quantum /= 2;
        quantum_count *= 2;
    }
    // Below thread will update the rendering with at least 50ms., and at most with update_rate
    // redraw() function calls glfwPostEmptyEvent(), which allows glfwWaitEvents called above to return
    auto thread_func = [quantum, quantum_count, nanogui_screens]() {
        while(true) {
            for(size_t i = 0; i < quantum_count; i++) {
                if(!lazyECS_mainloop_active)
                    return;
                std::this_thread::sleep_for(quantum);
                for(auto screen : nanogui_screens) {
                    if(screen.second->tooltip_fade_in_progress()) // if some gui element is active
                        screen.second->redraw();    // make an additional draw call, that is additional to the fixed update rate we specified
                }
            }
            // This is the main draw callback which allows steady render rate (irrespective of mouse/keyboard callbacks)
            // after sleeping for (quantum*quantum_count=update_rate) miliseconds
            for(auto screen : nanogui_screens)
                screen.second->redraw();
        }
    };
    update_thread = std::thread(thread_func);

    try {
        while(lazyECS_mainloop_active)
            main_loop_iter();
        
        // Process events one final time
        glfwPollEvents();
    }
    catch (const std::exception& e) {
        std::cerr << "Caught exception in lazyECS main loop: " << e.what() << std::endl;
        lazyECS_mainloop_active=false;
    }

    update_thread.join();
}
