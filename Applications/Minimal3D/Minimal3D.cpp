#include "Minimal3D.h"

bool Minimal3D::lazyECS_mainloop_active{false};

Minimal3D::Minimal3D(bool isFullscreen, int windowWidth, int windowHeight) :
    prevFrameTime{std::chrono::high_resolution_clock::now()}, currentFrameTime{std::chrono::high_resolution_clock::now()}, 
    deltaTime{0.0f}, sleep_duration{0}
{
    
    // --------------- lAZYECS -------------- //

    gOrchestrator.Init(); // initializes the ECS managers

    // Register the components
    gOrchestrator.RegisterComponentType<lazyECS::Transform3D>();
    gOrchestrator.RegisterComponentType<lazyECS::RigidBody3D>();
    gOrchestrator.RegisterComponentType<lazyECS::Mesh>();

    // Register the systems
    physicsSys = gOrchestrator.RegisterSystem<lazyECS::PhysicsSystem>();
    renderSys = gOrchestrator.RegisterSystem<lazyECS::RenderingSystem>(isFullscreen, windowWidth, windowHeight); // constructs the System

    renderSys->SetupSignature(); // entities below will be matched with systems so signature of the system is set

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
        spawn_trans.rp3d_prev_transform = spawn_trans.rp3d_transform;
        spawn_trans.mScalingMatrix = openglframework::Matrix4(  boxSize[0]*0.5f, 0, 0, 0,
                                                                0, boxSize[1]*0.5f, 0, 0,
                                                                0, 0, boxSize[2]*0.5f, 0,
                                                                0, 0, 0, 1);
        spawn_trans.ConvertRP3DToOpenglTransform(); // temporary interpolation factor
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

void Minimal3D::main_loop(const float& update_rate_sec) {

    auto& nanogui_screens = renderSys->GetScreen();

    if(lazyECS_mainloop_active)
        throw std::runtime_error("Main loop is already running!");

    auto main_loop_step = [&nanogui_screens, this](){

        // Update physics
        this->physicsSys->Update();
        
        // Update graphics
        for(auto& entity : this->renderSys->m_entities) {
            auto& transform = gOrchestrator.GetComponent<lazyECS::Transform3D>(entity);
            // transform.rp3d_transform.setPosition(transform.rp3d_transform.getPosition() + rp3d::Vector3(0.1,0.0,0.0));
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
                screen->draw_all();
                num_nanogui_screens++;
            }

            if(num_nanogui_screens == 0) {
                lazyECS_mainloop_active = false;
                return;
            }

            // Blocks next iteration until:
            // a) A keyboard/mouse event is made
            // b) update_thread below periodically interrupts it by a glfwPostEmptyEvent call for steady render rate
            glfwWaitEvents();         

        }
    };

    lazyECS_mainloop_active = true;

    std::thread update_thread;
    std::chrono::microseconds quantum;
    quantum = std::chrono::microseconds((int64_t)(update_rate_sec*1'000'000));
    // auto& _sleep_duration = this->sleep_duration;
    auto _sleep_duration = std::chrono::duration<float>(update_rate_sec);
    auto thread_func = [quantum, nanogui_screens, &_sleep_duration]() {
        while(true) {
            if(!lazyECS_mainloop_active)
                return;
            else {                
                std::this_thread::sleep_for(_sleep_duration);
                // This is the main draw interrupt which allows steady render rate (irrespective of mouse/keyboard callbacks)
                // after sleeping for update_rate_sec
                // redraw() calls glfwPostEmptyEvent() which allows the iteration to proceed from the blocked glfwWaitEvents() state
                for(auto screen : nanogui_screens)
                    screen.second->redraw();                
            }
        }
    };    
    update_thread = std::thread(thread_func);

    try {
        while(lazyECS_mainloop_active) {
            // this->currentFrameTime = std::chrono::high_resolution_clock::now();
            // this->deltaTime = std::chrono::duration<float, std::chrono::seconds::period>(currentFrameTime-prevFrameTime).count();
            // this->prevFrameTime = currentFrameTime; // update previous time
            // if(deltaTime < update_rate_sec) { // we have time to sleep
            //     sleep_duration = std::chrono::duration<float>(update_rate_sec-deltaTime);
            // }
            // else {
            //     sleep_duration = std::chrono::duration<float>(0);
            // }
            main_loop_step();
        }
        
        // Process events one final time
        glfwPollEvents();
    }
    catch (const std::exception& e) {
        std::cerr << "Caught exception in lazyECS main loop: " << e.what() << std::endl;
        lazyECS_mainloop_active=false;
    }

    update_thread.join();
}
