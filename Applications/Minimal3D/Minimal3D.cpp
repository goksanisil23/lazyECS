#include "Minimal3D.h"

bool Minimal3D::lazyECS_mainloop_active{false};

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
    std::vector<lazyECS::Entity> entities(10);
    float boxSize[3] = {2.0, 2.0, 2.0};                   
    
    // x = lateral(right=+), y = height (up=+), z = depth(towards cam = +)
    const int box_x_pos = 0.0;
    const int box_y_pos = 40.0;
    const int box_z_pos = 0.0;

    for (int i = 0; i < entities.size(); i++) {
        auto entity = entities.at(i);
        entity = gOrchestrator.CreateEntity();

        // Transform component (Initial position)
        reactphysics3d::Vector3 spawn_pos(box_x_pos + i*5.0, box_y_pos , box_z_pos);
        reactphysics3d::Quaternion spawn_rot(reactphysics3d::Quaternion::identity());
        lazyECS::Transform3D spawn_trans(spawn_pos, spawn_rot);
        spawn_trans.SetScale(boxSize[0], boxSize[1], boxSize[2]);
        gOrchestrator.AddComponent<lazyECS::Transform3D>(entity,spawn_trans);

        // Mesh componenet (for rendering)
        lazyECS::Mesh mesh;
        mesh.meshPath = "/home/goksan/Work/lazyECS/Applications/meshes/cube.obj";
        gOrchestrator.AddComponent<lazyECS::Mesh>(entity, mesh);

        // Rigid Body component for physical motion
        lazyECS::RigidBody3D rigid_body; // will be initialized in PhysicsSystem
        rigid_body.isStatic = false;
        gOrchestrator.AddComponent<lazyECS::RigidBody3D>(entity, rigid_body);
    }

    {
    // Create entity as solid platform

        float floorSize[3] = {100.0, 2.0, 100.0};
        const int floor_x_pos = 0.0;
        const int floor_y_pos = 0.0;
        const int floor_z_pos = 0.0;

        lazyECS::Entity floor_entity(gOrchestrator.CreateEntity());
        // Transform component (Initial position)
        reactphysics3d::Vector3 floor_pos(floor_x_pos , floor_y_pos , floor_z_pos);
        reactphysics3d::Quaternion floor_rot(reactphysics3d::Quaternion::identity());
        lazyECS::Transform3D floor_trans(floor_pos, floor_rot);
        floor_trans.SetScale(floorSize[0], floorSize[1], floorSize[2]);
        gOrchestrator.AddComponent<lazyECS::Transform3D>(floor_entity,floor_trans);

        // Mesh componenet (for rendering)
        lazyECS::Mesh floor_mesh;
        floor_mesh.meshPath = "/home/goksan/Work/lazyECS/Applications/meshes/cube.obj"; 
        gOrchestrator.AddComponent<lazyECS::Mesh>(floor_entity, floor_mesh);

        // Rigid Body component for physical motion
        lazyECS::RigidBody3D rigid_body; // will be initialized in PhysicsSystem
        rigid_body.isStatic = true;
        gOrchestrator.AddComponent<lazyECS::RigidBody3D>(floor_entity, rigid_body);    

    // end of floor creation
    }

    // Init requires the entities to be initialized and entities requires system's signature to be set
    renderSys->Init();
    physicsSys->Init();
}

void Minimal3D::main_loop(const float& render_rate_sec) {

    auto& nanogui_screens = renderSys->GetScreen();

    if(lazyECS_mainloop_active)
        throw std::runtime_error("Main loop is already running!");

    auto main_loop_step = [&nanogui_screens, this]() {

        // Update physics
        this->physicsSys->Update();
        
        // Update graphics
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
    quantum = std::chrono::microseconds((int64_t)(render_rate_sec*1'000'000));
    auto render_timer_thread_func = [quantum, nanogui_screens]() {
        while(true) {
            if(!lazyECS_mainloop_active)
                return;
            else {                
                std::this_thread::sleep_for(quantum); // since no other operation happens in this thread, enough to just sleep for FPS rate
                // This is the main event interrupt which allows steady render rate (irrespective of mouse/keyboard callbacks)
                // after sleeping for render_rate_sec
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
