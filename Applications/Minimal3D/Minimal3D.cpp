#include "Minimal3D.h"

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

    // ------------ Dynamic entities ------------ //
    json entities_json = launch_obj.at("entities");
    std::vector<lazyECS::Entity> entities(static_cast<unsigned int>(entities_json.at("amount")));

    // x = lateral(right=+), y = height (up=+), z = depth(towards cam = +)
    float boxSize[3] = {static_cast<float>(entities_json.at("size")), static_cast<float>(entities_json.at("size")), static_cast<float>(entities_json.at("size"))};                  
    const float box_x_pos = static_cast<float>(entities_json.at("amount")) / 2.0 * -1.0;
    const float box_y_pos = static_cast<float>(entities_json.at("initial_height"));
    const float box_z_pos = 0.0;

    for (int i = 0; i < entities.size(); i++) {
        auto entity = entities.at(i);
        entity = gOrchestrator.CreateEntity();

        // Transform component (Initial position)
        reactphysics3d::Vector3 spawn_pos(0, 5+i*1.0 , box_z_pos);
        reactphysics3d::Quaternion spawn_rot(reactphysics3d::Quaternion::identity());
        lazyECS::Transform3D spawn_trans(spawn_pos, spawn_rot);
        spawn_trans.SetScale(boxSize[0], boxSize[1], boxSize[2]);
        gOrchestrator.AddComponent<lazyECS::Transform3D>(entity,spawn_trans);

        // Mesh componenet (for rendering)
        if(i < static_cast<int>(entities_json.at("amount"))/2) {
            lazyECS::Mesh mesh(lazyECS::Shape::Sphere);
            mesh.mColor = openglframework::Color(0.0, 0.0, 1.0, 1.0);
            gOrchestrator.AddComponent<lazyECS::Mesh>(entity, mesh);

        }
        else {
            lazyECS::Mesh mesh(lazyECS::Shape::Box);
            mesh.mColor = openglframework::Color(1.0, 0.0, 0.0, 1.0);
            gOrchestrator.AddComponent<lazyECS::Mesh>(entity, mesh);
        }   

        // Rigid Body component for physical motion
        lazyECS::RigidBody3D rigid_body; // will be initialized in PhysicsSystem
        rigid_body.rp3d_bodyType = rp3d::BodyType::DYNAMIC;
        gOrchestrator.AddComponent<lazyECS::RigidBody3D>(entity, rigid_body);
    }

    // ------------ Static entities ------------ //
    {
        // Create entity as solid platform
        json floor_scale = launch_obj.at("floor").at("scale");
        float floorSize[3] = {floor_scale.at("x"), floor_scale.at("y"), floor_scale.at("z")}; // TODO: Keep concave mesh scaling below 2. above 2 there is some scaling bug !
        const float floor_x_pos = 0.0;
        const float floor_y_pos = 0.0;
        const float floor_z_pos = 0.0;     
     
        lazyECS::Entity floor_entity(gOrchestrator.CreateEntity());
        
        // Mesh component (for rendering)
        std::shared_ptr<lazyECS::Mesh> floor_mesh;
        std::string floor_type = launch_obj.at("floor").at("type");
        if(floor_type == "heightfield")
            floor_mesh = std::make_shared<lazyECS::Mesh>(lazyECS::Shape::Hfield);
        else if (floor_type == "concavemesh")
            floor_mesh = std::make_shared<lazyECS::Mesh>(lazyECS::Shape::ConcaveMesh);
        else if (floor_type == "flat")
            floor_mesh = std::make_shared<lazyECS::Mesh>(lazyECS::Shape::Box);
        else
            std::runtime_error("No such floor type!");
        floor_mesh->mColor = openglframework::Color(0.47f, 0.48f, 0.49f, 1.0f);
        gOrchestrator.AddComponent<lazyECS::Mesh>(floor_entity, *floor_mesh);

        // Transform component (Initial position)
        reactphysics3d::Vector3 floor_pos(floor_x_pos , floor_y_pos , floor_z_pos);
        reactphysics3d::Quaternion floor_rot(reactphysics3d::Quaternion::identity());
        lazyECS::Transform3D floor_trans(floor_pos, floor_rot);
        if(floor_mesh->mShape != lazyECS::Shape::Hfield)
            floor_trans.SetScale(floorSize[0], floorSize[1], floorSize[2]);
        gOrchestrator.AddComponent<lazyECS::Transform3D>(floor_entity,floor_trans);        

        // Rigid Body component for physical motion
        lazyECS::RigidBody3D floor_rigid_body; // will be initialized in PhysicsSystem
        floor_rigid_body.rp3d_bodyType = rp3d::BodyType::STATIC;
        gOrchestrator.AddComponent<lazyECS::RigidBody3D>(floor_entity, floor_rigid_body);    
    }

    // ------------ Render-only entities ------------ //    
    {
        const rp3d::Vector3 marker_position{3.0,0.0,3.0};
        std::vector<lazyECS::Entity> marker_entities(10);
        for(int i = 0; i < marker_entities.size(); i++) {
            auto marker_ent = marker_entities.at(i);
            marker_ent = gOrchestrator.CreateEntity();

            // Transform component
            reactphysics3d::Vector3 marker_pos = marker_position + rp3d::Vector3(i,0,i);
            reactphysics3d::Quaternion marker_rot(reactphysics3d::Quaternion::identity());
            lazyECS::Transform3D marker_trans(marker_pos, marker_rot);
            marker_trans.SetScale(0.1,0.1,0.1);
            gOrchestrator.AddComponent<lazyECS::Transform3D>(marker_ent, marker_trans);

            // Mesh componenet (for rendering)
            lazyECS::Mesh marker_mesh(lazyECS::Shape::Sphere);
            marker_mesh.mColor = openglframework::Color(0.0, 1.0, 0.0, 1.0);
            gOrchestrator.AddComponent<lazyECS::Mesh>(marker_ent, marker_mesh);
        }
    }

    // ------------ Kinematic entities ------------ //
    lazyECS::Entity targetEntity;
    targetEntity = gOrchestrator.CreateEntity();

    // Transform component (Initial position)
    reactphysics3d::Vector3 target_pos(5, 0, -5);
    reactphysics3d::Quaternion target_rot(reactphysics3d::Quaternion::identity());
    lazyECS::Transform3D target_trans(target_pos, target_rot);
    target_trans.SetScale(1.0, 2.0, 1.0);
    gOrchestrator.AddComponent<lazyECS::Transform3D>(targetEntity,target_trans);

    // Mesh componenet (for rendering)
    lazyECS::Mesh targetMesh(lazyECS::Shape::Capsule);
    targetMesh.mColor = openglframework::Color(0.6, 0.7, 0.7, 1.0);
    gOrchestrator.AddComponent<lazyECS::Mesh>(targetEntity, targetMesh);

    // Rigid Body component for physical motion
    lazyECS::RigidBody3D target_rigid_body; // will be initialized in PhysicsSystem
    target_rigid_body.rp3d_bodyType = rp3d::BodyType::KINEMATIC;
    gOrchestrator.AddComponent<lazyECS::RigidBody3D>(targetEntity, target_rigid_body);     

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
