#include "Minimal3D.h"
#include "Spawner.hpp"

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

    // entities below will be matched with systems so signature of the system is set
    renderSys->SetupSignature(); 
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

    auto main_loop_step = [this]() {

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
        this->renderSys->Update();

    };

    // start the render timer thread to trigger render pipeline periodically
    std::thread render_timer_thread(&lazyECS::RenderingSystem::TimerThreadFunc, this->renderSys);

    try {
        while(true) {
            main_loop_step();
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Caught exception in lazyECS main loop: " << e.what() << std::endl;
    }

    render_timer_thread.join();
}