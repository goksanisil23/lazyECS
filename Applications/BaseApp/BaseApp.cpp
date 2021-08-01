#include "BaseApp.h"

#include "RigidBody3D.hpp"
#include "Transform3D.hpp"

#include "Spawner.hpp"

#include <reactphysics3d/configuration.h>
#include <reactphysics3d/mathematics/Quaternion.h>
#include <reactphysics3d/mathematics/Ray.h>
#include <reactphysics3d/mathematics/Transform.h>
#include <reactphysics3d/mathematics/Vector3.h>
#include <reactphysics3d/utils/DebugRenderer.h>


extern json launch_obj;

BaseApp::BaseApp(bool isFullscreen, int windowWidth, int windowHeight) :
    prevAppTime_{std::chrono::high_resolution_clock::now()},  deltaTime_(0.0),timeAccumulator_(0.0)
{
    // --------------- lAZYECS -------------- //
    gOrchestrator.Init(); // initializes the ECS managers

    // Register the components (all available components in lazyECS)
    gOrchestrator.RegisterComponentType<lazyECS::Transform3D>();
    gOrchestrator.RegisterComponentType<lazyECS::RigidBody3D>();
    gOrchestrator.RegisterComponentType<lazyECS::Mesh>();
    gOrchestrator.RegisterComponentType<lazyECS::Tag>();

    // Register the all available systems in lazyECS (calls Constructors during registration)
    physicsSys = gOrchestrator.RegisterSystem<lazyECS::PhysicsSystem>();
    renderSys = gOrchestrator.RegisterSystem<lazyECS::RenderingSystem>(isFullscreen, windowWidth, windowHeight, "Potential Field");
    tagSys = gOrchestrator.RegisterSystem<lazyECS::TagSystem>();
    manControlSys = gOrchestrator.RegisterSystem<lazyECS::ManualControlSystem>();

    // entities below will be matched with systems so signature of the system is set first
    renderSys->SetupSignature(); 
    physicsSys->SetupSignature();
    tagSys->SetupSignature();
    manControlSys->SetupSignature();

    // Create the entities and assign components (parsing the json file)
    json entities_json = launch_obj.at("entities");
    lazyECS::Spawner::CreateEntities(entities_json);
   
    // Init requires the entities to be initialized and entities requires system's signature to be set
    renderSys->Init();
    physicsSys->Init();
    manControlSys->Init("ego");

    // --------------- Application -------------- //
    app_step_time_ = launch_obj.at("application").at("APP_STEP_TIME");
}

void BaseApp::main_lazyECS_loop() {

    auto loop_step = [this]() {

        // Adjust the iteration rate for the Application (not the engine)
        auto current_app_time = std::chrono::high_resolution_clock::now();
        deltaTime_ = std::chrono::duration<float, std::chrono::seconds::period>(current_app_time-prevAppTime_).count();
        prevAppTime_ = current_app_time; // update previous time
        timeAccumulator_ += deltaTime_;

        while(timeAccumulator_ >= app_step_time_ ) {
           timeAccumulator_ -= app_step_time_;

            // ------------- 1) Apply ego control, NPC AI, kinematic control, etc. -------------  //
            
            // App specific stuff ()
            // main_app_func();

            // ------------- 2) Update physics ------------- //
            if(app_step_time_ <= lazyECS::RenderingSystem::GetTimeStep()) {
                physicsSys->Update();
                std::cout << "physics updated INSIDE" << std::endl;
            }
        }

        // ------------- 2) Update physics ------------- //
        if(app_step_time_ > lazyECS::RenderingSystem::GetTimeStep()) {
            std::cout << "start phy update outside" << std::endl;
            physicsSys->Update();
        }

        // ------------- 3) Update graphics ------------- //
        // a) Update debugging primities        
        // b) Main entity graphics update
        std::cout << "start Graphics updated OUTSIDE" << std::endl;
        renderSys->Update();
                
    };

    // start the render timer thread to trigger render pipeline periodically
    std::thread render_timer_thread(&lazyECS::RenderingSystem::TimerThreadFunc, renderSys);      

    try {
        while(true) {
            loop_step();
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Caught exception in lazyECS main loop: " << e.what() << std::endl;
    }

    render_timer_thread.join();

}