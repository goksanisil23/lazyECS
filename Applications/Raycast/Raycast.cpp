#include "Raycast.h"

#include "RigidBody3D.hpp"
#include "Spawner.hpp"
#include "Transform3D.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <random>
#include <reactphysics3d/configuration.h>
#include <reactphysics3d/mathematics/Quaternion.h>
#include <reactphysics3d/mathematics/Ray.h>
#include <reactphysics3d/mathematics/Transform.h>
#include <reactphysics3d/mathematics/Vector3.h>
#include <reactphysics3d/utils/DebugRenderer.h>

extern json launch_obj;

Raycast::Raycast(bool isFullscreen, int windowWidth, int windowHeight) : 
    rand_eng_(rand_dev_()),
    prevAppTime_{std::chrono::high_resolution_clock::now()},  deltaTime_(0.0),timeAccumulator_(0.0)
{
    // --------------- lAZYECS -------------- //

    gOrchestrator.Init(); // initializes the ECS managers

    // Register the components
    gOrchestrator.RegisterComponentType<lazyECS::Transform3D>();
    gOrchestrator.RegisterComponentType<lazyECS::RigidBody3D>();
    gOrchestrator.RegisterComponentType<lazyECS::Mesh>();
    gOrchestrator.RegisterComponentType<lazyECS::Tag>();

    // Register the systems (calls Constructors during registration)
    physicsSys = gOrchestrator.RegisterSystem<lazyECS::PhysicsSystem>();
    renderSys = gOrchestrator.RegisterSystem<lazyECS::RenderingSystem>(isFullscreen, windowWidth, windowHeight, "Potential Field");
    tagSys = gOrchestrator.RegisterSystem<lazyECS::TagSystem>();
    manControlSys = gOrchestrator.RegisterSystem<lazyECS::ManualControlSystem>();

    // entities below will be matched with systems so signature of the system is set
    renderSys->SetupSignature(); 
    physicsSys->SetupSignature();
    tagSys->SetupSignature();
    manControlSys->SetupSignature();

    // Create the entities and assign components (parsing the json file)
    json entities_json = launch_obj.at("entities");    
    lazyECS::Spawner::CreatePhysicsEntities(entities_json);
    lazyECS::Spawner::CreateTerrainEntity(entities_json);
    lazyECS::Spawner::CreateRenderOnlyEntities(entities_json);
   
    // Init requires the entities to be initialized and entities requires system's signature to be set
    renderSys->Init();
    physicsSys->Init();
    manControlSys->Init("ego");


    // run Application specific init
    this->init();
}

void Raycast::init() {
    // Create application specific Actors here
    // 1) Filter the entities based on their logical Tag
    // 2) Construct App specific Actors
    // 3) Point Actors to entities based on the tag group

    // std::vector<lazyECS::Entity> ego_entities = tagSys->GetEntitiesWithTag("ego");
    // for(const auto& ego_ent : ego_entities) {
    //     egoActors_.insert(std::make_pair(ego_ent, Ego()));
    // }

    // std::vector<lazyECS::Entity> goal_entities = tagSys->GetEntitiesWithTag("goal");
    // for(const auto& goal_ent : goal_entities) {
    //     goalActors_.insert(std::make_pair(goal_ent, Goal()));
    // }

    // std::vector<lazyECS::Entity> obst_entities = tagSys->GetEntitiesWithTag("obstacle");
    // for(const auto& obst_ent : obst_entities) {
    //     obstacleActors_.insert(std::make_pair(obst_ent, Obstacle()));
    // }

    // Populate Application parameters
    app_step_time_ = launch_obj.at("application").at("APP_STEP_TIME");

    // Initialize random distribution for resetting actor positions after episode
    // rand_dist_ = std::uniform_int_distribution<int>(-grid_size_x_/2.0, grid_size_x_/2.0);  
}

void Raycast::main_loop() {

    auto main_loop_step = [this]() {

        // ------------- 1) Apply ego control, NPC AI, kinematic control, etc. -------------  //

        // Adjust the iteration rate for Control (not the engine)
        auto current_app_time = std::chrono::high_resolution_clock::now();
        this->deltaTime_ = std::chrono::duration<float, std::chrono::seconds::period>(current_app_time-prevAppTime_).count();
        this->prevAppTime_ = current_app_time; // update previous time
        this->timeAccumulator_ += deltaTime_;

        while(this->timeAccumulator_ >= app_step_time_ ) {
            this->timeAccumulator_ -= app_step_time_;


            renderSys->mDebugSpheres.clear();
            renderSys->mDebugRays.clear();
            physicsSys->raycast_manager_.hit_points_.clear();
            
            const auto& ego_pos = gOrchestrator.GetComponent<lazyECS::Transform3D>(tagSys->GetEntitiesWithTag("ego").at(0)).rp3d_transform.getPosition();

            for(int i = 0; i < 30; i++) {
                // rp3d::Ray ray(rp3d::Vector3(0,0,0), rp3d::Vector3(-15+i,0,-30));
                rp3d::Ray ray(rp3d::Vector3(ego_pos.x, ego_pos.y, ego_pos.z - 0.3), 
                              rp3d::Vector3(ego_pos.x -15 + i, ego_pos.y, ego_pos.z - 0.3 - 30));
                renderSys->mDebugRays.emplace_back(ray);
                physicsSys->RayCast(ray);
            }
            for(const auto& hit_point : physicsSys->raycast_manager_.hit_points_) {
                // std::cout << hit_point.x << " " << hit_point.y << " " << hit_point.z << std::endl;
                renderSys->mDebugSpheres.emplace_back(lazyECS::RenderingSystem::DebugSphere(hit_point,
                                                                    0.05, rp3d::DebugRenderer::DebugColor::BLACK));                
            }             

            // for(auto& pair : egoActors_) {
            //     const auto& ego_entity = pair.first;
            //     auto& ego_actor = pair.second;
                
            // }
            // ------------- 2) Update physics ------------- //
            if(app_step_time_ <= lazyECS::RenderingSystem::GetTimeStep())
                this->physicsSys->Update();            
        }

        // ------------- 2) Update physics ------------- //
        if(app_step_time_ > lazyECS::RenderingSystem::GetTimeStep())
            this->physicsSys->Update();

        // ------------- 3) Update graphics ------------- //
        // a) Update debugging primities        
        // b) Main entity graphics update
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
