#include "WaveFront.h"

#include "RigidBody3D.hpp"
#include "Spawner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <reactphysics3d/configuration.h>
#include <reactphysics3d/mathematics/Quaternion.h>
#include <reactphysics3d/mathematics/Transform.h>
#include <reactphysics3d/mathematics/Vector3.h>
#include <reactphysics3d/utils/DebugRenderer.h>
#include <utility>
#include <vector>

extern json launch_obj;

WaveFront::WaveFront(bool isFullscreen, int windowWidth, int windowHeight)
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

    // entities below will be matched with systems so signature of the system is set
    renderSys->SetupSignature(); 
    physicsSys->SetupSignature();
    tagSys->SetupSignature();

    // Create the entities and assign components (parsing the json file)
    json entities_json = launch_obj.at("entities");    
    lazyECS::Spawner::CreatePhysicsEntities(entities_json);
    lazyECS::Spawner::CreateTerrainEntity(entities_json);
    lazyECS::Spawner::CreateRenderOnlyEntities(entities_json);
   
    // Init requires the entities to be initialized and entities requires system's signature to be set
    renderSys->Init();
    physicsSys->Init();

    // run Application specific init
    this->init();
}

void WaveFront::init() {
    // Create application specific Actors here
    // 1) Filter the entities based on their logical Tag
    // 2) Construct App specific Actors
    // 3) Point Actors to entities based on the tag group

    std::vector<lazyECS::Entity> ego_entities = tagSys->GetEntitiesWithTag("ego");
    for(const auto& ego_ent : ego_entities) {
        auto ent_trans = gOrchestrator.GetComponent<lazyECS::Transform3D>(ego_ent);
        Position2D actor_pos(ent_trans.rp3d_transform.getPosition().x, ent_trans.rp3d_transform.getPosition().z);
        egoActors_.insert(std::make_pair(ego_ent, Ego(actor_pos)));
    }

    std::vector<lazyECS::Entity> goal_entities = tagSys->GetEntitiesWithTag("goal");
    for(const auto& goal_ent : goal_entities) {
        auto ent_trans = gOrchestrator.GetComponent<lazyECS::Transform3D>(goal_ent);
        Position2D actor_pos(ent_trans.rp3d_transform.getPosition().x, ent_trans.rp3d_transform.getPosition().z);
        goalActors_.insert(std::make_pair(goal_ent, Goal(actor_pos)));
    }

    std::vector<lazyECS::Entity> obst_entities = tagSys->GetEntitiesWithTag("obstacle");
    for(const auto& obst_ent : obst_entities) {
        auto ent_trans = gOrchestrator.GetComponent<lazyECS::Transform3D>(obst_ent);
        Position2D actor_pos(ent_trans.rp3d_transform.getPosition().x, ent_trans.rp3d_transform.getPosition().z);
        obstacleActors_.insert(std::make_pair(obst_ent, Obstacle(actor_pos)));
    }

    // Setup the grid objects
    this->SetupGridCells();
}

void WaveFront::main_loop() {

    auto main_loop_step = [this]() {

        // ------------- 1) Apply ego control, NPC AI, kinematic control, etc. -------------  //

        // ------------- 2) Update physics ------------- //
        // (needs to happen right after Actor applies force/teleports on rigid body)
        this->physicsSys->Update();


        // ------------- 3) Update graphics ------------- //
        
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

void WaveFront::SetupGridCells() {

    const float grid_size_x = launch_obj.at("application").at("GRID_SIZE_X");
    const float grid_size_z = launch_obj.at("application").at("GRID_SIZE_Z");
    const float grid_resolution = launch_obj.at("application").at("GRID_RESOLUTION");

    std::pair<float, float> x_limit = std::make_pair(-grid_size_x/2.0, grid_size_x/2.0);
    std::pair<float, float> z_limit = std::make_pair(-grid_size_z/2.0, grid_size_z/2.0);

    for(int i = 0; i < (x_limit.second-x_limit.first)/(grid_resolution)+1; i++) { // terrain boundary on x
        for(int j = 0; j < (z_limit.second-z_limit.first)/(grid_resolution)+1; j++) { // terrain boundary on z
            float x_coord = x_limit.first + i*grid_resolution;
            float z_coord = z_limit.first + j*grid_resolution;

            rp3d::Vector3 grid_min_coord(x_coord - (grid_resolution/2.0-0.01), 0.0, z_coord - (grid_resolution/2.0-0.01));
            rp3d::Vector3 grid_max_coord(z_coord + (grid_resolution/2.0-0.01), 0.0, z_coord + (grid_resolution/2.0-0.01));

            gridCells_.emplace_back(GridCell(grid_min_coord, grid_max_coord));

            renderSys->mDebugRectangles.emplace_back(lazyECS::RenderingSystem::DebugRectangle(
                rp3d::Transform(rp3d::Vector3(x_coord, -0.0, z_coord), rp3d::Quaternion::identity()),
                 rp3d::Vector3(grid_resolution/2.0-0.01, 0, grid_resolution/2.0-0.01), 0xFFFF00));             
        }
    }    



}

