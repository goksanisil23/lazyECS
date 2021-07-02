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

    // Populate Application parameters (GRID parameters)
    grid_size_x_ = launch_obj.at("application").at("GRID_SIZE_X");
    grid_size_z_ = launch_obj.at("application").at("GRID_SIZE_Z");
    grid_resolution_ = launch_obj.at("application").at("GRID_RESOLUTION");

    grid_x_limit_ = std::make_pair(-grid_size_x_/2.0, grid_size_x_/2.0);
    grid_z_limit_ = std::make_pair(-grid_size_z_/2.0, grid_size_z_/2.0);

    num_cells_x_ = static_cast<uint32_t>((grid_x_limit_.second-grid_x_limit_.first) / (grid_resolution_)) + 1;
    num_cells_z_ = static_cast<uint32_t>((grid_z_limit_.second-grid_z_limit_.first) / (grid_resolution_)) + 1;

    // Setup the grid objects (must be done after the actors above are spawned, since checks for occupancy of the cells as well)
    this->SetupGridCells();
    this->UpdateGridOccupancy();
}

void WaveFront::main_loop() {

    auto main_loop_step = [this]() {

        // ------------- 1) Apply ego control, NPC AI, kinematic control, etc. -------------  //

        // ------------- 2) Update physics ------------- //
        // (needs to happen right after Actor applies force/teleports on rigid body)
        this->physicsSys->Update();


        // ------------- 3) Update graphics ------------- //
        // a) Update debugging primities
        // this->UpdateGridOccupancy();
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

    for(uint32_t i = 0; i < num_cells_x_; i++) { // terrain boundary on x
        for(uint32_t j = 0; j < num_cells_z_; j++) { // terrain boundary on z
            float x_coord = grid_x_limit_.first + static_cast<float>(i) * grid_resolution_;
            float z_coord = grid_z_limit_.first + static_cast<float>(j) * grid_resolution_;

            rp3d::Vector3 cell_min_coord(x_coord - (grid_resolution_/2.0-0.01), 0.11, z_coord - (grid_resolution_/2.0-0.01));
            rp3d::Vector3 cell_max_coord(x_coord + (grid_resolution_/2.0-0.01), 0.11, z_coord + (grid_resolution_/2.0-0.01));

            auto grid_cell = GridCell(cell_min_coord, cell_max_coord, i, j);
            gridCells_.emplace_back(grid_cell); // move due to unique_ptr
            renderSys->mDebugRectangles.emplace_back(grid_cell.rectangle_);
        }
    }  
}

void WaveFront::UpdateGridOccupancy() {
    // for each type of actor, check which grid cell it's occupying and update the status of the cell
    for(const auto& obstacle : obstacleActors_) { 
        for(auto& cell : gridCells_) {
            auto obstacle_aabb = gOrchestrator.GetComponent<lazyECS::RigidBody3D>(obstacle.first).rp3d_collider->getWorldAABB();
            // find and update the rectangle in Rendering System by using the indices of the cell
            if(cell.UpdateOccupancy(obstacle_aabb, CellState::OBSTACLE)) {
                std::cout << "obstacle at: " << cell.x_idx_ << " " << cell.z_idx_ << "\n";
                // update the debug renderer for this cell
                renderSys->mDebugRectangles.at(cell.x_idx_ * num_cells_z_ + cell.z_idx_) = cell.rectangle_;
                // Found where obstacle is, no need to search the grid further            
                break;
            }
        }
    }

    for(const auto& goal : goalActors_) { 
        for(auto& cell : gridCells_) {
            auto goal_aabb = gOrchestrator.GetComponent<lazyECS::RigidBody3D>(goal.first).rp3d_collider->getWorldAABB();
            // find and update the rectangle in Rendering System by using the indices of the cell
            if(cell.UpdateOccupancy(goal_aabb, CellState::GOAL)) {
                std::cout << "goal at: " << cell.x_idx_ << " " << cell.z_idx_ << "\n";
                // update the debug renderer for this cell
                renderSys->mDebugRectangles.at(cell.x_idx_ * num_cells_z_ + cell.z_idx_) = cell.rectangle_;
                // Found where goal is, no need to search the grid further         
                break;
            }
        }
    }    

}
