#include "WaveFront.h"

#include "RigidBody3D.hpp"
#include "Spawner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <list>
#include <random>
#include <reactphysics3d/configuration.h>
#include <reactphysics3d/mathematics/Quaternion.h>
#include <reactphysics3d/mathematics/Transform.h>
#include <reactphysics3d/mathematics/Vector3.h>
#include <reactphysics3d/utils/DebugRenderer.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <unordered_set>

extern json launch_obj;

WaveFront::WaveFront(bool isFullscreen, int windowWidth, int windowHeight) : 
    goalsReached_(false), rand_eng_(rand_dev_()),
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
        egoActors_.insert(std::make_pair(ego_ent, Ego()));
    }

    std::vector<lazyECS::Entity> goal_entities = tagSys->GetEntitiesWithTag("goal");
    for(const auto& goal_ent : goal_entities) {
        goalActors_.insert(std::make_pair(goal_ent, Goal()));
    }

    std::vector<lazyECS::Entity> obst_entities = tagSys->GetEntitiesWithTag("obstacle");
    for(const auto& obst_ent : obst_entities) {
        obstacleActors_.insert(std::make_pair(obst_ent, Obstacle()));
    }

    // Populate Application parameters (GRID parameters)
    grid_size_x_ = launch_obj.at("application").at("GRID_SIZE_X");
    grid_size_z_ = launch_obj.at("application").at("GRID_SIZE_Z");
    grid_resolution_ = launch_obj.at("application").at("GRID_RESOLUTION");
    app_step_time_ = launch_obj.at("application").at("APP_STEP_TIME");

    rand_dist_ = std::uniform_int_distribution<int>(-grid_size_x_/2.0, grid_size_x_/2.0);  

    grid_x_limit_ = std::make_pair(-grid_size_x_/2.0, grid_size_x_/2.0);
    grid_z_limit_ = std::make_pair(-grid_size_z_/2.0, grid_size_z_/2.0);

    num_cells_x_ = static_cast<uint32_t>((grid_x_limit_.second-grid_x_limit_.first) / (grid_resolution_)) + 1;
    num_cells_z_ = static_cast<uint32_t>((grid_z_limit_.second-grid_z_limit_.first) / (grid_resolution_)) + 1;

    // Setup the grid objects (must be done after the actors above are spawned, since checks for occupancy of the cells as well)
    this->CreateGridCells();
    this->ResetGridStatus();
    this->UpdateGridOccupancy();
    this->ShowGridValues(); // before executing the wave propagation
    this->RunWavePropagation();
    this->ShowGridValues(); // after executing the wave propagation
}

void WaveFront::main_loop() {

    auto main_loop_step = [this]() {

        // ------------- 1) Apply ego control, NPC AI, kinematic control, etc. -------------  //

        // Adjust the iteration rate for Control (not the engine)
        auto current_app_time = std::chrono::high_resolution_clock::now();
        this->deltaTime_ = std::chrono::duration<float, std::chrono::seconds::period>(current_app_time-prevAppTime_).count();
        this->prevAppTime_ = current_app_time; // update previous time
        this->timeAccumulator_ += deltaTime_;

        while(this->timeAccumulator_ >= app_step_time_ ) {
            this->timeAccumulator_ -= app_step_time_;
            for(auto& pair : egoActors_) {
                const auto& ego_entity = pair.first;
                auto& ego_actor = pair.second;
                
                // Calculate the shortest path from ego to the goal
                if(!ego_actor.pathCalculated_) {
                    ego_actor.FindShortestPath(this->gridCells_, this->num_cells_x_,this->num_cells_z_);
                    HighlightPath(ego_actor.path_);
                }
                else { // Move to the goal position by using the path calculated above
                       // move the physical entity that is connected to the ego (to the center of the grid cell)
                    if(!ego_actor.path_.empty()) {
                        auto& rigid_body = gOrchestrator.GetComponent<lazyECS::RigidBody3D>(ego_entity);
                        const auto next_idx = std::move(ego_actor.path_.front());
                        ego_actor.path_.pop_front();
                        rigid_body.rp3d_rigidBody->setTransform(rp3d::Transform(gridCells_.at(next_idx).aabb_.getCenter(), 
                                                                                rp3d::Quaternion::identity()));
                    }
                    else { // goal has been reached, so reset the episode
                        ResetActorPositions();
                        ResetGridStatus();
                        ego_actor.pathCalculated_ = false;
                        this->UpdateGridOccupancy();
                        this->ShowGridValues(); // before executing the wave propagation
                        this->RunWavePropagation();
                        this->ShowGridValues(); // after executing the wave propagation                        
                    }
                }
            }

        }
        // ------------- 2) Update physics ------------- //
        // (needs to happen right after Actor applies force/teleports on rigid body)
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

void WaveFront::HighlightPath(const std::deque<uint16_t>& ego_path) {
    for(const auto& path_cell : ego_path) {
        gridCells_.at(path_cell).rectangle_.color = static_cast<uint32_t>(GridColor::TRAVELED);
        renderSys->mDebugRectangles.at(path_cell) = gridCells_.at(path_cell).rectangle_;
    }    
}

void WaveFront::CreateGridCells() {

    for(uint32_t j = 0; j < num_cells_z_; j++) { // terrain boundary on z
        for(uint32_t i = 0; i < num_cells_x_; i++) { // terrain boundary on x
            float x_coord = grid_x_limit_.first + static_cast<float>(i) * grid_resolution_;
            float z_coord = grid_z_limit_.first + static_cast<float>(j) * grid_resolution_;

            rp3d::Vector3 cell_min_coord(x_coord - (grid_resolution_/2.0-0.01), 0.11, z_coord - (grid_resolution_/2.0-0.01));
            rp3d::Vector3 cell_max_coord(x_coord + (grid_resolution_/2.0-0.01), 0.11, z_coord + (grid_resolution_/2.0-0.01));

            // we are giving lower left and upper right global vertex coordinates here
            auto grid_cell = GridCell(cell_min_coord, cell_max_coord, i, j);
            gridCells_.emplace_back(grid_cell);
            renderSys->mDebugRectangles.emplace_back(grid_cell.rectangle_);
        }
    }  
}

// Resets all distance and color values
void WaveFront::ResetGridStatus() {
    goalsReached_ = false;

    for(auto& cell : gridCells_) {
        cell.distance_ = 0;
        cell.rectangle_.color = static_cast<uint32_t>(GridColor::FREE);
        renderSys->mDebugRectangles.at(cell.z_idx_ * num_cells_x_ + cell.x_idx_) = cell.rectangle_;
    }
}

void WaveFront::UpdateGridOccupancy() {

    // for each type of actor, check which grid cell it's occupying and update the status of the cell

    for(auto& obstacle : obstacleActors_) { 
        obstacle.second.x_idx_.clear();
        obstacle.second.z_idx_.clear();
        for(auto& cell : gridCells_) {
            auto obstacle_aabb = gOrchestrator.GetComponent<lazyECS::RigidBody3D>(obstacle.first).rp3d_collider->getWorldAABB();
            // find and update the rectangle in Rendering System by using the indices of the cell
            if(cell.UpdateOccupancy(obstacle_aabb, CellState::OBSTACLE)) {
                std::cout << "obstacle at: " << cell.x_idx_ << " " << cell.z_idx_ << "\n";
                // update the debug renderer for this cell
                renderSys->mDebugRectangles.at(cell.z_idx_ * num_cells_x_ + cell.x_idx_) = cell.rectangle_;
                // Update the cell index of the obstacle
                obstacle.second.x_idx_.emplace_back(cell.x_idx_);
                obstacle.second.z_idx_.emplace_back(cell.z_idx_);
                // Found where obstacle is, no need to search the grid further            
                // break;
            }
        }
    }

    for(auto& goal : goalActors_) { 
        goal.second.x_idx_.clear();
        goal.second.z_idx_.clear();          
        for(auto& cell : gridCells_) {
            auto goal_aabb = gOrchestrator.GetComponent<lazyECS::RigidBody3D>(goal.first).rp3d_collider->getWorldAABB();          
            // find and update the rectangle in Rendering System by using the indices of the cell
            if(cell.UpdateOccupancy(goal_aabb, CellState::GOAL)) {
                std::cout << "goal at: " << cell.x_idx_ << " " << cell.z_idx_ << "\n";
                // update the debug renderer for this cell
                renderSys->mDebugRectangles.at(cell.z_idx_ * num_cells_x_ + cell.x_idx_) = cell.rectangle_;
                // Update the cell index of the goal
                goal.second.x_idx_.emplace_back(cell.x_idx_);
                goal.second.z_idx_.emplace_back(cell.z_idx_);                
                // Found where goal is, no need to search the grid further         
                // break;
            }
        }
    }   

    for(auto& ego : egoActors_) {
        ego.second.x_idx_.clear();
        ego.second.z_idx_.clear();           
        for(auto& cell : gridCells_) {
            auto ego_aabb = gOrchestrator.GetComponent<lazyECS::RigidBody3D>(ego.first).rp3d_collider->getWorldAABB();              
            // find and update the rectangle in Rendering System by using the indices of the cell
            if(cell.UpdateOccupancy(ego_aabb, CellState::TRAVELED)) {
                std::cout << "ego at: " << cell.x_idx_ << " " << cell.z_idx_ << "\n";
                // update the debug renderer for this cell
                renderSys->mDebugRectangles.at(cell.z_idx_ * num_cells_x_ + cell.x_idx_) = cell.rectangle_;
                // Update the cell index of the ego
                ego.second.x_idx_.emplace_back(cell.x_idx_);
                ego.second.z_idx_.emplace_back(cell.z_idx_);                  
                // Found where goal is, no need to search the grid further         
                // break;
            }
        }
    }        

}

std::pair<uint16_t, uint16_t> WaveFront::GetGoalPosition() {
    for(const auto& cell : gridCells_) {
        if(cell.distance_ == 1) {
            return std::make_pair(cell.x_idx_, cell.z_idx_);
        }
    }
    std::__throw_runtime_error("Goal Position Not Found !");
}

void WaveFront::RunWavePropagation() {

    // we start the wave propagation from the goal cell, by manually giving it distance 1
    std::vector<Node> discovered_nodes;
    std::pair<uint16_t, uint16_t> goal_position(GetGoalPosition()); 
    discovered_nodes.emplace_back(Node(goal_position.first, goal_position.second, 1));

    while(!discovered_nodes.empty()) {
        // for each discovered cell, we investigate it's neightboring nodes, and save them if they're unoccupied
        // If we started with multiple discovered_cells, we might end up with duplicate nodes in the 'free_neighbor_nodes'
        // since traversing in different directions from different cells might end up at the same cell
        // Therefore we use a set, which does not insert the duplicate elements in the first place      
        std::unordered_set<Node, NodeHashFunc> free_neighbor_nodes;
        for(const auto& discovered_cell : discovered_nodes) {
            uint16_t cur_x = discovered_cell.x;
            uint16_t cur_z = discovered_cell.z;
            uint16_t target_x;
            uint16_t target_z;
            // NORTH (if not edge and not occupied)
            target_x = cur_x; target_z = cur_z -1;
            if( (cur_z != 0) && ( gridCells_.at(target_z * num_cells_x_ + target_x).distance_ == 0 ) ) {
                free_neighbor_nodes.insert(Node(target_x,target_z,discovered_cell.distance+1));
            }
            // SOUTH (if not edge and not occupied)
            target_x = cur_x; target_z = cur_z +1;
            if( (target_z < num_cells_z_) && ( gridCells_.at(target_z * num_cells_x_ + target_x).distance_ == 0 ) ) {
                free_neighbor_nodes.insert(Node(target_x,target_z,discovered_cell.distance+1));
            }
            // WEST (if not edge and not occupied)
            target_x = cur_x -1; target_z = cur_z;
            if( (cur_x != 0) && ( gridCells_.at(target_z * num_cells_x_ + target_x).distance_ == 0 ) ) {
                free_neighbor_nodes.insert(Node(target_x,target_z,discovered_cell.distance+1));
            }
            // EAST (if not edge and not occupied)
            target_x = cur_x +1; target_z = cur_z;
            if( (target_x < num_cells_x_) && ( gridCells_.at(target_z * num_cells_x_ + target_x).distance_ == 0 ) ) {
                free_neighbor_nodes.insert(Node(target_x,target_z,discovered_cell.distance+1));
            }            
        }
        // The exploration of the neighbors of the discovered_cells is over. Now, those neighbors will be the new set of discovered_cells
        // and we'll re-run the wave propagation
        discovered_nodes.clear();
        discovered_nodes.reserve(free_neighbor_nodes.size());
        for(auto it = free_neighbor_nodes.begin(); it != free_neighbor_nodes.end(); ) { // it++ is not here since extract invalidates it
            // Update the gricCell elements to use for navigation later on
            gridCells_.at(it->z * num_cells_x_ + it->x).distance_ = it->distance;
            // move the neighbors
            discovered_nodes.push_back(std::move(free_neighbor_nodes.extract(it++).value()));
        }
    }

    // Here gridCells_ have updated distance values based on the proximity to the goal
    // Now the ego needs to move "down the gradient" based on these distance values 


}

void WaveFront::ShowGridValues() const {
    // relies on the fact that we populated gridCells_ in row-first fashion
    for(const auto& cell : gridCells_) {
        std::cout << std::setw(3) << cell.distance_ << " ";
        if(cell.x_idx_ == (num_cells_x_-1))
            std::cout << "\n";
    }
    std::cout << "------------------------------------------" << std::endl;
}

void WaveFront::ResetActorPositions() {
        for(const auto& entity : this->physicsSys->m_entities) {
            auto& tag = gOrchestrator.GetComponent<lazyECS::Tag>(entity);
            auto rigid_body = gOrchestrator.GetComponent<lazyECS::RigidBody3D>(entity);

            if((tag.mTag == "goal") || (tag.mTag == "ego") || (tag.mTag == "obstacle")) {
                // reset position in physics system (which will internally update graphics too)
                auto new_x = static_cast<float>(rand_dist_(rand_eng_));
                auto new_z = static_cast<float>(rand_dist_(rand_eng_));
                rigid_body.rp3d_rigidBody->setTransform(rp3d::Transform(rp3d::Vector3(new_x,0,new_z),
                                                        rp3d::Quaternion::identity()));                             
                
            }
        }
}
