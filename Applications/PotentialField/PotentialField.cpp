#include "PotentialField.h"

#include "RigidBody3D.hpp"
#include "Spawner.hpp"
#include "Transform3D.hpp"

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

PotentialField::PotentialField(bool isFullscreen, int windowWidth, int windowHeight) 
    : goalReached_(false), rand_eng_(rand_dev_()), 
    rand_dist_{std::uniform_int_distribution<int>(-GRID_SIZE_X/2.0, GRID_SIZE_X/2.0)},
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

void PotentialField::init() {
    // Create application specific Actors here
    // 1) Filter the entities based on their logical Tag
    // 2) Construct App specific Actors
    // 3) Point Actors to entities based on the tag group

    std::vector<lazyECS::Entity> ego_entities = tagSys->GetEntitiesWithTag("ego");
    for(const auto& ego_ent : ego_entities) {
        auto ent_trans = gOrchestrator.GetComponent<lazyECS::Transform3D>(ego_ent);
        p_field::Position actor_pos(ent_trans.rp3d_transform.getPosition().x, ent_trans.rp3d_transform.getPosition().z);
        egoActors_.insert(std::make_pair(ego_ent, Ego(EGO_RADIUS_, 20, actor_pos)));
    }

    std::vector<lazyECS::Entity> goal_entities = tagSys->GetEntitiesWithTag("goal");
    for(const auto& goal_ent : goal_entities) {
        auto ent_trans = gOrchestrator.GetComponent<lazyECS::Transform3D>(goal_ent);
        p_field::Position actor_pos(ent_trans.rp3d_transform.getPosition().x, ent_trans.rp3d_transform.getPosition().z);
        goalActors_.insert(std::make_pair(goal_ent, Goal(GOAL_STD_DEV_, actor_pos)));
    }

    std::vector<lazyECS::Entity> obst_entities = tagSys->GetEntitiesWithTag("obstacle");
    for(const auto& obst_ent : obst_entities) {
        auto ent_trans = gOrchestrator.GetComponent<lazyECS::Transform3D>(obst_ent);
        p_field::Position actor_pos(ent_trans.rp3d_transform.getPosition().x, ent_trans.rp3d_transform.getPosition().z);
        obstacleActors_.insert(std::make_pair(obst_ent, Obstacle(OBST_STD_DEV_, actor_pos)));
    }    
    
    // Initialize visual debug elements
    this->resetStaticDebugElements();

}

void PotentialField::main_loop() {

    auto main_loop_step = [this]() {

        // ------------- 1) Apply ego control, NPC AI, kinematic control, etc. -------------  //

        // Adjust the iteration rate for Control (not the engine)
        auto current_app_time = std::chrono::high_resolution_clock::now();
        this->deltaTime_ = std::chrono::duration<float, std::chrono::seconds::period>(current_app_time-prevAppTime_).count();
        this->prevAppTime_ = current_app_time; // update previous time
        this->timeAccumulator_ += deltaTime_;

        while(this->timeAccumulator_ >= APP_STEP_TIME ) {
            this->timeAccumulator_ -= APP_STEP_TIME;

            for(const auto& entity : this->physicsSys->m_entities) {
                auto& rigid_body = gOrchestrator.GetComponent<lazyECS::RigidBody3D>(entity);
                auto& tag = gOrchestrator.GetComponent<lazyECS::Tag>(entity);
                auto transform = gOrchestrator.GetComponent<lazyECS::Transform3D>(entity);
                
                if(tag.mTag == "ego") {

                    auto& ego  = egoActors_.at(entity);
                    // update ego position from physics system
                    ego.position_ = p_field::Position(transform.rp3d_transform.getPosition().x, transform.rp3d_transform.getPosition().z);
                    // calculate the best possible move among possibilities
                    p_field::Position best_move; float cur_heading;
                    std::tie(best_move, cur_heading) = ego.ComputeBestMove(obstacleActors_, goalActors_); 
                    // Move the ego based on best action
                    rigid_body.rp3d_rigidBody->setTransform(rp3d::Transform(rp3d::Vector3(best_move.x,0,best_move.z),
                                                                            rp3d::Quaternion::fromEulerAngles(rp3d::Vector3(0.0, cur_heading + rp3d::PI, 0.0))));

                                                    
                    if(ego.goalReached_) {
                        this->goalReached_ = true;
                        ego.goalReached_ = false;
                    }
                }
                else if(goalReached_) {
                    ResetActorPositions();
                    this->goalReached_ = false;
                    this->resetStaticDebugElements();
                }
            }

            // // ------------- 2) Update physics ------------- //
            // (needs to happen right after Actor applies force/teleports on rigid body)
            if(APP_STEP_TIME < lazyECS::RenderingSystem::GetTimeStep())
                this->physicsSys->Update();
        }
        
        if(APP_STEP_TIME > lazyECS::RenderingSystem::GetTimeStep())
            this->physicsSys->Update();

        // ------------- 3) Update graphics ------------- //
        // a) Update debugging primities
        this->UpdateDynamicDebugElements();
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

void PotentialField::resetStaticDebugElements() {

    // Reset the trajectory and grid
    renderSys->mDebugSpheres.clear();
    renderSys->mDebugRectangles.clear();
    renderSys->mDebugArrows.clear();


    // Draw a grid on the terrain to visualize the potential field
    std::pair<float, float> x_limit = std::make_pair(-GRID_SIZE_X/2.0, GRID_SIZE_X/2.0);
    std::pair<float, float> z_limit = std::make_pair(-GRID_SIZE_Z/2.0, GRID_SIZE_Z/2.0);
    float grid_resolution = GRID_RES;
    float max_force = -std::numeric_limits<float>::infinity();
    float min_force = std::numeric_limits<float>::infinity();
    std::queue<float> forces_at_points; // serialized vector of forces at the 2D grid
    // 1st pass to find min/max for color scaling
    for(int i = 0; i < (x_limit.second-x_limit.first)/(grid_resolution)+1; i++) { // terrain boundary on x
        for(int j = 0; j < (z_limit.second-z_limit.first)/(grid_resolution)+1; j++) { // terrain boundary on z
            float x_coord = x_limit.first + i*grid_resolution;
            float z_coord = z_limit.first + j*grid_resolution;
            float force_at_point = GetResultantForceAtPoint(p_field::Position(x_coord, z_coord)) * 10.0F; // scaled up for visibility
            min_force = (force_at_point < min_force) ? force_at_point : min_force;
            max_force = (force_at_point > max_force) ? force_at_point : max_force;
            forces_at_points.push(force_at_point);               
        }
    }
    // 2nd pass to write actual color values
    for(int i = 0; i < (x_limit.second-x_limit.first)/(grid_resolution)+1; i++) { // terrain boundary on x
        for(int j = 0; j < (z_limit.second-z_limit.first)/(grid_resolution)+1; j++) { // terrain boundary on z
            float x_coord = x_limit.first + i*grid_resolution;
            float z_coord = z_limit.first + j*grid_resolution;
            float force_at_point = forces_at_points.front(); // since read & write orders are the same
            forces_at_points.pop();
        
            float normalization_factor = (force_at_point - min_force)/(max_force-min_force);
            auto normalized_color = static_cast<uint32_t>(normalization_factor* static_cast<float>(0xFFFFFF));

            renderSys->mDebugRectangles.emplace_back(lazyECS::RenderingSystem::DebugRectangle(
                rp3d::Transform(rp3d::Vector3(x_coord, -0.0, z_coord), rp3d::Quaternion::identity()),
                 rp3d::Vector3(GRID_RES/2.0-0.01, 0, GRID_RES/2.0-0.01), normalized_color)); 
        }
    }

    // Debug arrow
    renderSys->mDebugArrows.emplace_back(lazyECS::RenderingSystem::DebugArrow(
        rp3d::Transform(rp3d::Vector3(0,0,0),rp3d::Quaternion::fromEulerAngles(rp3d::Vector3(0.0, 0.0, 0.0 ))),
        rp3d::DebugRenderer::DebugColor::BLACK
    ));

}


// given a list of obstacles and goals, calculate the resultant force at this position
float PotentialField::GetResultantForceAtPoint(const p_field::Position& position) {
        float action_value = 0.0F;
        // Add all the forces at this position, based on all the surrounding Goals and Obstacles
        for(const auto& goal_actor : goalActors_) {
            action_value += goal_actor.second.GetAttractionForce(position);
        }
        for(const auto& obst_actor : obstacleActors_) {
            action_value += obst_actor.second.GetRepulsionForce(position);
        } 
        return action_value;
}

void PotentialField::ResetActorPositions() {
        for(const auto& entity : this->physicsSys->m_entities) {
            auto& tag = gOrchestrator.GetComponent<lazyECS::Tag>(entity);
            auto rigid_body = gOrchestrator.GetComponent<lazyECS::RigidBody3D>(entity);

            if((tag.mTag == "goal") || (tag.mTag == "ego") || (tag.mTag == "obstacle")) {
                // reset position in physics system (which will internally update graphics too)
                auto new_x = static_cast<float>(rand_dist_(rand_eng_));
                auto new_z = static_cast<float>(rand_dist_(rand_eng_));
                rigid_body.rp3d_rigidBody->setTransform(rp3d::Transform(rp3d::Vector3(new_x,0,new_z),
                                                        rp3d::Quaternion::identity()));
                // Update position in the application
                if(tag.mTag == "goal") {
                    goalActors_[entity].position_ = p_field::Position(new_x, new_z);
                }
                if(tag.mTag == "obstacle") {
                    obstacleActors_[entity].position_ = p_field::Position(new_x, new_z);
                }
                if(tag.mTag == "ego") {
                    egoActors_[entity].position_ = p_field::Position(new_x, new_z);
                }                                
                
            }
        }
}

void PotentialField::UpdateDynamicDebugElements() {
        auto ego_entities = tagSys->GetEntitiesWithTag("ego");
        for(const auto& entity : ego_entities) {
            const auto& transform = gOrchestrator.GetComponent<lazyECS::Transform3D>(entity);
             // Debug draw the previous location
            renderSys->mDebugSpheres.emplace_back(lazyECS::RenderingSystem::DebugSphere(transform.rp3d_transform.getPosition(),
                                                                        0.05, rp3d::DebugRenderer::DebugColor::BLUE));
            // Set the direction arrow (debug render)
            const auto cur_rotation = transform.rp3d_transform.getOrientation();
            renderSys->mDebugArrows.at(0).transform.setOrientation(cur_rotation);
            renderSys->mDebugArrows.at(0).transform.setPosition(rp3d::Vector3(transform.rp3d_transform.getPosition().x,
                                                                              0,
                                                                              transform.rp3d_transform.getPosition().z));           
        
        }                                                                 
}