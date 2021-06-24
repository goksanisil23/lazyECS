#include "PotentialField.h"
#include "Spawner.hpp"
#include <reactphysics3d/mathematics/Quaternion.h>
#include <reactphysics3d/mathematics/Transform.h>
#include <reactphysics3d/mathematics/Vector3.h>
#include <utility>
#include <vector>

extern json launch_obj;

PotentialField::PotentialField(bool isFullscreen, int windowWidth, int windowHeight) {

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

    // Create the entities and assign components

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

    // Filter the entities based on their logical Tag, and point them into Application Actors
    std::vector<lazyECS::Entity> ego_entities = tagSys->GetEntitiesWithTag("ego");
    for(const auto& ego_ent : ego_entities) {
        auto ent_trans = gOrchestrator.GetComponent<lazyECS::Transform3D>(ego_ent);
        p_field::Position actor_pos(ent_trans.rp3d_transform.getPosition().x, ent_trans.rp3d_transform.getPosition().z);
        egoActors_.insert(std::make_pair(ego_ent, Ego(0.001F, 6, actor_pos)));
    }

    std::vector<lazyECS::Entity> goal_entities = tagSys->GetEntitiesWithTag("goal");
    for(const auto& goal_ent : goal_entities) {
        auto ent_trans = gOrchestrator.GetComponent<lazyECS::Transform3D>(goal_ent);
        p_field::Position actor_pos(ent_trans.rp3d_transform.getPosition().x, ent_trans.rp3d_transform.getPosition().z);
        goalActors_.insert(std::make_pair(goal_ent, Goal(50.0F, actor_pos)));
    }

    std::vector<lazyECS::Entity> obst_entities = tagSys->GetEntitiesWithTag("obstacle");
    for(const auto& obst_ent : obst_entities) {
        auto ent_trans = gOrchestrator.GetComponent<lazyECS::Transform3D>(obst_ent);
        p_field::Position actor_pos(ent_trans.rp3d_transform.getPosition().x, ent_trans.rp3d_transform.getPosition().z);
        obstacleActors_.insert(std::make_pair(obst_ent, Obstacle(5.0F, actor_pos)));
    }        

}

void PotentialField::main_loop() {

    auto main_loop_step = [this]() {

        // ------------- 1) Apply ego control, NPC AI, kinematic control, etc. -------------  //
        for(const auto& entity : this->physicsSys->m_entities) {
            auto& rigid_body = gOrchestrator.GetComponent<lazyECS::RigidBody3D>(entity);
            auto& tag = gOrchestrator.GetComponent<lazyECS::Tag>(entity);
            auto transform = gOrchestrator.GetComponent<lazyECS::Transform3D>(entity);
            
            if(tag.mTag == "ego") {
                auto& ego  = egoActors_.at(entity);
                // update ego position from physics system
                ego.position_ = p_field::Position(transform.rp3d_transform.getPosition().x, transform.rp3d_transform.getPosition().z);
                // calculate possible moves with the current position information
                ego.CalculatePossibleMoves();
                // calculate the best possible move among possibilities
                p_field::Position best_move = ego.ComputeBestMove(obstacleActors_, goalActors_);
                // Move the ego based on best action
                auto new_trans = rigid_body.rp3d_rigidBody->getTransform();
                new_trans.setPosition(new_trans.getPosition() + rp3d::Vector3(best_move.x,0,best_move.y));
                std::cout << best_move.x << " " << best_move.y << std::endl;
                rigid_body.rp3d_rigidBody->setTransform(new_trans);
            }
            

            // if(rigid_body.rp3d_bodyType == rp3d::BodyType::DYNAMIC) { // set force for the dynamic bodies
            //     // rigid_body.rp3d_rigidBody->applyForceToCenterOfMass(rp3d::Vector3(1.0,0,1.0));
            //     auto new_trans = rigid_body.rp3d_rigidBody->getTransform();
            //     new_trans.setPosition(new_trans.getPosition() + rp3d::Vector3(0.1,0,0));
            //     rigid_body.rp3d_rigidBody->setTransform(new_trans);
            // }

            //     else if (rigid_body.rp3d_bodyType == rp3d::BodyType::KINEMATIC) { // set velocity for the kinematic body
            //         // rigid_body.rp3d_rigidBody->setLinearVelocity(rp3d::Vector3(0.2,0.0,0.0));
            //         auto new_trans = rigid_body.rp3d_rigidBody->getTransform();
            //         new_trans.setPosition(new_trans.getPosition() + rp3d::Vector3(0,0.1,0));
            //         rigid_body.rp3d_rigidBody->setTransform(new_trans);
            //     }
            // }

            // // Update render only entities
            // for(const auto& entity : this->renderSys->m_entities) {
            //     if (!gOrchestrator.CheckComponentExistsInEntity<lazyECS::RigidBody3D>(entity)) { // if render only entity
            //         auto & trans = gOrchestrator.GetComponent<lazyECS::Transform3D>(entity);
            //         trans.rp3d_transform.setPosition(trans.rp3d_transform.getPosition() + rp3d::Vector3(0,0,0.02));  
            //     }
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