#include "Raycast.h"

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

Raycast::Raycast() :
    BaseApp{false, 800, 800}, // BaseApp(isFullscreen, windowWidth, windowHeight) 
    rand_eng_{rand_dev_()}
{
    // --------------- Application -------------- //
    app_step_time_ = launch_obj.at("application").at("APP_STEP_TIME");

    init();
}

void Raycast::init() {
    // Create application specific Actors here
    //      1) Filter the entities based on their logical Tag
    //      2) Construct App specific Actors
    //      3) Point Actors to entities based on the tag group

    std::vector<lazyECS::Entity> ego_entities = tagSys->GetEntitiesWithTag("ego");
    for(const auto& ego_ent : ego_entities) {
        auto ego_actor = Ego(renderSys, physicsSys, 
                            gOrchestrator.GetComponent<lazyECS::Transform3D>(ego_ent).rp3d_transform.getPosition(),
                            gOrchestrator.GetComponent<lazyECS::Transform3D>(ego_ent).rp3d_transform.getOrientation()
                            );
        egoActors_.insert(std::make_pair(ego_ent, std::move(ego_actor)));
    }

    // std::vector<lazyECS::Entity> goal_entities = tagSys->GetEntitiesWithTag("goal");
    // for(const auto& goal_ent : goal_entities) {
    //     goalActors_.insert(std::make_pair(goal_ent, Goal()));
    // }

    // std::vector<lazyECS::Entity> obst_entities = tagSys->GetEntitiesWithTag("obstacle");
    // for(const auto& obst_ent : obst_entities) {
    //     obstacleActors_.insert(std::make_pair(obst_ent, Obstacle()));
    // }

    // Initialize random distribution for resetting actor positions after episode
    // rand_dist_ = std::uniform_int_distribution<int>(-grid_size_x_/2.0, grid_size_x_/2.0);  
}

// runs every app_step_time_ interval
void Raycast::app_func() {

    renderSys->mDebugSpheres.clear();
    renderSys->mDebugRays.clear();
    physicsSys->raycast_manager_.hit_points_.clear();

    for(auto& pair : egoActors_) {
        const auto& ego_entity = pair.first;
        auto& ego_actor = pair.second;

        ego_actor.step();
    }
    
    // Apply force to the wheels

}


