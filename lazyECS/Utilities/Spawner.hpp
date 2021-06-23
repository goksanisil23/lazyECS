#pragma once

#include "ECSCore/Orchestrator.hpp"
#include "Components/RigidBody3D.hpp"
#include "Components/Transform3D.hpp"
#include "Components/Mesh.hpp"

#include <reactphysics3d/reactphysics3d.h>
#include <unordered_map>

extern lazyECS::Orchestrator gOrchestrator;

namespace lazyECS {

class Spawner {

public:

    // mapping from config file types to internal lazyECS types
    static std::unordered_map<std::string, lazyECS::Shape> stringToShape;
    static std::unordered_map<std::string, rp3d::BodyType> stringToBodyType;    

    // This is an entity template that contains Transform3D, RigidBody3D and Mesh components
    // It's preferable to use PhysicsEntity as "intelligent" actors that are controlled by force, 
    // or environmental actors that act/move based on physics
    static void CreatePhysicsEntities(const json& entities_json) {
        int ent_ctr = 0;
        for(auto& phy_obj : entities_json.at("PhysicsEntities")) {
            lazyECS::Entity entity = gOrchestrator.CreateEntity();
            ent_ctr++;

            // --------------- Transform component (Initial position) --------------- //
            rp3d::Vector3 spawn_pos(phy_obj.at("initial_position")[0], 
                                    5 + ent_ctr * 1.0,
                                    phy_obj.at("initial_position")[2]);
            rp3d::Quaternion spawn_rot = rp3d::Quaternion::fromEulerAngles(rp3d::Vector3(phy_obj.at("initial_rotation")[0],
                                                                                         phy_obj.at("initial_rotation")[1],
                                                                                         phy_obj.at("initial_rotation")[2]));
            lazyECS::Transform3D spawn_trans(spawn_pos, spawn_rot);
            spawn_trans.SetScale(phy_obj.at("scale")[0],
                                 phy_obj.at("scale")[1],
                                 phy_obj.at("scale")[2]);
            gOrchestrator.AddComponent<lazyECS::Transform3D>(entity, spawn_trans);

            // --------------- Mesh component (For rendering) --------------- //
            lazyECS::Mesh mesh(stringToShape.at(phy_obj.at("shape")));
            mesh.mColor = openglframework::Color(0.0,0.0,1.0,1.0);
            gOrchestrator.AddComponent<lazyECS::Mesh>(entity, mesh);

            // --------------- Rigid Body component (For physical motion) --------------- //
            lazyECS::RigidBody3D rigid_body; // will be initialize in Physics System
            rigid_body.rp3d_bodyType = stringToBodyType.at(phy_obj.at("body_type"));
            gOrchestrator.AddComponent<lazyECS::RigidBody3D>(entity, rigid_body);
        }
    }

    // Creates a static solid platform, meant to be used as terrain
    static void CreateTerrainEntity(const json& entities_json) {
        auto terrain_obj = entities_json.at("TerrainEntity");

        lazyECS::Entity entity = gOrchestrator.CreateEntity();

        // --------------- Mesh component (For rendering) --------------- //
        lazyECS::Mesh mesh(stringToShape.at(terrain_obj.at("shape"))); // can be heightfield, concavemesh or Box
        mesh.mColor = openglframework::Color(0.47f, 0.48f, 0.49f, 1.0f);
        gOrchestrator.AddComponent<lazyECS::Mesh>(entity, mesh);

        // --------------- Transform component (Initial position) --------------- //
        rp3d::Vector3 spawn_pos(terrain_obj.at("initial_position")[0], 
                                terrain_obj.at("initial_position")[1], 
                                terrain_obj.at("initial_position")[2]);
        rp3d::Quaternion spawn_rot = rp3d::Quaternion::fromEulerAngles(rp3d::Vector3(terrain_obj.at("initial_rotation")[0],
                                                                                     terrain_obj.at("initial_rotation")[1],
                                                                                     terrain_obj.at("initial_rotation")[2]));
        lazyECS::Transform3D spawn_trans(spawn_pos, spawn_rot);
        if(mesh.mShape != lazyECS::Shape::Hfield) // scaling problem with height fields currently
            spawn_trans.SetScale(terrain_obj.at("scale")[0],
                                    terrain_obj.at("scale")[1],
                                    terrain_obj.at("scale")[2]);
        gOrchestrator.AddComponent<lazyECS::Transform3D>(entity, spawn_trans);

        // --------------- Rigid Body component (For physical motion) --------------- //
        lazyECS::RigidBody3D rigid_body; // will be initialize in Physics System
        rigid_body.rp3d_bodyType = stringToBodyType.at(terrain_obj.at("body_type"));
        gOrchestrator.AddComponent<lazyECS::RigidBody3D>(entity, rigid_body);
    }    

    // Creates a render only entities, meant to be used for objects that does not require physical movement/interaction
    static void CreateRenderOnlyEntities(const json& entities_json) {
        int ent_ctr = 0;
        for(auto& render_obj : entities_json.at("RenderOnlyEntities")) {
            lazyECS::Entity entity = gOrchestrator.CreateEntity();
            ent_ctr++;

            // --------------- Transform component (Initial position) --------------- //
            rp3d::Vector3 spawn_pos(ent_ctr, 
                                    render_obj.at("initial_position")[1],
                                    ent_ctr);
            rp3d::Quaternion spawn_rot = rp3d::Quaternion::fromEulerAngles(rp3d::Vector3(render_obj.at("initial_rotation")[0],
                                                                                         render_obj.at("initial_rotation")[1],
                                                                                         render_obj.at("initial_rotation")[2]));
            lazyECS::Transform3D spawn_trans(spawn_pos, spawn_rot);
            spawn_trans.SetScale(render_obj.at("scale")[0],
                                 render_obj.at("scale")[1],
                                 render_obj.at("scale")[2]);
            gOrchestrator.AddComponent<lazyECS::Transform3D>(entity, spawn_trans);

            // --------------- Mesh component (For rendering) --------------- //
            lazyECS::Mesh mesh(stringToShape.at(render_obj.at("shape")));
            mesh.mColor = openglframework::Color(0.0,1.0,0.0,1.0);
            gOrchestrator.AddComponent<lazyECS::Mesh>(entity, mesh);               
        }
    }
};

// Static variable initialization
std::unordered_map<std::string, lazyECS::Shape> Spawner::stringToShape = {
        {"Box",lazyECS::Shape::Box},{"Sphere",lazyECS::Shape::Sphere}, {"ConcaveMesh",lazyECS::Shape::ConcaveMesh}, {"Capsule",lazyECS::Shape::Capsule}, {"Hfield",lazyECS::Shape::Hfield}
};

std::unordered_map<std::string, rp3d::BodyType> Spawner::stringToBodyType = {
    {"Dynamic",rp3d::BodyType::DYNAMIC},{"Kinematic",rp3d::BodyType::KINEMATIC}, {"Static",rp3d::BodyType::STATIC}
};  

} // end of lazyECS namespace