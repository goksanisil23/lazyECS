#include "RenderingSystemRaylib.hpp"

#include "Components/Transform3D.hpp"
#include "Components/Mesh.hpp"

#include "ECSCore/Orchestrator.hpp"

#include <iostream>

extern lazyECS::Orchestrator gOrchestrator; // expected to be defined globally in main

namespace lazyECS {

void RenderingSystemRaylib::Init(){

    // Set the system signature based on the utilized Components below
    Signature signature;
    signature.set(gOrchestrator.GetComponentTypeId<Transform3D>(), true);
    signature.set(gOrchestrator.GetComponentTypeId<Mesh>(), true);

    gOrchestrator.SetSystemSignature<RenderingSystemRaylib>(signature);


    // Raylib related initialization
    window = std::make_shared<raylib::Window>(800, 600, "Minimal 3D");
    window->SetTargetFPS(50.0);
    camera = std::make_shared<raylib::Camera3D>(
        raylib::Vector3(100.0f, 100.0f, 100.0f), // position
        raylib::Vector3(0.0f, 0.0f, 0.0f), // target camera looks at
        raylib::Vector3(0.0f, 1.0f, 0.0f), // camera up vector (rotation axis)
        45.0f, // camerea fov
        CAMERA_PERSPECTIVE
    );
    camera->SetMode(CAMERA_FREE);

}

void RenderingSystemRaylib::Update(float dt) {

    camera->Update(); // updates the camera based on cursor and keyboard inputs

    BeginDrawing();
        window->ClearBackground(RAYWHITE);
        camera->BeginMode();

            // Draw the entities in a loop here
            for (auto const& entity : m_entities) {
                Transform3D& ent_trans = gOrchestrator.GetComponent<Transform3D>(entity);
                Mesh& ent_mesh = gOrchestrator.GetComponent<Mesh>(entity);

                raylib::Vector3 cube_extents(ent_mesh.GetMeshParam("halfExtends"));
                ent_trans.GetRaylibTransform().DrawCube(cube_extents, BLUE);

                // ent_trans.position.DrawSphere(2.0f, BLUE);
            }

        camera->EndMode();

        DrawFPS(10, 10);
    EndDrawing();

}

}
