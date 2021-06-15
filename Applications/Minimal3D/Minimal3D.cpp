#include "Minimal3D.h"

Minimal3D::Minimal3D(bool isFullscreen, int windowWidth, int windowHeight)
{
    
    // --------------- lAZYECS -------------- //

    gOrchestrator.Init(); // initializes the ECS managers

    // Register the components
    gOrchestrator.RegisterComponentType<lazyECS::Transform3D>();
    gOrchestrator.RegisterComponentType<lazyECS::RigidBody3D>();
    gOrchestrator.RegisterComponentType<lazyECS::Mesh>();

    // Register the systems
    physicsSys = gOrchestrator.RegisterSystem<lazyECS::PhysicsSystem>();
    // Rendering system is registered and initialized here since it has dependency on OpenGL context to be initialized first
    // which is happening during nanogui::Screen initialization
    renderSys = gOrchestrator.RegisterSystem<lazyECS::RenderingSystem>(isFullscreen, windowWidth, windowHeight); // constructs the System

    // renderSys->SetupSignature(); // entities below will be matched with systems so signature of the system is set

    // Create the entities and assign components
    std::vector<lazyECS::Entity> entities(10);
    float boxSize[3] = {2.0, 2.0, 2.0};                   
    const int box_x_pos = 3.0;

    for (int i = 0; i < entities.size(); i++) {
        auto entity = entities.at(i);
        entity = gOrchestrator.CreateEntity();

        // Transform component (Initial position)
        reactphysics3d::Vector3 spawn_pos(box_x_pos + i*5.0, box_x_pos + i*5.0, box_x_pos + i*5.0);
        reactphysics3d::Quaternion spawn_rot(reactphysics3d::Quaternion::identity());
        lazyECS::Transform3D spawn_trans;
        spawn_trans.rp3d_transform = reactphysics3d::Transform(spawn_pos, spawn_rot);
        spawn_trans.mScalingMatrix = openglframework::Matrix4(  boxSize[0]*0.5f, 0, 0, 0,
                                                                0, boxSize[1]*0.5f, 0, 0,
                                                                0, 0, boxSize[2]*0.5f, 0,
                                                                0, 0, 0, 1);
        spawn_trans.ConvertRP3DToOpenglTransform(0.9); // temporary interpolation factor
        
        gOrchestrator.AddComponent<lazyECS::Transform3D>(entity,spawn_trans);


        // ADD MESH COMPONENT!!!!!!!!!!!!!!!!
        lazyECS::Mesh mesh;
        gOrchestrator.AddComponent<lazyECS::Mesh>(entity, mesh);

        // Rigid Body component for physical motion
        // lazyECS::RigidBody3D rigid_body; // will be initialized in PhysicsSystem
        // gOrchestrator.AddComponent<lazyECS::RigidBody3D>(entity, rigid_body);
    }

    // Init requires the entities to be initialized and entities requires system's signature to be set
    renderSys->Init("/home/goksan/Work/lazyECS/Applications/meshes/cube.obj");
    
    renderSys->set_visible(true); // nanogui set visibility
}
