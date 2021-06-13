#include "Minimal3D.h"

Minimal3D::Minimal3D(bool isFullscreen, int windowWidth, int windowHeight) :
    nanogui::Screen(nanogui::Vector2i(windowWidth, windowHeight), "Minimal 3D", true, isFullscreen, true, true, false, 4, 1) 
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
    renderSys = gOrchestrator.RegisterSystem<lazyECS::RenderingSystem>(); // constructs the System

    renderSys->SetupSignature(); // entities below will be matched with systems so signature of the system is set

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
        spawn_trans.ConvertRP3DToOpenglTransform(0.9);
        
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

}


void Minimal3D::draw_contents(){
    
    int bufferWidth, bufferHeight;
    glfwMakeContextCurrent(m_glfw_window);
    glfwGetFramebufferSize(m_glfw_window, &bufferWidth, &bufferHeight);
    renderSys->SetViewport(0, 0, bufferWidth, bufferHeight);

    renderSys->Render();
    // renderSys->Render2();
}

bool Minimal3D::mouse_button_event(const nanogui::Vector2i& p, int button, bool down, int modifiers) {
    std::cout << "Mouse Button Event" << std::endl;
    if(Screen::mouse_button_event(p, button, down, modifiers))
        return true;
    
    double x, y;
    glfwGetCursorPos(m_glfw_window, &x, &y);

    return renderSys->MouseButtonEvent(button, down, modifiers, x, y);
}

bool Minimal3D::mouse_motion_event(const nanogui::Vector2i& p, const nanogui::Vector2i& rel, int button, int modifiers) {
    std::cout << "Mouse Motion Event" << std::endl;
    if(Screen::mouse_motion_event(p, rel, button, modifiers))
        return true;

    int leftButtonState = glfwGetMouseButton(m_glfw_window, GLFW_MOUSE_BUTTON_LEFT);
    int rightButtonState = glfwGetMouseButton(m_glfw_window, GLFW_MOUSE_BUTTON_RIGHT);
    int middleButtonState = glfwGetMouseButton(m_glfw_window, GLFW_MOUSE_BUTTON_MIDDLE);
    int altKeyState = glfwGetKey(m_glfw_window, GLFW_KEY_LEFT_ALT);        

    return renderSys->MouseMotionEvent(p[0], p[1], leftButtonState, rightButtonState, middleButtonState, altKeyState);
}

bool Minimal3D::scroll_event(const nanogui::Vector2i& p, const nanogui::Vector2f& rel) {
    std::cout << "Scroll Event" << std::endl;

    if(Screen::scroll_event(p, rel))
        return true;
    
    return renderSys->ScrollingEvent(rel[0], rel[1], 0.08f);
}