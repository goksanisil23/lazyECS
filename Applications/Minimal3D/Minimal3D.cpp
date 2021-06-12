#include "Minimal3D.h"

// Minimal3D::Minimal3D(bool isFullscreen, int windowWidth, int windowHeight, 
//                      std::shared_ptr<lazyECS::RenderingSystem> renderSys_ptr) :
Minimal3D::Minimal3D(bool isFullscreen, int windowWidth, int windowHeight) :
    nanogui::Screen(nanogui::Vector2i(windowWidth, windowHeight), "Minimal 3D", true, isFullscreen, true, true, false, 4, 1) 
{
    // renderSys = renderSys_ptr;
    renderSys = gOrchestrator.RegisterSystem<lazyECS::RenderingSystem>();

    renderSys->Init("/home/goksan/Work/lazyECS/Applications/meshes/cube.obj");
}


void Minimal3D::draw_contents() {
    

    int bufferWidth, bufferHeight;
    glfwMakeContextCurrent(m_glfw_window);
    glfwGetFramebufferSize(m_glfw_window, &bufferWidth, &bufferHeight);
    renderSys->SetViewport(0, 0, bufferWidth, bufferHeight);

    renderSys->Render();
}