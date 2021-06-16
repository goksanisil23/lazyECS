#ifndef RENDERINGSYSTEM_H
#define RENDERINGSYSTEM_H

#include "ECSCore/Types.hpp"
#include "Components/RigidBody3D.hpp"
#include "Components/Transform3D.hpp"

#include <reactphysics3d/reactphysics3d.h>
#include "openglframework.h"

// #include <nanogui/opengl.h>
#include <nanogui/nanogui.h>
#include <GLFW/glfw3.h>

namespace lazyECS {

class RenderingSystem : public System, public nanogui::Screen { // NanoGUI will initialize and manage the openGL context via Screen

protected:
    // ----------------- Member variables ----------------- //

    static constexpr int NUM_SHADOW_MAPS = 3;
    static constexpr int SHADOWMAP_WIDTH = 2048;
    static constexpr int SHADOWMAP_HEIGHT = 2048;
    
    openglframework::Camera mCamera;

    double mLastMouseX, mLastMouseY; // Last mouse coordinates on the windows
    int mWindowWidth, mWindowHeight; // Window dimensions
    openglframework::Vector3 mLastPointOnSphere; // last point computed on scene sphere (for camera rotation)
    bool mIsLastPointOnSphereValid; // True if the last point computed on the scene sphere is valid
    int mViewportX, mViewportY, mViewportWidth, mViewportHeight; // Viewport x,y, width, height values
    openglframework::Vector3 mSceneCenter; // Center of the scene

    openglframework::Light mLight0;
    openglframework::Light mLight1;
    openglframework::Light mLight2;

    openglframework::Shader mPhongShader; // phong Shader
    openglframework::Shader mColorShader; // constant color shader
    openglframework::Shader mQuadShader;

    bool mIsShadowMappingEnabled; // true if shadow mapping is enabled
    bool mIsShadowMappingInitialized; // true if shadow's FBO and textures have been created

    openglframework::Matrix4 mShadowMapBiasMatrix; // Shadow map bias matrix
    openglframework::Camera mShadowMapLightCameras[NUM_SHADOW_MAPS]; // Cameras at lights position for the shadow maps

    // Shared variables among all renderable entities
    openglframework::VertexBufferObject mVBOVertices; // vertex buffer object for the vertices data
    openglframework::VertexBufferObject mVBONormals; // vertex buffer object for the normals data
    openglframework::VertexBufferObject mVBOTextureCoords; // vertex buffer object for texture coordinates
    openglframework::VertexBufferObject mVBOIndices; //
    openglframework::VertexArrayObject mVAO; // vertex array object for vertex data above
    
    int numRenderables; // total number of renderables created

    std::chrono::_V2::system_clock::time_point prevFrameTime;

    // ----------------- Member functions ----------------- //

public:

    RenderingSystem(bool isFullscreen, int windowWidth, int windowHeight);

    // nanogui::Screen overrides
    virtual void draw_contents() override; // render the contents of the application
    virtual bool mouse_button_event(const nanogui::Vector2i& p, int button, bool down, int modifiers) override;
    virtual bool mouse_motion_event(const nanogui::Vector2i& p, const nanogui::Vector2i& rel, int button, int modifiers) override;
    virtual bool scroll_event(const nanogui::Vector2i& p, const nanogui::Vector2f& rel) override;
    virtual bool resize_event(const nanogui::Vector2i& size) override;
    virtual bool keyboard_event(int key, int scancode, int action, int modifiers) override;
    std::map<GLFWwindow*, nanogui::Screen*>& GetNanoguiScreen();

    void SetupSignature(); // sets the signature of the system based on components its using

    void ResetCameraToViewAll(); // Set the camera so that we can view the whole scene
    // Map mouse coordinates to the coordinates on the scene sphere
    bool MapMouseCoordinatesToSphere(double xMouse, double yMouse, openglframework::Vector3& spherePoint) const;
    void Zoom(float zoomDiff); // Zoom the camera
    void Translate(int xMouse, int yMouse); // Translate the camera
    void Rotate(int xMouse, int yMouse); // Rotate the camera
    void ReshapeCameraView(int width, int height); // reshape the view

    void SetWindowDimension(int width, int height); // set window dimension
    void SetViewport(int x, int y, int width, int height); // Set the viewport to render the scene
    void SetScenePosition(const openglframework::Vector3& position, float sceneRadius); // where main scene camera looks at

    bool KeyboardEvent(int key, int scancode, int action, int mods);  // Called when a keyboard event occurs
    bool MouseButtonEvent(int button, bool down, int mods, double mousePosX, double mousePosY); // called when mouse button event occurs
    bool MouseMotionEvent(double xMouse, double yMouse, int leftButtonState, int rightButtonState, int middleButtonState, int altKeyState);
    bool ScrollingEvent(float xAxis, float yAxis, float scrollSensitivy); // called when scrolling event occurs

    void Init(const std::string& meshPath); // Used to generate the mesh for entities

    void Render(); // Render the scene (possibly in multiple passes due to shadow mapping)

    // render the scene in a single pass
    void RenderSinglePass(const openglframework::Matrix4& worldToCamereMatrix);

    void CreateVBOVAO(Mesh& mesh); // Create VBOs and VAO to render with OpenGL

};

/// Reshape the view
inline void RenderingSystem::ReshapeCameraView(int width, int height) {
    mCamera.setDimensions(width, height);
}

inline void RenderingSystem::SetWindowDimension(int width, int height) {
    mWindowWidth = width;
    mWindowHeight = height;
}

inline void RenderingSystem::SetViewport(int x, int y, int width, int height) {
    mViewportX = x;
    mViewportY = y;
    mViewportWidth = width;
    mViewportHeight = height;
}

} // namespace lazyECS

#endif // RENDERINGSYSTEM_H