#ifndef RENDERINGSYSTEM_H
#define RENDERINGSYSTEM_H

#include "ECSCore/Types.hpp"
#include "Components/RigidBody3D.hpp"
#include "Components/Transform3D.hpp"

#include <cstdint>
#include <reactphysics3d/collision/shapes/AABB.h>
#include <reactphysics3d/mathematics/Transform.h>
#include <reactphysics3d/mathematics/Vector3.h>
#include <reactphysics3d/reactphysics3d.h>
#include "VertexBufferObject.h"
#include "nanogui/label.h"
#include "openglframework.h"

// #include <nanogui/opengl.h>
#include <nanogui/nanogui.h>
#include <GLFW/glfw3.h>
#include <reactphysics3d/utils/DebugRenderer.h>
#include <unordered_map>
#include <unordered_set>
#include <thread>
#include <vector>


namespace lazyECS {

class RenderingSystem : public System, public nanogui::Screen { // NanoGUI will initialize and manage the openGL context via Screen

public:

    // Triangle structure used for debug rendering of primitives
    struct DebugTriangle {
        DebugTriangle(const rp3d::Vector3& pt1, const rp3d::Vector3& pt2, const rp3d::Vector3& pt3, std::uint32_t color)
            : point1(pt1), color1(color), point2(pt2), color2(color), point3(pt3), color3(color) {}

        rp3d::Vector3 point1; // first point on the triangle
        std::uint32_t color1; // color of 1st point
        rp3d::Vector3 point2;
        std::uint32_t color2;
        rp3d::Vector3 point3;
        std::uint32_t color3;
    };

    // Line structure used for debug rendering of lines
    struct DebugLine {
        DebugLine(const rp3d::Vector3& pt1, const rp3d::Vector3& pt2, std::uint32_t color):
            point1(pt1), color1(color), point2(pt2), color2(color) {}

        rp3d::Vector3 point1;
        std::uint32_t color1;
        rp3d::Vector3 point2;
        std::uint32_t color2;        
    };

protected:
    // ----------------- Member variables ----------------- //

    std::chrono::microseconds quantum; // sleep time in render timer thread

    static constexpr float RENDER_TIME_STEP = 0.02; // (sec. = 1/50 FPS)

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
    openglframework::Shader mDepthShader; // depth shader to render the shadow map

    openglframework::Texture2D mShadowMapTexture[NUM_SHADOW_MAPS]; // shadow map is the depth texture rendered from the light's perspective
    openglframework::FrameBufferObject mFBOShadowMap[NUM_SHADOW_MAPS]; // FBOs for rendering the depth maps per light source

    bool mIsShadowMappingEnabled; // true if shadow mapping is enabled
    bool mIsShadowMappingInitialized; // true if shadow's FBO and textures have been created

    openglframework::Matrix4 mShadowMapBiasMatrix; // Shadow map bias matrix
    openglframework::Camera mShadowMapLightCameras[NUM_SHADOW_MAPS]; // Cameras at lights position for the shadow maps

    // We need a separate set of VBO and VAO per each unique mesh = Shape (not Mesh instance) -> Shape is enum, for iteration convenience map is created with int key
    std::unordered_map<int, openglframework::VertexBufferObject> mVBOVertices;
    std::unordered_map<int, openglframework::VertexBufferObject> mVBONormals;
    std::unordered_map<int, openglframework::VertexBufferObject> mVBOTextureCoords;
    std::unordered_map<int, openglframework::VertexBufferObject> mVBOIndices;
    std::unordered_map<int, openglframework::VertexArrayObject> mVAO;

    // VBO and VAO for debug meshes
    openglframework::VertexBufferObject mDebugVBOTrianglesVertices; // triangle vertex data for debug objects
    openglframework::VertexArrayObject mDebugTrianglesVAO; // vertex array object for debug triangle vertices
    openglframework::VertexBufferObject mDebugVBOLinesVertices; // line vertex data for debug objects
    openglframework::VertexArrayObject mDebugLinesVAO; // vertex array object for debug line vertices
    std::vector<DebugTriangle> mDebugTriangles;
    std::vector<DebugLine> mDebugLines;
    bool mIsDebugRenderingEnabled;

    std::unordered_set<Shape> bufferedShapes; // storing shapes for which VBO and VAO are already created

    static constexpr int NB_SECTORS_SPHERE = 10; // Number of sectors used to draw a sphere or a capsule
    static constexpr int NB_STACKS_SPHERE = 4;  // Number of stacks used to draw a sphere or a capsule

    // ----------------- Member functions ----------------- //

public:

    RenderingSystem(bool isFullscreen, int windowWidth, int windowHeight, const std::string& appName);

    // nanogui::Screen overrides
    virtual void draw_contents() override; // render the contents of the application
    virtual bool mouse_button_event(const nanogui::Vector2i& p, int button, bool down, int modifiers) override;
    virtual bool mouse_motion_event(const nanogui::Vector2i& p, const nanogui::Vector2i& rel, int button, int modifiers) override;
    virtual bool scroll_event(const nanogui::Vector2i& p, const nanogui::Vector2f& rel) override;
    virtual bool resize_event(const nanogui::Vector2i& size) override;
    virtual bool keyboard_event(int key, int scancode, int action, int modifiers) override;
    std::map<GLFWwindow*, nanogui::Screen*>& GetNanoguiScreen();

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

    void Init(); // Used to generate the mesh for entities
    void Render(); // Render the scene (possibly in multiple passes due to shadow mapping)
    void RenderSinglePass(openglframework::Shader& shader, const openglframework::Matrix4& worldToCameraMatrix); // render the scene in a single pass
    void RenderDebugObjects(openglframework::Shader& shader, const openglframework::Matrix4& worldToCameraMatrix); // renders debug objects
    void SetupSignature(); // sets the signature of the system based on components its using
    void Update(); // highest level of function called from the main loop of the application, that calls the rest of the rendering pipeline

    void CreateVBOVAO(Mesh& mesh); // Create VBOs and VAO to render with OpenGL
    void CreateShadowMapFBOAndTexture(); // create shadow map frame buffer object and texture
    void CreateDebugVBOVAO(); // create VBO for debug only objects
    void UpdateDebugVBOVAO(); // update vertices and indices for debug objects
    void DrawDebugBox(const rp3d::Transform& transform, const rp3d::Vector3& halfExtents, uint32_t color);
    void DrawDebugAABB(const rp3d::Transform& transform, const rp3d::Vector3& min_local, const rp3d::Vector3& max_local, uint32_t color);
    void DrawDebugSphere(const rp3d::Vector3& position, const float& radius, uint32_t color);
    void DrawDebugRectangle(const rp3d::Transform& transform, const rp3d::Vector3& halfExtents, uint32_t color);
    void DrawDebugArrow(const rp3d::Transform& transform, uint32_t color);

    void TimerThreadFunc(); // function used in a thread to make periodic draw calls
    void GuiInit(); // Initialize Gui panels & widgets
    void UpdateGui() const; // update Gui information

    // ----------------- Member variables ----------------- //
    // std::shared_ptr<nanogui::Label> fpsLabel_;
    nanogui::Label* fpsLabel_;
    std::chrono::_V2::system_clock::time_point prevFrameTime;
    float deltaTime;     

    struct DebugBox {
        rp3d::Transform transform;
        rp3d::Vector3 halfExtents;
        rp3d::DebugRenderer::DebugColor color;

        DebugBox(const rp3d::Transform& trans, const rp3d::Vector3& halfExt, const rp3d::DebugRenderer::DebugColor& col)
            : transform(trans), halfExtents(halfExt), color(col) {}
    };

    struct DebugAABB {
        rp3d::Transform transform;
        rp3d::Vector3 min, max;
        rp3d::DebugRenderer::DebugColor color;

        DebugAABB(const rp3d::Transform& trans, const rp3d::Vector3& min, const rp3d::Vector3& max, const rp3d::DebugRenderer::DebugColor& col)
            : transform(trans), min(min), max(max), color(col) {}        
    };

    struct DebugSphere {
        rp3d::Vector3 position;
        float radius;
        rp3d::DebugRenderer::DebugColor color;

        DebugSphere(const rp3d::Vector3& pos, const float& rad, const rp3d::DebugRenderer::DebugColor& col)
            : position(pos), radius(rad), color(col) {}  
    };

    struct DebugRectangle {
        rp3d::Transform transform;
        rp3d::Vector3 halfExtents;
        uint32_t color;

        DebugRectangle(const rp3d::Transform& trans, const rp3d::Vector3& halfExt, uint32_t& col)
            : transform(trans), halfExtents(halfExt), color(col) {}
    };

    struct DebugArrow {
        rp3d::Transform transform;
        rp3d::DebugRenderer::DebugColor color;

        DebugArrow(const rp3d::Transform& trans, const rp3d::DebugRenderer::DebugColor& col)
            : transform(trans), color(col) {}
    };

    std::vector<DebugBox> mDebugBoxes; // debug boxes to be populated by the user
    std::vector<DebugSphere> mDebugSpheres; // debug spheres to be populated by the user
    std::vector<DebugAABB> mDebugAABBs; // debug AABBs to be populated by the user
    std::vector<DebugRectangle> mDebugRectangles; // debug rectangles to be populated by the user
    std::vector<DebugArrow> mDebugArrows; // debug rectangles to be populated by the user
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