#ifndef RENDERINGSYSTEM_H
#define RENDERINGSYSTEM_H

#include "ECSCore/Types.hpp"

#include <reactphysics3d/reactphysics3d.h>
#include "openglframework.h"

namespace lazyECS {

class RenderingSystem {

protected:
    // ----------------- Member variables ----------------- //
    
    openglframework::Camera mCamera;

    double mLastMouseX, mLastMouseY; // Last mouse coordinates on the windows
    int mWindowWidth, mWindowHeight; // Window dimensions
    openglframework::Vector3 mLastPointOnSphere; // last point computed on scene sphere (for camera rotation)
    bool mIsLastPointOnSphereValid; // True if the last point computed on the scene sphere is valid
    int mViewportX, mViewportY, mViewportWidth, mViewportHeight; // Viewport x,y, width, height values
    openglframework::Vector3 mSceneCenter; // Center of the scene

    // ----------------- Member functions ----------------- //

public:

    RenderingSystem();

    void ResetCameraToViewAll(); // Set the camera so that we can view the whole scene

    // Map mouse coordinates to the coordinates on the scene sphere
    bool MapMouseCoordinatesToSphere(double xMouse, double yMouse, openglframework::Vector3& spherePoint) const;

    void Zoom(float zoomDiff); // Zoom the camera
    void Translate(int xMouse, int yMouse); // Translate the camera
    void Rotate(int xMouse, int yMouse); // Rotate the camera

    void Reshape(int width, int height); // reshape the view

    void SetWindowDimension(int width, int height); // set window dimension

    void SetViewport(int x, int y, int width, int height); // Set the viewport to render the scene

    void SetScenePosition(const openglframework::Vector3& position, float sceneRadius); // where main scene camera looks at

    virtual bool KeyboardEvent(int key, int scancode, int action, int mods);  // Called when a keyboard event occurs

    virtual bool MouseButtonEvent(int button, bool down, int mods, double mousePosX, double mousePosY); // called when mouse button event occurs

    virtual bool MouseMotionEvent(double xMouse, double yMouse, int leftButtonState, int rightButtonState, int middleButtonState, int altKeyState);

    virtual bool ScrollingEvent(float xAxis, float yAxis, float scrollSensitivy); // called when scrolling event occurs    

};

/// Reshape the view
inline void RenderingSystem::Reshape(int width, int height) {
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