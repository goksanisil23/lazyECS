#include "RenderingSystem.hpp"

#include "ECSCore/Orchestrator.hpp"

extern lazyECS::Orchestrator gOrchestrator; // expected to be defined globally in main

namespace lazyECS {

RenderingSystem::RenderingSystem() : mLastMouseX(0), mLastMouseY(0), mViewportX(0), mViewportY(0),
                                     mViewportWidth(0), mViewportHeight(0){}

void RenderingSystem::SetScenePosition(const openglframework::Vector3& position, float sceneRadius) {
    // Set the position and radius of the scene
    mSceneCenter = position;
    mCamera.setSceneRadius(sceneRadius);
    ResetCameraToViewAll(); // Reset camera position and zoom to view all the scene
}

void RenderingSystem::ResetCameraToViewAll() {
    mCamera.translateWorld(-mCamera.getOrigin()); // move the camera to the origin of the scene (0,0,0)
    mCamera.translateWorld(mSceneCenter); // then move the camera to the center of the scene (user defined)
    mCamera.setZoom(1.0); // set the zoom of the camera so that the scene is in negative view direction of the camera
}

bool RenderingSystem::MapMouseCoordinatesToSphere(double xMouse, double yMouse, openglframework::Vector3& spherePoint) const {
    if ((xMouse >= 0) && (xMouse <= mWindowWidth) && (yMouse >= 0) && (yMouse <= mWindowHeight)) {
        float x = float(xMouse - 0.5f * mWindowWidth) / float(mWindowWidth);
        float y = float(0.5f * mWindowHeight - yMouse) / float(mWindowHeight);
        float sinx = std::sin(openglframework::PI * x * 0.5f);
        float siny = std::sin(openglframework::PI * y * 0.5f);
        float sinx2siny2 = sinx * sinx + siny * siny;

        // Compute the point on the sphere
        spherePoint.x = sinx;
        spherePoint.y = siny;
        spherePoint.z = (sinx2siny2 < 1.0f) ? std::sqrt(1.0f - sinx2siny2) : 0.0f;

        return true;
    }

    return false;    
}

bool RenderingSystem::MouseButtonEvent(int button, bool down, int mods, double mousePosX, double mousePosY) {

    // If the mouse button is pressed
    if (down) {
        mLastMouseX = mousePosX;
        mLastMouseY = mousePosY;
        mIsLastPointOnSphereValid = MapMouseCoordinatesToSphere(mousePosX, mousePosY, mLastPointOnSphere);
    }
    else {  // If the mouse button is released
        mIsLastPointOnSphereValid = false;
    }

    return true;
}

bool RenderingSystem::MouseMotionEvent(double xMouse, double yMouse, int leftButtonState,
                             int rightButtonState, int middleButtonState, int altKeyState) {

    // Zoom
    if (leftButtonState == GLFW_PRESS && altKeyState == GLFW_PRESS) {

        float dy = static_cast<float>(yMouse - mLastMouseY);
        float h = static_cast<float>(mWindowHeight);

        Zoom(-dy / h); // Zoom the camera
    }
    // Translation
    else if (middleButtonState == GLFW_PRESS || rightButtonState == GLFW_PRESS ||
             (leftButtonState == GLFW_PRESS && altKeyState == GLFW_PRESS)) {
        Translate(xMouse, yMouse);
    }
    // Rotation
    else if (leftButtonState == GLFW_PRESS) {
        Rotate(xMouse, yMouse);
    }

    // Remember the mouse position
    mLastMouseX = xMouse;
    mLastMouseY = yMouse;
    mIsLastPointOnSphereValid = MapMouseCoordinatesToSphere(xMouse, yMouse, mLastPointOnSphere);

    return true;
}

bool RenderingSystem::ScrollingEvent(float xAxis, float yAxis, float scrollSensitivy) {
    Zoom(yAxis * scrollSensitivy);
    return true;
}

void RenderingSystem::Zoom(float zoomDiff) {
    mCamera.setZoom(zoomDiff);  // Zoom the camera
}

void RenderingSystem::Translate(int xMouse, int yMouse) {
   float dx = static_cast<float>(xMouse - mLastMouseX);
   float dy = static_cast<float>(yMouse - mLastMouseY);

   // Translate the camera
   mCamera.translateCamera(-dx / float(mCamera.getWidth()),
                           -dy / float(mCamera.getHeight()), mSceneCenter);
}

void RenderingSystem::Rotate(int xMouse, int yMouse) {
    if (mIsLastPointOnSphereValid) {

        openglframework::Vector3 newPoint3D;
        bool isNewPointOK = MapMouseCoordinatesToSphere(xMouse, yMouse, newPoint3D);

        if (isNewPointOK) {
            openglframework::Vector3 axis = mLastPointOnSphere.cross(newPoint3D);
            float cosAngle = mLastPointOnSphere.dot(newPoint3D);

            float epsilon = std::numeric_limits<float>::epsilon();
            if (std::abs(cosAngle) < 1.0f && axis.length() > epsilon) {
                axis.normalize();
                float angle = 2.0f * std::acos(cosAngle);

                // Rotate the camera around the center of the scene
                mCamera.rotateAroundLocalPoint(axis, -angle, mSceneCenter);
            }
        }
    }
}


}

