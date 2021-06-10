#include "RenderingSystem.hpp"

#include "Components/RigidBody3D.hpp"
#include "Components/Transform3D.hpp"

#include "ECSCore/Orchestrator.hpp"

extern lazyECS::Orchestrator gOrchestrator; // expected to be defined globally in main

namespace lazyECS {

RenderingSystem::RenderingSystem() : mLastMouseX(0), mLastMouseY(0), mViewportX(0), mViewportY(0),
                                     mViewportWidth(0), mViewportHeight(0), 
                                     mIsShadowMappingEnabled(false), mIsShadowMappingInitialized(false){}

void RenderingSystem::Init(const std::string& meshPath) {
    // Set the system signature based on the utilized Components below
    Signature signature;
    signature.set(gOrchestrator.GetComponentTypeId<RigidBody3D>(), true);
    signature.set(gOrchestrator.GetComponentTypeId<Transform3D>(), true);
    signature.set(gOrchestrator.GetComponentTypeId<Mesh>(), true);
    gOrchestrator.SetSystemSignature<RenderingSystem>(signature);

    // Populate the mesh for the entities that have a Mesh component, from a model file
    for(auto& entity : m_entities) {
        auto& mesh = gOrchestrator.GetComponent<Mesh>(entity); // uninitialized here
        openglframework::MeshReaderWriter::loadMeshFromFile(meshPath, mesh); // mesh is initialized here
        if(mesh.getNormals().empty()) { // if the mesh file don't have the normals, calculate them
            mesh.calculateNormals();
        }
    }

    // Create Vertex Buffer object if it hasn't been created yet
}

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

void RenderingSystem::Render() {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    // Render the scene to generate the shadow map (1st pass)
    //  TODO: CUrrently just setting dummy values to shadow related data to make shaders happy
    openglframework::Matrix4 shadowMapProjMatrix[NUM_SHADOW_MAPS];
    openglframework::Matrix4 worldToLightCameraMatrix[NUM_SHADOW_MAPS];
    GLint textureUnits[NUM_SHADOW_MAPS];

    // Render the scene for final rendering (2nd pass)
    glCullFace(GL_BACK);

    // Get the world-space to camera-space matrix
    const openglframework::Matrix4 worldToCameraMatrix = mCamera.getTransformMatrix().getInverse();

    mPhongShader.bind();

    // Set the variables of the phong shader
    mPhongShader.setMatrix4x4Uniform("projectionMatrix", mCamera.getProjectionMatrix());
    mPhongShader.setMatrix4x4Uniform("shadowMapLight0ProjectionMatrix", mShadowMapBiasMatrix * shadowMapProjMatrix[0]);
    mPhongShader.setMatrix4x4Uniform("shadowMapLight1ProjectionMatrix", mShadowMapBiasMatrix * shadowMapProjMatrix[1]);
    mPhongShader.setMatrix4x4Uniform("shadowMapLight2ProjectionMatrix", mShadowMapBiasMatrix * shadowMapProjMatrix[2]);
    mPhongShader.setMatrix4x4Uniform("worldToLight0CameraMatrix", worldToLightCameraMatrix[0]);
    mPhongShader.setMatrix4x4Uniform("worldToLight1CameraMatrix", worldToLightCameraMatrix[1]);
    mPhongShader.setMatrix4x4Uniform("worldToLight2CameraMatrix", worldToLightCameraMatrix[2]);
    mPhongShader.setVector3Uniform("light0PosCameraSpace", worldToCameraMatrix * mLight0.getOrigin());
    mPhongShader.setVector3Uniform("light1PosCameraSpace", worldToCameraMatrix * mLight1.getOrigin());
    mPhongShader.setVector3Uniform("light2PosCameraSpace", worldToCameraMatrix * mLight2.getOrigin());
    mPhongShader.setVector3Uniform("lightAmbientColor", openglframework::Vector3(0.3f, 0.3f, 0.3f));
    mPhongShader.setVector3Uniform("light0DiffuseColor", openglframework::Vector3(mLight0.getDiffuseColor().r, mLight0.getDiffuseColor().g, mLight0.getDiffuseColor().b));
    mPhongShader.setVector3Uniform("light1DiffuseColor", openglframework::Vector3(mLight1.getDiffuseColor().r, mLight1.getDiffuseColor().g, mLight1.getDiffuseColor().b));
    mPhongShader.setVector3Uniform("light2DiffuseColor", openglframework::Vector3(mLight2.getDiffuseColor().r, mLight2.getDiffuseColor().g, mLight2.getDiffuseColor().b));
    mPhongShader.setIntUniform("shadowMapSampler0", textureUnits[0]);
    mPhongShader.setIntUniform("shadowMapSampler1", textureUnits[1]);
    mPhongShader.setIntUniform("shadowMapSampler2", textureUnits[2]);
    mPhongShader.setIntUniform("isShadowEnabled", mIsShadowMappingEnabled);
    mPhongShader.setVector2Uniform("shadowMapDimension", openglframework::Vector2(SHADOWMAP_WIDTH, SHADOWMAP_HEIGHT));

    mPhongShader.unbind();

    // Set variables for the color shader
    mColorShader.bind();
    mColorShader.setMatrix4x4Uniform("projectionMatrix", mCamera.getProjectionMatrix());
    mColorShader.unbind();

    glViewport(mViewportX, mViewportY, mViewportWidth, mViewportHeight); // Set the viewport to render the scene
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE); // enable writing of frame buffer color components (disabled in z-buffer rendering)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear previous frame values

    RenderSinglePass(mPhongShader, worldToCameraMatrix);
    

}

void RenderingSystem::RenderSinglePass(const openglframework::Shader& shader, const openglframework::Matrix4& worldToCamereMatrix) {
    
    shader.bind();

    // For all objects, render
    for (auto const& entity : m_entities) {
        shader.bind();
        // Set the model to camera matrix
        auto& transform = gOrchestrator.GetComponent<Transform3D>(entity);
        shader.setMatrix4x4Uniform("localToWorldMatrix", transform.opengl_transform.getTransformMatrix());
        shader.setMatrix4x4Uniform("worldToCameraMatrix", worldToCamereMatrix);

        // Set the normal matrix (Inverse Transpose of the 3x3 upper-left part of the model-view matrix)
        const openglframework::Matrix4 localToCameraMatrix = worldToCamereMatrix * transform.opengl_transform.getTransformMatrix();
        const openglframework::Matrix3 normalMatrix = localToCameraMatrix.getUpperLeft3x3Matrix().getInverse().getTranspose();
        shader.setMatrix3x3Uniform("normalMatrix", normalMatrix, false);

        // Bind VAO
        mVAO

    }
}


}

