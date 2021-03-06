#include "RenderingSystem.hpp"

#include "ECSCore/Orchestrator.hpp"
#include "nanogui/label.h"
#include "nanogui/screen.h"
#include "nanogui/widget.h"
#include <GL/gl.h>
#include <chrono>
#include <cstdint>
#include <memory>
#include <reactphysics3d/mathematics/Vector3.h>
#include <reactphysics3d/utils/DebugRenderer.h>
#include <string>
#include <thread>

extern lazyECS::Orchestrator gOrchestrator; // expected to be defined globally in main
extern json launch_obj;

namespace lazyECS {

RenderingSystem::RenderingSystem(bool isFullscreen, int windowWidth, int windowHeight, const std::string& appName) :
    // Its important to have nanogui::Screen initialized first since it's creating openGL context for the rest of the Opengl variables in the render System
                nanogui::Screen{nanogui::Vector2i(windowWidth, windowHeight), appName, true, isFullscreen, true, true, false, 4, 1},
                mLastMouseX{0}, mLastMouseY{0}, mViewportX{0}, mViewportY{0},
                mViewportWidth{0}, mViewportHeight{0}, 
                mPhongShader{"../../shaders/phong.vert",
                            "../../shaders/phong.frag"},
                mColorShader{"../../shaders/color.vert",
                            "../../shaders/color.frag"},
                mDepthShader{"../../shaders/depth.vert",
                            "../../shaders/depth.frag"},
                mIsShadowMappingEnabled{true}, mIsShadowMappingInitialized{false},                            
                mDebugVBOTrianglesVertices{GL_ARRAY_BUFFER},
                mDebugVBOLinesVertices{GL_ARRAY_BUFFER},
                mIsDebugRenderingEnabled{true},
                fpsLabel_(nullptr),
                prevFrameTime{std::chrono::high_resolution_clock::now()}
{
    // Allocate 1 buffer object per available shapes in LazyECS
    for(int _shape = Shape::Box; _shape < Shape::Last; _shape++) {
        mVBOVertices.emplace(std::make_pair(_shape, GL_ARRAY_BUFFER));
        mVBONormals.emplace(std::make_pair(_shape, GL_ARRAY_BUFFER)); 
        mVBOTextureCoords.emplace(std::make_pair(_shape, GL_ARRAY_BUFFER));
        mVBOIndices.emplace(std::make_pair(_shape, GL_ELEMENT_ARRAY_BUFFER));
        mVAO.emplace(std::make_pair(_shape, openglframework::VertexArrayObject()));         
    }

    quantum = std::chrono::microseconds((int64_t)(RENDER_TIME_STEP*1'000'000));  // sleep time for the render timer thread

    // ----------------- Scene Lighting and View Setup ----------------- // 

    float lightsRadius = 30.0f;
    float lightsHeight = 20.0f;
    mLight0.translateWorld(openglframework::Vector3(0*lightsRadius, lightsHeight, 1*lightsRadius));
    mLight1.translateWorld(openglframework::Vector3(0.95f*lightsRadius, lightsHeight, -0.3f*lightsRadius));
    mLight2.translateWorld(openglframework::Vector3(-0.58f*lightsRadius, lightsHeight, -0.81f*lightsRadius));

    // Set the lights colors
	mLight0.setDiffuseColor(openglframework::Color(0.6f, 0.6f, 0.6f, 1.0f));
	mLight1.setDiffuseColor(openglframework::Color(0.6f, 0.6f, 0.6f, 1.0f));
	mLight2.setDiffuseColor(openglframework::Color(0.6f, 0.6f, 0.6f, 1.0f));

    const float PI = 3.141592654f;
	mShadowMapLightCameras[0].translateWorld(mLight0.getOrigin());
	mShadowMapLightCameras[0].rotateLocal(openglframework::Vector3(1, 0, 0), -PI / 4.0f);

	mShadowMapLightCameras[1].translateWorld(mLight1.getOrigin());
	mShadowMapLightCameras[1].rotateLocal(openglframework::Vector3(0, 1, 0), -5.0f * PI/3.7f);
	mShadowMapLightCameras[1].rotateLocal(openglframework::Vector3(1, 0, 0), -PI/4.0f);

	mShadowMapLightCameras[2].translateWorld(mLight2.getOrigin());
	mShadowMapLightCameras[2].rotateLocal(openglframework::Vector3(0, 1, 0), 5 * PI/4.0f);
	mShadowMapLightCameras[2].rotateLocal(openglframework::Vector3(1, 0 , 0), -PI/4.0f);

    for(int i = 0; i < NUM_SHADOW_MAPS; i++) {
        mShadowMapLightCameras[i].setDimensions(SHADOWMAP_WIDTH, SHADOWMAP_HEIGHT);
        mShadowMapLightCameras[i].setFieldOfView(100.0f);
        mShadowMapLightCameras[i].setSceneRadius(100);
    }

    mShadowMapBiasMatrix.setAllValues(0.5, 0.0, 0.0, 0.5,
                                      0.0, 0.5, 0.0, 0.5,
                                      0.0, 0.0, 0.5, 0.5,
                                      0.0, 0.0, 0.0, 1.0);  

    if(mIsShadowMappingEnabled)
        CreateShadowMapFBOAndTexture();

    // Setup where camera looks at 
    json camera_json =  launch_obj.at("scene").at("camera");
    openglframework::Vector3 center(camera_json.at("center").at("x"),camera_json.at("center").at("y"),camera_json.at("center").at("z"));
    const float scene_radius = camera_json.at("scene_radius");    
    SetScenePosition(center, scene_radius);
    mCamera.rotateLocal(openglframework::Vector3(1, 0, 0), static_cast<float>(camera_json.at("camera_pitch")) * PI/180.0F);
}

void RenderingSystem::Init() {

    // Populate the mesh for the entities that have a Mesh component, from a model file
    for(auto& entity : m_entities) {
        auto& transform = gOrchestrator.GetComponent<Transform3D>(entity); // initialized outsize
        auto& mesh = gOrchestrator.GetComponent<Mesh>(entity); // uninitialized here, only contains the mesh path given by the user
        if(mesh.mShape != Shape::Hfield)
            openglframework::MeshReaderWriter::loadMeshFromFile(mesh.meshPath, mesh); // mesh is initialized here
        else
            openglframework::MeshReaderWriter::generateHeightFieldMesh(*(mesh.mHeightField), mesh);
            
        if(mesh.getNormals().empty()) { // if the mesh file don't have the normals, calculate them
            mesh.calculateNormals();
        }
        
        // Create Vertex Buffer object if it hasn't been created yet (we do it here since we need vertex info from the Mesh)
        if(bufferedShapes.find(mesh.mShape) == bufferedShapes.end()) { // if this shape hasn't been buffered yet
            CreateVBOVAO(mesh);
            bufferedShapes.insert(mesh.mShape);
        }
        
        // Set the scaling
        transform.opengl_transform.setTransformMatrix(transform.opengl_transform.getTransformMatrix() * transform.mScalingMatrix);        
    }

    if(mIsDebugRenderingEnabled)
        CreateDebugVBOVAO();

    // Set window and camera size
    int bufferWidth, bufferHeight;
    glfwGetFramebufferSize(m_glfw_window, &bufferWidth, &bufferHeight);
    this->ReshapeCameraView(bufferWidth, bufferHeight);
    int windowWidth, windowHeight;
    glfwGetWindowSize(m_glfw_window, &windowWidth, &windowHeight);
    this->SetWindowDimension(windowWidth, windowHeight); 

    this->set_visible(true); // nanogui set visibility

    GuiInit();
}

void RenderingSystem::SetupSignature() {
    // Set the system signature based on the utilized Components below
    Signature signature;
    signature.set(gOrchestrator.GetComponentTypeId<Transform3D>(), true);
    signature.set(gOrchestrator.GetComponentTypeId<Mesh>(), true);
    gOrchestrator.SetSystemSignature<RenderingSystem>(signature);     
}

// ----------------- nanogui::Screen overrides --------------- //
void RenderingSystem::draw_contents(){
    
    int bufferWidth, bufferHeight;
    glfwMakeContextCurrent(m_glfw_window);
    glfwGetFramebufferSize(m_glfw_window, &bufferWidth, &bufferHeight);
    this->SetViewport(0, 0, bufferWidth, bufferHeight);

    this->Render();

    // measure time diff
    auto current_frame_time = std::chrono::high_resolution_clock::now();
    this->deltaTime = std::chrono::duration<float, std::chrono::seconds::period>(current_frame_time-prevFrameTime).count();
    this->prevFrameTime = current_frame_time; // update previous time
    UpdateGui();
}

std::map<GLFWwindow*, nanogui::Screen*>& RenderingSystem::GetNanoguiScreen() {
    return nanogui::Screen::GetScreen();
}

bool RenderingSystem::mouse_button_event(const nanogui::Vector2i& p, int button, bool down, int modifiers) {
    if(Screen::mouse_button_event(p, button, down, modifiers))
        return true;
    
    double x, y;
    glfwGetCursorPos(m_glfw_window, &x, &y);

    return this->MouseButtonEvent(button, down, modifiers, x, y);
}

bool RenderingSystem::mouse_motion_event(const nanogui::Vector2i& p, const nanogui::Vector2i& rel, int button, int modifiers) {
    if(Screen::mouse_motion_event(p, rel, button, modifiers))
        return true;

    int leftButtonState = glfwGetMouseButton(m_glfw_window, GLFW_MOUSE_BUTTON_LEFT);
    int rightButtonState = glfwGetMouseButton(m_glfw_window, GLFW_MOUSE_BUTTON_RIGHT);
    int middleButtonState = glfwGetMouseButton(m_glfw_window, GLFW_MOUSE_BUTTON_MIDDLE);
    int altKeyState = glfwGetKey(m_glfw_window, GLFW_KEY_LEFT_ALT);        

    return this->MouseMotionEvent(p[0], p[1], leftButtonState, rightButtonState, middleButtonState, altKeyState);
}

bool RenderingSystem::scroll_event(const nanogui::Vector2i& p, const nanogui::Vector2f& rel) {

    if(Screen::scroll_event(p, rel))
        return true;
    
    return this->ScrollingEvent(rel[0], rel[1], 0.08f);
}

bool RenderingSystem::resize_event(const nanogui::Vector2i& size) {
    (void)size;

    int width, height;
    glfwGetFramebufferSize(m_glfw_window, &width, &height); // Get the framebuffer dimension
    this->ReshapeCameraView(width, height); // Resize the camera viewport

    int windowWidth, windowHeight;
    glfwGetWindowSize(m_glfw_window, &windowWidth, &windowHeight); // Update the window size of the scene
    this->SetWindowDimension(windowWidth, windowHeight);

    return true;
}

bool RenderingSystem::keyboard_event(int key, int scancode, int action, int modifiers) {
    if(Screen::keyboard_event(key, scancode, action, modifiers)) {
        return true;
    }
    // Close app on Esc key
    if(key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(m_glfw_window, GL_TRUE);
        return true;
    }

    if(key == GLFW_KEY_W && (action == GLFW_REPEAT || action == GLFW_PRESS)) {
        // mCamera.setZoom(-0.1);
        gOrchestrator.event_bus_->publish<KeyboardEvent>(KeyboardEvent(KeyboardEvent::KeyButton::KEY_W));
        return true;
    }
    if(key == GLFW_KEY_S && (action == GLFW_REPEAT || action == GLFW_PRESS)) {
        // mCamera.setZoom(0.1);
        gOrchestrator.event_bus_->publish<KeyboardEvent>(KeyboardEvent(KeyboardEvent::KeyButton::KEY_S));
        return true;
    }
    if(key == GLFW_KEY_A && (action == GLFW_REPEAT || action == GLFW_PRESS)) {
        // mCamera.translateCamera(0.01, 0, mSceneCenter);
        gOrchestrator.event_bus_->publish<KeyboardEvent>(KeyboardEvent(KeyboardEvent::KeyButton::KEY_A));
        return true;
    }   
    if(key == GLFW_KEY_D && (action == GLFW_REPEAT || action == GLFW_PRESS)) {
        // mCamera.translateCamera(-0.01, 0, mSceneCenter);
        gOrchestrator.event_bus_->publish<KeyboardEvent>(KeyboardEvent(KeyboardEvent::KeyButton::KEY_D));
        return true;
    }
    return false;    
}
 // ----------------- end of Nanogui overrides --------------- //

void RenderingSystem::CreateVBOVAO(Mesh& mesh) {
    // VBO for the vertices data
    mVBOVertices.at(mesh.mShape).create();
    mVBOVertices.at(mesh.mShape).bind();
    size_t sizeVertices = mesh.getVertices().size() * sizeof(openglframework::Vector3);
    mVBOVertices.at(mesh.mShape).copyDataIntoVBO(sizeVertices, mesh.getVerticesPointer(), GL_STATIC_DRAW);
    mVBOVertices.at(mesh.mShape).unbind();

    // Create the VBO for the normals data
    mVBONormals.at(mesh.mShape).create();
    mVBONormals.at(mesh.mShape).bind();
    size_t sizeNormals = mesh.getNormals().size() * sizeof(openglframework::Vector3);
    mVBONormals.at(mesh.mShape).copyDataIntoVBO(sizeNormals, mesh.getNormalsPointer(), GL_STATIC_DRAW);
    mVBONormals.at(mesh.mShape).unbind();

    // Create VBO for the texture coordinates
    if(mesh.hasTexture()) {
        mVBOTextureCoords.at(mesh.mShape).create();
        mVBOTextureCoords.at(mesh.mShape).bind();
        size_t sizeTextureCoords = mesh.getUVs().size() * sizeof(openglframework::Vector2);
        mVBOTextureCoords.at(mesh.mShape).copyDataIntoVBO(sizeTextureCoords, mesh.getUVTextureCoordinatesPointer(), GL_STATIC_DRAW);
        mVBOTextureCoords.at(mesh.mShape).unbind();
    }

    // Create VBO for the indices data
    mVBOIndices.at(mesh.mShape).create();
    mVBOIndices.at(mesh.mShape).bind();
    size_t sizeIndices = mesh.getIndices().size() * sizeof(unsigned int);
    mVBOIndices.at(mesh.mShape).copyDataIntoVBO(sizeIndices, mesh.getIndicesPointer(), GL_STATIC_DRAW);
    mVBOIndices.at(mesh.mShape).unbind();

    // Create VAO and bind all the VBOs
    mVAO.at(mesh.mShape).create();
    mVAO.at(mesh.mShape).bind();

    mVBOVertices.at(mesh.mShape).bind();
    mVBONormals.at(mesh.mShape).bind();
    if (mesh.hasTexture()) mVBOTextureCoords.at(mesh.mShape).bind();
    mVBOIndices.at(mesh.mShape).bind();

    mVAO.at(mesh.mShape).unbind();
}

void RenderingSystem::CreateDebugVBOVAO() {
    // Triangles
    mDebugVBOTrianglesVertices.create();
    mDebugTrianglesVAO.create();

    mDebugTrianglesVAO.bind();
    mDebugVBOTrianglesVertices.bind();

    mDebugTrianglesVAO.unbind();
    mDebugVBOTrianglesVertices.unbind();

    // Lines
    mDebugVBOLinesVertices.create();
    mDebugLinesVAO.create();

    mDebugLinesVAO.bind();
    mDebugVBOLinesVertices.bind();

    mDebugLinesVAO.unbind();
    mDebugVBOLinesVertices.unbind();

}

void RenderingSystem::UpdateDebugVBOVAO() {
    if(mDebugTriangles.size() > 0) {
        mDebugVBOTrianglesVertices.bind();
        GLsizei sizeVertices = static_cast<GLsizei>(mDebugTriangles.size() * sizeof(DebugTriangle));
        mDebugVBOTrianglesVertices.copyDataIntoVBO(sizeVertices, &(mDebugTriangles[0]), GL_STREAM_DRAW); // stream since we plan to both modify and draw 
        mDebugVBOTrianglesVertices.unbind();
    }

    if(mDebugLines.size() > 0) {
        mDebugVBOLinesVertices.bind();
        GLsizei sizeVertices = static_cast<GLsizei>(mDebugLines.size() * sizeof(DebugLine));
        mDebugVBOLinesVertices.copyDataIntoVBO(sizeVertices, &(mDebugLines[0]), GL_STREAM_DRAW); // stream since we plan to both modify and draw 
        mDebugVBOLinesVertices.unbind();        
    }
}

void RenderingSystem::CreateShadowMapFBOAndTexture() {
    for (int i = 0; i < NUM_SHADOW_MAPS; i++) {
        // Create texture for depth values (depth value per pixel coordinate)
        mShadowMapTexture[i].create(SHADOWMAP_WIDTH, SHADOWMAP_HEIGHT, GL_DEPTH_COMPONENT24, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, GL_LINEAR,
                                        GL_LINEAR, GL_CLAMP_TO_BORDER, GL_CLAMP_TO_BORDER, NULL);
        mShadowMapTexture[i].setUnit(i);
        // Ignore the texture lookups outside texture coordinates during shadow mapping
        glBindTexture(GL_TEXTURE_2D, mShadowMapTexture[i].getID());
        GLfloat border[] = {1.0f, 0.0f, 0.0f, 0.0f};
		glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);
		glBindTexture(GL_TEXTURE_2D, 0);

        // Create the FBO for shadow map
        mFBOShadowMap[i].create(0, 0, false);
        mFBOShadowMap[i].bind();
        // Dont need a color for depth FBO
        glDrawBuffer(GL_NONE);
        glReadBuffer(GL_NONE);

        mFBOShadowMap[i].attachTexture(GL_DEPTH_ATTACHMENT, mShadowMapTexture[i].getID());
        mFBOShadowMap[i].unbind();
    }
    mIsShadowMappingInitialized = true;

}

// set where main scene camera looks at
void RenderingSystem::SetScenePosition(const openglframework::Vector3& position, float sceneRadius) {
    // Set the position and radius of the scene
    mSceneCenter = position;
    mCamera.setSceneRadius(sceneRadius);
    ResetCameraToViewAll(); // Reset camera position and zoom to view all the scene
}

void RenderingSystem::ResetCameraToViewAll() {
    mCamera.translateWorld(-mCamera.getOrigin()); // move the camera to the origin of the scene (0,0,0)
    mCamera.translateWorld(mSceneCenter); // then move the camera to the center of the scene (user defined)
    mCamera.setZoom(1); // set the zoom of the camera so that the scene is in negative view direction of the camera
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
    (void)button;
    (void)mods;

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
    (void)xAxis;
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

    // Update debug buffers
    if(mIsDebugRenderingEnabled) {
        mDebugTriangles.clear(); // clear previous debug shapes
        for (const auto& debug_box : mDebugBoxes) {
            DrawDebugBox(debug_box);
        }
        for(const auto& debug_sphere : mDebugSpheres) {
            DrawDebugSphere(debug_sphere);
        }
        for (const auto& debug_rect : mDebugRectangles) {
            DrawDebugRectangle(debug_rect);
        }   
        mDebugLines.clear();
        for (const auto& debug_aabb : mDebugAABBs) {
            DrawDebugAABB(debug_aabb);
        }
        for(const auto& debug_arrow : mDebugArrows) {
            DrawDebugArrow(debug_arrow);
        }

        DrawDebugRays();
        

        UpdateDebugVBOVAO();
    }

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    // Render the scene to generate the shadow map (1st pass)
    openglframework::Matrix4 shadowMapProjMatrix[NUM_SHADOW_MAPS];
    openglframework::Matrix4 worldToLightCameraMatrix[NUM_SHADOW_MAPS];
    for(int i = 0; i< NUM_SHADOW_MAPS; i++) {
        shadowMapProjMatrix[i] = mShadowMapLightCameras[i].getProjectionMatrix();
        worldToLightCameraMatrix[i] = mShadowMapLightCameras[i].getTransformMatrix().getInverse();
    }

    // --------- 1st pass: Rendering the scene for shadow mapping --------- // 
    if(mIsShadowMappingEnabled) {
        glCullFace(GL_BACK); // To avoid peterpanning. we cull front faces
        for (int i = 0; i < NUM_SHADOW_MAPS; i++) {
            mFBOShadowMap[i].bind();
            mDepthShader.bind();
            mDepthShader.setMatrix4x4Uniform("projectionMatrix", shadowMapProjMatrix[i]);
            glViewport(0, 0, SHADOWMAP_WIDTH, SHADOWMAP_HEIGHT);
            glClear(GL_DEPTH_BUFFER_BIT); // clear previous depth values
            glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
            RenderSinglePass(mDepthShader, worldToLightCameraMatrix[i]);
            mDepthShader.unbind();
            mFBOShadowMap[i].unbind();
        }
        glDisable(GL_POLYGON_OFFSET_FILL);
    }

    // --------- 2nd pass: Rendering the scene for final rendering --------- // 
    glCullFace(GL_BACK);

    // Get the world-space to camera-space matrix
    const openglframework::Matrix4 worldToCameraMatrix = mCamera.getTransformMatrix().getInverse();

    // Attach the shadow mapping results to the relevant textures
    GLint textureUnits[NUM_SHADOW_MAPS];
    if(mIsShadowMappingEnabled) {
        for(int i = 0; i < NUM_SHADOW_MAPS; i++) {
            mShadowMapTexture[i].bind();
            textureUnits[i] = mShadowMapTexture[i].getUnit();
        }
    }

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

    if(mIsDebugRenderingEnabled)
        RenderDebugObjects(mColorShader, worldToCameraMatrix);

    if(mIsShadowMappingEnabled) {
        for(auto& shadowTexture : mShadowMapTexture) {
            shadowTexture.unbind();
        }
    }

    mPhongShader.unbind();
}

void RenderingSystem::RenderSinglePass(openglframework::Shader& shader, const openglframework::Matrix4& worldToCameraMatrix) {
    
    shader.bind();

    // For all objects, render
    for (auto const& entity : m_entities) {
        auto& mesh = gOrchestrator.GetComponent<Mesh>(entity);

        shader.bind();
        // Set the model to camera matrix
        auto& transform = gOrchestrator.GetComponent<Transform3D>(entity);
        shader.setMatrix4x4Uniform("localToWorldMatrix", transform.opengl_transform.getTransformMatrix());
        shader.setMatrix4x4Uniform("worldToCameraMatrix", worldToCameraMatrix);

        // Set the normal matrix (Inverse Transpose of the 3x3 upper-left part of the model-view matrix)
        const openglframework::Matrix4 localToCameraMatrix = worldToCameraMatrix * transform.opengl_transform.getTransformMatrix();
        const openglframework::Matrix3 normalMatrix = localToCameraMatrix.getUpperLeft3x3Matrix().getInverse().getTranspose();
        shader.setMatrix3x3Uniform("normalMatrix", normalMatrix, false);

        // Set the vertex color
        openglframework::Vector4 color(mesh.mColor.r, mesh.mColor.g, mesh.mColor.b, mesh.mColor.a);
        shader.setVector4Uniform("globalVertexColor", color, false);

        // Bind VAO
        mVAO.at(mesh.mShape).bind();
        mVBOVertices.at(mesh.mShape).bind();

        // Get the location of shader attribute variables
        GLint vertexPositionAttLoc = shader.getAttribLocation("vertexPosition");
        GLint vertexNormalAttLoc = shader.getAttribLocation("vertexNormal", false);
        glEnableVertexAttribArray(vertexPositionAttLoc);
        glVertexAttribPointer(vertexPositionAttLoc, 3, GL_FLOAT, GL_FALSE, 0, (char*)nullptr);

        mVBONormals.at(mesh.mShape).bind();
        if(vertexNormalAttLoc != -1) glVertexAttribPointer(vertexNormalAttLoc, 3, GL_FLOAT, GL_FALSE, 0, (char*)nullptr);
        if(vertexNormalAttLoc != -1) glEnableVertexAttribArray(vertexNormalAttLoc);

        // For each part of the mesh
        for(unsigned int i = 0; i < mesh.getNbParts(); i++) {
            glDrawElements(GL_TRIANGLES, mesh.getNbFaces(i) * 3, GL_UNSIGNED_INT, (char*)nullptr);
        }

        glDisableVertexAttribArray(vertexPositionAttLoc);
        if (vertexNormalAttLoc != -1) glDisableVertexAttribArray(vertexNormalAttLoc);

        mVBONormals.at(mesh.mShape).unbind();
        mVBOVertices.at(mesh.mShape).unbind();
        mVAO.at(mesh.mShape).unbind();
        shader.unbind();
    }

    shader.unbind();
}

void RenderingSystem::RenderDebugObjects(openglframework::Shader& shader, const openglframework::Matrix4& worldToCameraMatrix) {
    // Render in wideframe mode
    // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    shader.bind();

    // Set the normal matrix
    const openglframework::Matrix4 localToCameraMatrix = worldToCameraMatrix;
    const openglframework::Matrix3 normalMatrix = localToCameraMatrix.getUpperLeft3x3Matrix().getInverse().getTranspose();
    shader.setMatrix3x3Uniform("normalMatrix", normalMatrix, false);

    // Set the model to camere matrix
    shader.setMatrix4x4Uniform("localToWorldMatrix", openglframework::Matrix4::identity());
    shader.setMatrix4x4Uniform("worldToCameraMatrix", worldToCameraMatrix);
    shader.setIntUniform("isGlobalVertexColorEnabled", 0, false);
    // Get the location of shader attribute variables
    GLint vertexPositionLoc = shader.getAttribLocation("vertexPosition");
    GLint vertexColorLoc = shader.getAttribLocation("vertexColor");    

    if(!mDebugTriangles.empty()) {
        mDebugTrianglesVAO.bind();
        mDebugVBOTrianglesVertices.bind();
        glEnableVertexAttribArray(vertexPositionLoc);
        glVertexAttribPointer(vertexPositionLoc, 3, GL_FLOAT, GL_FALSE, sizeof(rp3d::Vector3) + sizeof(rp3d::uint32), (char*)nullptr);
        glEnableVertexAttribArray(vertexColorLoc);
        glVertexAttribIPointer(vertexColorLoc, 3, GL_UNSIGNED_INT, sizeof(rp3d::Vector3) + sizeof(rp3d::uint32), (void*)sizeof(rp3d::Vector3));
        
        glDrawArrays(GL_TRIANGLES, 0, mDebugTriangles.size()*3); // draw the triangles geometry

        glDisableVertexAttribArray(vertexPositionLoc);
        glDisableVertexAttribArray(vertexColorLoc);
        mDebugVBOTrianglesVertices.unbind();
        mDebugTrianglesVAO.unbind();
    }

    glDisable(GL_CULL_FACE);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glEnable(GL_LINE_SMOOTH);
    if(!mDebugLines.empty()) {
        mDebugLinesVAO.bind();
        mDebugVBOLinesVertices.bind();
        glEnableVertexAttribArray(vertexPositionLoc);
        glVertexAttribPointer(vertexPositionLoc, 3, GL_FLOAT, GL_FALSE, sizeof(rp3d::Vector3) + sizeof(rp3d::uint32), (char*)nullptr);
        glEnableVertexAttribArray(vertexColorLoc);
        glVertexAttribIPointer(vertexColorLoc, 3, GL_UNSIGNED_INT, sizeof(rp3d::Vector3) + sizeof(rp3d::uint32), (void*)sizeof(rp3d::Vector3));
        
        glDrawArrays(GL_TRIANGLES, 0, mDebugLines.size()*2); // draw the lines geometry

        glDisableVertexAttribArray(vertexPositionLoc);
        glDisableVertexAttribArray(vertexColorLoc);
        mDebugVBOLinesVertices.unbind();
        mDebugLinesVAO.unbind();
    } 

    shader.unbind();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); // disable wideframe mode
    glDisable(GL_LINE_SMOOTH);
    glEnable(GL_CULL_FACE);
}



void RenderingSystem::DrawDebugBox(const DebugBox& debug_box) {
    const auto& transform = debug_box.transform;
    const auto& half_extents = debug_box.halfExtents;
    const auto color = static_cast<uint32_t>(debug_box.color);
    rp3d::Vector3 debug_vertices[8];

    // vertices
	debug_vertices[0] = transform * rp3d::Vector3(-half_extents.x, -half_extents.y, half_extents.z);
	debug_vertices[1] = transform * rp3d::Vector3(half_extents.x, -half_extents.y, half_extents.z);
	debug_vertices[2] = transform * rp3d::Vector3(half_extents.x, -half_extents.y, -half_extents.z);
	debug_vertices[3] = transform * rp3d::Vector3(-half_extents.x, -half_extents.y, -half_extents.z);
	debug_vertices[4] = transform * rp3d::Vector3(-half_extents.x, half_extents.y, half_extents.z);
	debug_vertices[5] = transform * rp3d::Vector3(half_extents.x, half_extents.y, half_extents.z);
	debug_vertices[6] = transform * rp3d::Vector3(half_extents.x, half_extents.y, -half_extents.z);
	debug_vertices[7] = transform * rp3d::Vector3(-half_extents.x, half_extents.y, -half_extents.z);

    // triangle faces
	mDebugTriangles.emplace_back(DebugTriangle(debug_vertices[0], debug_vertices[1], debug_vertices[5], color));
	mDebugTriangles.emplace_back(DebugTriangle(debug_vertices[0], debug_vertices[5], debug_vertices[4], color));
	mDebugTriangles.emplace_back(DebugTriangle(debug_vertices[1], debug_vertices[2], debug_vertices[6], color));
	mDebugTriangles.emplace_back(DebugTriangle(debug_vertices[1], debug_vertices[6], debug_vertices[5], color));
	mDebugTriangles.emplace_back(DebugTriangle(debug_vertices[2], debug_vertices[3], debug_vertices[6], color));
	mDebugTriangles.emplace_back(DebugTriangle(debug_vertices[3], debug_vertices[7], debug_vertices[6], color));
	mDebugTriangles.emplace_back(DebugTriangle(debug_vertices[0], debug_vertices[7], debug_vertices[3], color));
	mDebugTriangles.emplace_back(DebugTriangle(debug_vertices[0], debug_vertices[4], debug_vertices[7], color));
	mDebugTriangles.emplace_back(DebugTriangle(debug_vertices[0], debug_vertices[2], debug_vertices[1], color));
	mDebugTriangles.emplace_back(DebugTriangle(debug_vertices[0], debug_vertices[3], debug_vertices[2], color));
	mDebugTriangles.emplace_back(DebugTriangle(debug_vertices[5], debug_vertices[6], debug_vertices[4], color));
	mDebugTriangles.emplace_back(DebugTriangle(debug_vertices[4], debug_vertices[6], debug_vertices[7], color));

}

void RenderingSystem::DrawDebugRectangle(const DebugRectangle& debug_rectangle) {
    const auto& transform = debug_rectangle.transform;
    const auto& half_extents = debug_rectangle.halfExtents;
    const auto color = debug_rectangle.color;
    rp3d::Vector3 debug_vertices[4];
    // vertices
    debug_vertices[0] = transform * rp3d::Vector3(-half_extents.x, 0, -half_extents.z);
    debug_vertices[1] = transform * rp3d::Vector3(-half_extents.x, 0, half_extents.z);
    debug_vertices[2] = transform * rp3d::Vector3(half_extents.x, 0, half_extents.z);
    debug_vertices[3] = transform * rp3d::Vector3(half_extents.x, 0, -half_extents.z);

    mDebugTriangles.emplace_back(DebugTriangle(debug_vertices[0], debug_vertices[1], debug_vertices[2], color));
    mDebugTriangles.emplace_back(DebugTriangle(debug_vertices[2], debug_vertices[3], debug_vertices[0], color));
}

void RenderingSystem::DrawDebugSphere(const DebugSphere& debug_sphere) {
    const auto& position = debug_sphere.position;
    const auto& radius = debug_sphere.radius;
    const auto color = static_cast<uint32_t>(debug_sphere.color);

    rp3d::Vector3 vertices[(NB_SECTORS_SPHERE + 1) * (NB_STACKS_SPHERE + 1) + (NB_SECTORS_SPHERE + 1)];
	
	// Vertices
	const float sectorStep = 2 * rp3d::PI / NB_SECTORS_SPHERE;
	const float stackStep = rp3d::PI / NB_STACKS_SPHERE;
	
	for (uint i = 0; i <= NB_STACKS_SPHERE; i++) {

		const float stackAngle = rp3d::PI / 2 - i * stackStep;
		const float radiusCosStackAngle = radius * std::cos(stackAngle);
		const float z = radius * std::sin(stackAngle);

        for (uint j = 0; j <= NB_SECTORS_SPHERE; j++) {
		
			const float sectorAngle = j * sectorStep;
			const float x = radiusCosStackAngle * std::cos(sectorAngle);
			const float y = radiusCosStackAngle * std::sin(sectorAngle);

            vertices[i * (NB_SECTORS_SPHERE + 1) + j] = position + rp3d::Vector3(x, y, z);
		}
	}

	// Faces
	for (uint i = 0; i < NB_STACKS_SPHERE; i++) {
		uint a1 = i * (NB_SECTORS_SPHERE + 1);
		uint a2 = a1 + NB_SECTORS_SPHERE + 1;
		for (uint j = 0; j < NB_SECTORS_SPHERE; j++, a1++, a2++) {
			// 2 triangles per sector except for the first and last stacks
			if (i != 0) {
				mDebugTriangles.emplace_back(DebugTriangle(vertices[a1], vertices[a2], vertices[a1 + 1], color));
			}
			if (i != (NB_STACKS_SPHERE - 1)) {	
				mDebugTriangles.emplace_back(DebugTriangle(vertices[a1 + 1], vertices[a2], vertices[a2 + 1], color));
			}
		}
	}
}

void RenderingSystem::DrawDebugRays() {
    for(const auto& ray : mDebugRays) {
        mDebugLines.emplace_back(DebugLine(ray.point1, ray.point2, static_cast<uint32_t>(rp3d::DebugRenderer::DebugColor::BLUE)));
    }
}

void RenderingSystem::DrawDebugArrow(const DebugArrow& debug_arrow) {
    const auto& transform = debug_arrow.transform;
    const auto color = static_cast<uint32_t>(debug_arrow.color);

    mDebugLines.emplace_back(DebugLine(transform * rp3d::Vector3(0, 0.1, 0), transform *rp3d::Vector3(0, 0.1, -1.5), color)); // arrow body
    mDebugLines.emplace_back(DebugLine(transform *rp3d::Vector3(0, 0.1, -1.5), transform *rp3d::Vector3(-0.5, 0.1, -1), color)); // left arrow head edge
    mDebugLines.emplace_back(DebugLine(transform *rp3d::Vector3(0, 0.1, -1.5), transform *rp3d::Vector3(0.5, 0.1, -1), color)); // right arrow head edge
}

void RenderingSystem::DrawDebugAABB(const DebugAABB& debug_aabb) {
    const auto& transform = debug_aabb.transform;
    const auto& min_local = debug_aabb.min;
    const auto& max_local = debug_aabb.max;
    const auto color = static_cast<uint32_t>(debug_aabb.color);

	// Local to world
    const rp3d::Vector3 min = transform * min_local;
    const rp3d::Vector3 max = transform * max_local;
    
    // Bottom edges
	mDebugLines.emplace_back(DebugLine(rp3d::Vector3(min.x, min.y, max.z), rp3d::Vector3(max.x, min.y, max.z), color));
	mDebugLines.emplace_back(DebugLine(rp3d::Vector3(max.x, min.y, max.z),  rp3d::Vector3(max.x, min.y, min.z), color));
	mDebugLines.emplace_back(DebugLine(rp3d::Vector3(max.x, min.y, min.z), rp3d::Vector3(min.x, min.y, min.z), color));
	mDebugLines.emplace_back(DebugLine(rp3d::Vector3(min.x, min.y, min.z), rp3d::Vector3(min.x, min.y, max.z), color));

	// Top edges
	mDebugLines.emplace_back(DebugLine(rp3d::Vector3(min.x, max.y, max.z), rp3d::Vector3(max.x, max.y, max.z), color));
	mDebugLines.emplace_back(DebugLine(rp3d::Vector3(max.x, max.y, max.z), rp3d::Vector3(max.x, max.y, min.z), color));
	mDebugLines.emplace_back(DebugLine(rp3d::Vector3(max.x, max.y, min.z), rp3d::Vector3(min.x, max.y, min.z), color));
	mDebugLines.emplace_back(DebugLine(rp3d::Vector3(min.x, max.y, min.z), rp3d::Vector3(min.x, max.y, max.z), color));

	// Side edges
	mDebugLines.emplace_back(DebugLine(rp3d::Vector3(min.x, min.y, max.z), rp3d::Vector3(min.x, max.y, max.z), color));
	mDebugLines.emplace_back(DebugLine(rp3d::Vector3(max.x, min.y, max.z), rp3d::Vector3(max.x, max.y, max.z), color));
	mDebugLines.emplace_back(DebugLine(rp3d::Vector3(max.x, min.y, min.z), rp3d::Vector3(max.x, max.y, min.z), color));
	mDebugLines.emplace_back(DebugLine(rp3d::Vector3(min.x, min.y, min.z), rp3d::Vector3(min.x, max.y, min.z), color));    
}

void RenderingSystem::TimerThreadFunc() {
    while(true) {
        // Not sleeping here, maximizing the FPS
        std::this_thread::sleep_for(this->quantum); // since no other operation happens in this thread, enough to just sleep for FPS rate
        // This is the main event interrupt which allows steady render rate (irrespective of mouse/keyboard callbacks)
        // after sleeping for RENDER_TIME_STEP
        // redraw() calls glfwPostEmptyEvent() which allows the iteration to proceed from the blocked glfwWaitEvents() state
        // and also sets m_redraw=True, for allowing draw_all to do rendering
        for(auto screen : this->GetNanoguiScreen())
            screen.second->redraw();                
    }       
}

void RenderingSystem::Update() {
        for(auto& entity : m_entities) {
            auto& transform = gOrchestrator.GetComponent<lazyECS::Transform3D>(entity);
            transform.ConvertRP3DToOpenglTransform(); // convert the physics transform to graphics transform
        }

        for(auto& screen_pair : GetNanoguiScreen()) {
            nanogui::Screen* screen = screen_pair.second;
            if(!screen->visible()) {
                continue;
            }
            else if(glfwWindowShouldClose(screen->glfw_window())) {
                screen->set_visible(false);
                continue;
            }
            else{
                screen->draw_all(); // only draws if m_redraw==True, sets m_redraw=False after iteration
            }
            // Blocks next iteration until:
            // a) A keyboard/mouse event is made
            // b) render_timer_thread periodically interrupts it by a glfwPostEmptyEvent call for steady render rate.
            // The program waits here only if everything else (physics + App) has already been finished in less time than render rate
            glfwWaitEvents();
        }    
}

void RenderingSystem::GuiInit() {
    // Gui Initialization
    nanogui::Widget* info_panel = new nanogui::Window(this, "Sim.");
    info_panel->set_position(nanogui::Vector2i(10,10));
    info_panel->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 10, 5));
    info_panel->set_fixed_width(220);
    fpsLabel_ = new nanogui::Label(info_panel, std::string("FPS : ") + std::to_string(1.0F/deltaTime), "sans-bold");
    info_panel->set_visible(true);




    this->perform_layout();
}

void RenderingSystem::UpdateGui() const {
    fpsLabel_->set_caption("FPS: " + std::to_string(1.0F/deltaTime));
}


} // namespace lazyECS

