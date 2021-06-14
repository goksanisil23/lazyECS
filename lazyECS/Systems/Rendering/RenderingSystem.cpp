#include "RenderingSystem.hpp"

#include "ECSCore/Orchestrator.hpp"

extern lazyECS::Orchestrator gOrchestrator; // expected to be defined globally in main

namespace lazyECS {

RenderingSystem::RenderingSystem() : mLastMouseX(0), mLastMouseY(0), mViewportX(0), mViewportY(0),
                                     mViewportWidth(0), mViewportHeight(0), 
                                     mIsShadowMappingEnabled(false), mIsShadowMappingInitialized(false),
                                     mVBOVertices(GL_ARRAY_BUFFER), mVBONormals(GL_ARRAY_BUFFER), mVBOTextureCoords(GL_ARRAY_BUFFER),
                                     mVBOIndices(GL_ELEMENT_ARRAY_BUFFER), mVAO(), numRenderables(0),
                                     mPhongShader("/home/goksan/Work/lazyECS/lazyECS/Systems/Rendering/shaders/phong.vert",
                                                  "/home/goksan/Work/lazyECS/lazyECS/Systems/Rendering/shaders/phong.frag"),
                                     mColorShader("/home/goksan/Work/lazyECS/lazyECS/Systems/Rendering/shaders/color.vert",
                                                  "/home/goksan/Work/lazyECS/lazyECS/Systems/Rendering/shaders/color.frag")
{}

void RenderingSystem::SetupSignature() {
    // Set the system signature based on the utilized Components below
    Signature signature;
    // signature.set(gOrchestrator.GetComponentTypeId<RigidBody3D>(), true);
    signature.set(gOrchestrator.GetComponentTypeId<Transform3D>(), true);
    signature.set(gOrchestrator.GetComponentTypeId<Mesh>(), true);
    gOrchestrator.SetSystemSignature<RenderingSystem>(signature);     
}

void RenderingSystem::Init(const std::string& meshPath) {

    // ------------- Scene setup (TODO: move to a new class/function) ------------------ //
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

    openglframework::Vector3 center(0, 0, 0);
    const float SCENE_RADIUS = 30.0f; // 30
    SetScenePosition(center, SCENE_RADIUS);

    // Populate the mesh for the entities that have a Mesh component, from a model file
    for(auto& entity : m_entities) {
        auto& transform = gOrchestrator.GetComponent<Transform3D>(entity); // initialized outsize
        auto& mesh = gOrchestrator.GetComponent<Mesh>(entity); // uninitialized here
        openglframework::MeshReaderWriter::loadMeshFromFile(meshPath, mesh); // mesh is initialized here
        if(mesh.getNormals().empty()) { // if the mesh file don't have the normals, calculate them
            mesh.calculateNormals();
        }
        
        // Set the scaling
        transform.opengl_transform.setTransformMatrix(transform.opengl_transform.getTransformMatrix() * transform.mScalingMatrix);
        
        // Create Vertex Buffer object if it hasn't been created yet (we do it here since we need vertex info from the Mesh)
        if(numRenderables == 0)
            CreateVBOVAO(mesh);
        numRenderables++;
    }
}

void RenderingSystem::CreateVBOVAO(Mesh& mesh) {
    // VBO for the vertices data
    mVBOVertices.create();
    mVBOVertices.bind();
    size_t sizeVertices = mesh.getVertices().size() * sizeof(openglframework::Vector3);
    mVBOVertices.copyDataIntoVBO(sizeVertices, mesh.getVerticesPointer(), GL_STATIC_DRAW);
    mVBOVertices.unbind();

    // Create the VBO for the normals data
    mVBONormals.create();
    mVBONormals.bind();
    size_t sizeNormals = mesh.getNormals().size() * sizeof(openglframework::Vector3);
    mVBONormals.copyDataIntoVBO(sizeNormals, mesh.getNormalsPointer(), GL_STATIC_DRAW);
    mVBONormals.unbind();

    // Create VBO for the texture coordinates
    if(mesh.hasTexture()) {
        mVBOTextureCoords.create();
        mVBOTextureCoords.bind();
        size_t sizeTextureCoords = mesh.getUVs().size() * sizeof(openglframework::Vector2);
        mVBOTextureCoords.copyDataIntoVBO(sizeTextureCoords, mesh.getUVTextureCoordinatesPointer(), GL_STATIC_DRAW);
        mVBOTextureCoords.unbind();
    }

    // Create VBO for the indices data
    mVBOIndices.create();
    mVBOIndices.bind();
    size_t sizeIndices = mesh.getIndices().size() * sizeof(unsigned int);
    mVBOIndices.copyDataIntoVBO(sizeIndices, mesh.getIndicesPointer(), GL_STATIC_DRAW);
    mVBOIndices.unbind();

    // Create VAO and bind all the VBOs
    mVAO.create();
    mVAO.bind();

    mVBOVertices.bind();
    mVBONormals.bind();
    if (mesh.hasTexture()) mVBOTextureCoords.bind();
    mVBOIndices.bind();

    mVAO.unbind(); 
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
    mCamera.setZoom(1); // set the zoom of the camera so that the scene is in negative view direction of the camera
}

bool RenderingSystem::MapMouseCoordinatesToSphere(double xMouse, double yMouse, openglframework::Vector3& spherePoint) const {
    printf("xMouse: %f, yMouse: %f, windowWidth: %d, windowHeight: %d\n");
    if ((xMouse >= 0) && (xMouse <= mWindowWidth) && (yMouse >= 0) && (yMouse <= mWindowHeight)) {
        std::cout << "inside the sphere" << std::endl;
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

        std::cout << "ZOOM" << std::endl;
        Zoom(-dy / h); // Zoom the camera
    }
    // Translation
    else if (middleButtonState == GLFW_PRESS || rightButtonState == GLFW_PRESS ||
             (leftButtonState == GLFW_PRESS && altKeyState == GLFW_PRESS)) {
        std::cout << "TRANSLATE" << std::endl;
        Translate(xMouse, yMouse);
    }
    // Rotation
    else if (leftButtonState == GLFW_PRESS) {
        std::cout << "ROTATE" << std::endl;
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
        std::cout << "LastPointOnSphereValid" << std::endl;

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

void RenderingSystem::Render2() {

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

    // TEMP
    mShadowMapBiasMatrix.setAllValues(0.5, 0.0, 0.0, 0.5,
                                      0.0, 0.5, 0.0, 0.5,
                                      0.0, 0.0, 0.5, 0.5,
                                      0.0, 0.0, 0.0, 1.0);
    // END OF TEMP

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
    // mPhongShader.setIntUniform("shadowMapSampler0", textureUnits[0]);
    // mPhongShader.setIntUniform("shadowMapSampler1", textureUnits[1]);
    // mPhongShader.setIntUniform("shadowMapSampler2", textureUnits[2]);
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

    RenderSinglePass(worldToCameraMatrix);

    mPhongShader.unbind();
}

void RenderingSystem::RenderSinglePass(const openglframework::Matrix4& worldToCamereMatrix) {
    
    mPhongShader.bind();

    // For all objects, render
    for (auto const& entity : m_entities) {
        // auto& rigidBody = gOrchestrator.GetComponent<RigidBody3D>(entity);
        auto& mesh = gOrchestrator.GetComponent<Mesh>(entity);

        mPhongShader.bind();
        // Set the model to camera matrix
        auto& transform = gOrchestrator.GetComponent<Transform3D>(entity);
        mPhongShader.setMatrix4x4Uniform("localToWorldMatrix", transform.opengl_transform.getTransformMatrix());
        mPhongShader.setMatrix4x4Uniform("worldToCameraMatrix", worldToCamereMatrix);

        // Set the normal matrix (Inverse Transpose of the 3x3 upper-left part of the model-view matrix)
        const openglframework::Matrix4 localToCameraMatrix = worldToCamereMatrix * transform.opengl_transform.getTransformMatrix();
        const openglframework::Matrix3 normalMatrix = localToCameraMatrix.getUpperLeft3x3Matrix().getInverse().getTranspose();
        mPhongShader.setMatrix3x3Uniform("normalMatrix", normalMatrix, false);

        // Set the vertex color
        // openglframework::Color currentColor = rigidBody.rp3d_rigidBody->isSleeping() ? mesh.mSleepingColor : mesh.mColor;
        // openglframework::Vector4 color(currentColor.r, currentColor.g, currentColor.b, currentColor.a);
        openglframework::Vector4 color(1,1,1,1);
        mPhongShader.setVector4Uniform("globalVertexColor", color, false);

        // Bind VAO
        mVAO.bind();
        mVBOVertices.bind();

        // Get the location of shader attribute variables
        GLint vertexPositionAttLoc = mPhongShader.getAttribLocation("vertexPosition");
        GLint vertexNormalAttLoc = mPhongShader.getAttribLocation("vertexNormal", false);
        glEnableVertexAttribArray(vertexPositionAttLoc);
        glVertexAttribPointer(vertexPositionAttLoc, 3, GL_FLOAT, GL_FALSE, 0, (char*)nullptr);

        mVBONormals.bind();
        if(vertexNormalAttLoc != -1) glVertexAttribPointer(vertexNormalAttLoc, 3, GL_FLOAT, GL_FALSE, 0, (char*)nullptr);
        if(vertexNormalAttLoc != -1) glEnableVertexAttribArray(vertexNormalAttLoc);

        // For each part of the mesh
        for(unsigned int i = 0; i < mesh.getNbParts(); i++) {
            glDrawElements(GL_TRIANGLES, mesh.getNbFaces(i) * 3, GL_UNSIGNED_INT, (char*)nullptr);
        }

        glDisableVertexAttribArray(vertexPositionAttLoc);
        if (vertexNormalAttLoc != -1) glDisableVertexAttribArray(vertexNormalAttLoc);

        mVBONormals.unbind();
        mVBOVertices.unbind();
        mVAO.unbind();
        mPhongShader.unbind();
    }

    mPhongShader.unbind();
}


}

