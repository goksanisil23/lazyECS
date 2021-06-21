cmake_minimum_required(VERSION 3.12)

# project(lazyECS CXX)

# set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

######################## ReachPhysics3D #####################

find_package(ReactPhysics3D REQUIRED)

message("ReachPhysics3D include dir: ${ReactPhysics3D_INCLUDE_DIR}" )
message("ReachPhysics3D library dir: ${ReactPhysics3D_LIBRARY_DIR}" )

########################### OPENGL FRAMEWORK ######################
# OpenGLFramework source files
set(OPENGLFRAMEWORK_SOURCES
    Systems/Rendering/opengl-framework/src/maths/Color.h
    Systems/Rendering/opengl-framework/src/maths/Matrix3.h
    Systems/Rendering/opengl-framework/src/maths/Matrix4.h
    Systems/Rendering/opengl-framework/src/maths/Vector2.h
    Systems/Rendering/opengl-framework/src/maths/Vector3.h
    Systems/Rendering/opengl-framework/src/maths/Vector4.h
    Systems/Rendering/opengl-framework/src/Camera.cpp
    Systems/Rendering/opengl-framework/src/Camera.h
    Systems/Rendering/opengl-framework/src/definitions.h
    Systems/Rendering/opengl-framework/src/FrameBufferObject.cpp
    Systems/Rendering/opengl-framework/src/FrameBufferObject.h
    Systems/Rendering/opengl-framework/src/Light.h
    Systems/Rendering/opengl-framework/src/Light.cpp
    # Systems/Rendering/opengl-framework/src/Mesh.h
    # Systems/Rendering/opengl-framework/src/Mesh.cpp
    Systems/Rendering/opengl-framework/src/MeshReaderWriter.h
    Systems/Rendering/opengl-framework/src/MeshReaderWriter.cpp
    Systems/Rendering/opengl-framework/src/Object3D.h
    Systems/Rendering/opengl-framework/src/Object3D.cpp
    Systems/Rendering/opengl-framework/src/openglframework.h
    Systems/Rendering/opengl-framework/src/Shader.h
    Systems/Rendering/opengl-framework/src/Shader.cpp
    Systems/Rendering/opengl-framework/src/Texture2D.h
    Systems/Rendering/opengl-framework/src/Texture2D.cpp
    # Systems/Rendering/opengl-framework/src/TextureReaderWriter.h
    # Systems/Rendering/opengl-framework/src/TextureReaderWriter.cpp
    Systems/Rendering/opengl-framework/src/VertexBufferObject.h
    Systems/Rendering/opengl-framework/src/VertexBufferObject.cpp
    Systems/Rendering/opengl-framework/src/VertexArrayObject.h
    Systems/Rendering/opengl-framework/src/VertexArrayObject.cpp
)

######################## MAIN lazyECS #########################

set(LAZY_ECS_SOURCES
    
    ECSCore/ComponentArrayHandler.hpp
    ECSCore/ComponentManager.hpp
    ECSCore/EntityManager.hpp
    ECSCore/SystemManager.hpp
    ECSCore/Orchestrator.hpp
    ECSCore/Types.hpp

    Components/RigidBody3D.hpp
    Components/Transform3D.hpp
    Components/Transform3D.cpp
    Components/Mesh.hpp
    Components/Mesh.cpp
    Components/utils/PerlinNoise.h
    Components/utils/PerlinNoise.cpp
    Components/utils/HeightField.h
    Components/utils/HeightField.cpp

    ${OPENGLFRAMEWORK_SOURCES}
    
    Systems/PhysicsSystem.hpp
    Systems/PhysicsSystem.cpp
    Systems/Rendering/RenderingSystem.hpp
    Systems/Rendering/RenderingSystem.cpp
    # Systems/Rendering/Gui.h
    # Systems/Rendering/Gui.cpp

    # SceneFramework/Scene.h
    # SceneFramework/Scene.cpp
)

# #Generate the static library from the sources
# add_library(lazyECS SHARED ${LAZY_ECS_SOURCES})
add_library(lazyECS STATIC ${LAZY_ECS_SOURCES})

target_include_directories(lazyECS PUBLIC ${LAZY_ECS_INCLUDE_DIRS})

foreach(p ${NANOGUI_EXTRA_INCS})
    target_include_directories(lazyECS PUBLIC ${p})
endforeach()

# Compile definitions
message("NANOGUI_EXTRA_DEFS: ${NANOGUI_EXTRA_DEFS}")
target_compile_definitions(lazyECS PUBLIC ${NANOGUI_EXTRA_DEFS})
target_link_libraries(lazyECS ReactPhysics3D::ReactPhysics3D nanogui ${NANOGUI_EXTRA_LIBS})
