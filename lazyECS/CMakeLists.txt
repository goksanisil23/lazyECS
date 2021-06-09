cmake_minimum_required(VERSION 3.12)

# project(lazyECS CXX)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
 
########################## raylib ##########################
# find_package(raylib REQUIRED)

# message("raylib include dir: " ${raylib_INCLUDE_DIR})
# message("raylib library dir: " ${raylib_LIBRARY_DIR})

###################### NANOGUI ##############################
# Disable building extras of nanogui we won't need (pure C++ project)
set(NANOGUI_BUILD_EXAMPLES OFF CACHE BOOL " " FORCE)
set(NANOGUI_BUILD_PYTHON  OFF CACHE BOOL " " FORCE)
set(NANOGUI_INSTALL       OFF CACHE BOOL " " FORCE)
set(NANOGUI_BACKEND       OpenGL CACHE BOOL " " FORCE)
# Ask Nanogui not to build shared libraries
SET(NANOGUI_BUILD_SHARED OFF CACHE BOOL "Build Nanogui with static libraries" FORCE)
add_subdirectory(External/nanogui)
######################## ReachPhysics3D #####################

find_package(ReactPhysics3D REQUIRED)

message("ReachPhysics3D include dir: ${ReactPhysics3D_INCLUDE_DIR}" )
message("ReachPhysics3D library dir: ${ReactPhysics3D_LIBRARY_DIR}" )

########################### OPENGL FRAMEWORK ######################
# OpenGLFramework source files
set(OPENGLFRAMEWORK_SOURCES
    Systems/opengl-framework/src/maths/Color.h
    Systems/opengl-framework/src/maths/Matrix3.h
    Systems/opengl-framework/src/maths/Matrix4.h
    Systems/opengl-framework/src/maths/Vector2.h
    Systems/opengl-framework/src/maths/Vector3.h
    Systems/opengl-framework/src/maths/Vector4.h
    Systems/opengl-framework/src/Camera.cpp
    Systems/opengl-framework/src/Camera.h
    Systems/opengl-framework/src/definitions.h
    # Systems/opengl-framework/src/FrameBufferObject.cpp
    # Systems/opengl-framework/src/FrameBufferObject.h
    Systems/opengl-framework/src/Light.h
    Systems/opengl-framework/src/Light.cpp
    Systems/opengl-framework/src/Mesh.h
    Systems/opengl-framework/src/Mesh.cpp
    # Systems/opengl-framework/src/MeshReaderWriter.h
    # Systems/opengl-framework/src/MeshReaderWriter.cpp
    Systems/opengl-framework/src/Object3D.h
    Systems/opengl-framework/src/Object3D.cpp
    Systems/opengl-framework/src/openglframework.h
    Systems/opengl-framework/src/Shader.h
    Systems/opengl-framework/src/Shader.cpp
    Systems/opengl-framework/src/Texture2D.h
    Systems/opengl-framework/src/Texture2D.cpp
    # Systems/opengl-framework/src/TextureReaderWriter.h
    # Systems/opengl-framework/src/TextureReaderWriter.cpp
    Systems/opengl-framework/src/VertexBufferObject.h
    Systems/opengl-framework/src/VertexBufferObject.cpp
    Systems/opengl-framework/src/VertexArrayObject.h
    Systems/opengl-framework/src/VertexArrayObject.cpp
)

######################## MAIN lazyECS #########################

set(
    LAZY_ECS_SOURCES
    ECSCore/ComponentArrayHandler.hpp
    ECSCore/ComponentManager.hpp
    ECSCore/EntityManager.hpp
    ECSCore/SystemManager.hpp
    ECSCore/Orchestrator.hpp
    ECSCore/Types.hpp

    Components/RigidBody3D.hpp
    Components/Transform3D.hpp
    # Components/RigidBody2D.hpp
    # Components/Transform2D.hpp
    # Components/Mesh.hpp

    ${OPENGLFRAMEWORK_SOURCES}
    
    Systems/PhysicsSystem.hpp
    Systems/PhysicsSystem.cpp
    Systems/RenderingSystem.hpp
    Systems/RenderingSystem.cpp
    # Systems/RenderingSystemRaylib.cpp
    # Systems/RenderingSystemRaylib.hpp

    # SceneFramework/Scene.h
    # SceneFramework/Scene.cpp
)

# #Generate the static library from the sources
# add_library(lazyECS SHARED ${LAZY_ECS_SOURCES})
add_library(lazyECS ${LAZY_ECS_SOURCES})

target_link_libraries(lazyECS ReactPhysics3D::ReactPhysics3D nanogui ${NANOGUI_EXTRA_LIBS})
target_include_directories(lazyECS PUBLIC
                                        ${CMAKE_CURRENT_SOURCE_DIR}
                                        ${CMAKE_CURRENT_SOURCE_DIR}/Systems/opengl-framework/src
                                        ${CMAKE_CURRENT_SOURCE_DIR}/External/nanogui/include
                                        # ${raylib_INCLUDE_DIR}
                                        ${ReactPhysics3D_INCLUDE_DIR}
                                        # ../external/raylib-cpp/include
                                        )

foreach(p ${NANOGUI_EXTRA_INCS})
    target_include_directories(lazyECS PUBLIC $<1:${p}>)
endforeach()

# Compile definitions
message("NANOGUI_EXTRA_DEFS: ${NANOGUI_EXTRA_DEFS}")
target_compile_definitions(lazyECS PUBLIC ${NANOGUI_EXTRA_DEFS})

