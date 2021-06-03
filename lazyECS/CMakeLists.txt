cmake_minimum_required(VERSION 3.12)

# project(lazyECS CXX)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
 
########################## raylib ##########################
find_package(raylib REQUIRED)

message("raylib include dir: " ${raylib_INCLUDE_DIR})
message("raylib library dir: " ${raylib_LIBRARY_DIR})

######################## ReachPhysics3D #####################

find_package(ReactPhysics3D REQUIRED)

message("ReachPhysics3D include dir: " ${ReactPhysics3D_INCLUDE_DIR})
message("ReachPhysics3D library dir: " ${ReactPhysics3D_LIBRARY_DIR})

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
    Components/RigidBody2D.hpp
    Components/Transform2D.hpp    
    Systems/PhysicsSystem.hpp
    Systems/PhysicsSystem.cpp
    Systems/RenderingSystem2D.cpp
    Systems/RenderingSystem2D.hpp
)

# #Generate the static library from the sources
# add_library(lazyECS SHARED ${LAZY_ECS_SOURCES})
add_library(lazyECS ${LAZY_ECS_SOURCES})

target_link_libraries(lazyECS raylib ReactPhysics3D::ReactPhysics3D)
target_include_directories(lazyECS PUBLIC
                                        ${CMAKE_CURRENT_SOURCE_DIR}
                                        ${raylib_INCLUDE_DIR}
                                        ${ReactPhysics3D_INCLUDE_DIR}
                                        ../external/raylib-cpp/include)

