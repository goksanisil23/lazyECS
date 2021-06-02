cmake_minimum_required(VERSION 3.12)

# project(lazyECS CXX)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
 
########################## raylib ##########################
find_package(raylib REQUIRED)

message("raylib include dir: " ${raylib_INCLUDE_DIR})
message("raylib binary dir: " ${raylib_LIBRARY_DIR})

######################## MAIN lazyECS #########################

set(
    LAZY_ECS_SOURCES
    ECSCore/ComponentArrayHandler.hpp
    ECSCore/ComponentManager.hpp
    ECSCore/EntityManager.hpp
    ECSCore/SystemManager.hpp
    ECSCore/Orchestrator.hpp
    ECSCore/Types.hpp
    Components/RigidBody.hpp
    Components/Transform.hpp
    Systems/PhysicsSystem.hpp
    Systems/PhysicsSystem.cpp
    Systems/RenderingSystem.cpp
    Systems/RenderingSystem.hpp
)

# #Generate the static library from the sources
# add_library(lazyECS SHARED ${LAZY_ECS_SOURCES})
add_library(lazyECS ${LAZY_ECS_SOURCES})

target_link_libraries(lazyECS raylib)
target_include_directories(lazyECS PUBLIC
                                        ${CMAKE_CURRENT_SOURCE_DIR}
                                        ${raylib_INCLUDE_DIR}
                                        ../external/raylib-cpp/include)

