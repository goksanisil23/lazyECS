cmake_minimum_required(VERSION 3.12)

project(lazyECS)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(lazyECS main.cpp)

target_sources(
    lazyECS
    PRIVATE
    src/ECSCore/ComponentArrayHandler.hpp
    src/ECSCore/ComponentManager.hpp
    src/ECSCore/EntityManager.hpp
    src/ECSCore/SystemManager.hpp
    src/ECSCore/Orchestrator.hpp
    src/ECSCore/Types.hpp
    src/Math/Vec2.hpp
    src/Math/Vec3.hpp
    src/Math/Vec4.hpp
    src/Math/Mat44.hpp
    src/Components/RigidBody.hpp
    src/Components/Transform.hpp
)

target_sources(
    lazyECS
    PRIVATE
    src/Systems/PhysicsSystem.cpp
)

target_include_directories(
    lazyECS
    PRIVATE
    src
)