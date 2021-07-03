#pragma once

#include <cstddef>
#include <cstdint>
#include <reactphysics3d/reactphysics3d.h>
#include <functional>

using Position2D = rp3d::Vector2;

enum class CellState {
    GOAL,
    FREE,
    OBSTACLE,
    TRAVELED
};

enum class GridColor {
    GOAL = 0xEF00FF, // magenta
    FREE = 0xBAB2B2, // light gray
    OBSTACLE = 0x000000, // black
    TRAVELED = 0xFFE600 // yellow
};


// Node is just a lightweight cell, which does not have AABB or DebugRectangle component
struct Node 
{
    uint16_t x;
    uint16_t z;
    int distance;
    Node(const size_t& x_in, const size_t& z_in, const int& distance_in) : x(x_in), z(z_in), distance(distance_in) {}

    bool operator == (const Node& other_node) const {
        return ( (this->x == other_node.x) && (this->z == other_node.z) );
    }    
};

struct NodeHashFunc {

    std::size_t operator()(const Node& node) const {
        uint64_t result = ( (node.x) << 16 ) | node.z;
        return std::hash<uint64_t>{}(result);  
    }
};