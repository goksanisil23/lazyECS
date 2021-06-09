#pragma once

#include "raylib-cpp.hpp" // for Vector library
#include <any> // for mixed map to get different parameter set per primitive
#include <unordered_map>
#include <string>

namespace lazyECS {

    enum Shape {POINT, BOX, SPHERE, CAPSULE, CYLINDER, CONVEX, CONCAVE, HEIGHTFIELD};

    // Abstract class to provide a base for the rest of the shapes
    class Mesh {
    
    protected:
        // These are the local properties of the mesh in the model space
        raylib::Vector3 position;
        raylib::Vector3 rotation;
        raylib::Vector3 scale;
        raylib::Color color;
        Shape shape;
        std::unordered_map<std::string, std::any> params;

    public:
        Mesh(const Shape& shape_in) : position(raylib::Vector3(0,0,0)), rotation(raylib::Vector3(0,0,0)), scale(raylib::Vector3(1,1,1)),
                                        shape(shape_in),
                                        color(raylib::RED) {}

    
        // it will be system's responsibility to check for the key name and type cast accordingly
        void SetMeshParam(const std::string& param_name, const std::any& param_value) {
            params[param_name] = param_value; 
        }

        std::any& GetMeshParam(const std::string& param_name) {
            return params[param_name];
        }

    };

}