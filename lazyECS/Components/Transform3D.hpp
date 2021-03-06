#pragma once

#include <reactphysics3d/reactphysics3d.h>
#include "Object3D.h"
#include "maths/Matrix4.h"

namespace lazyECS {

// Class to represent the position & orientation of a generic 3D entity in the scene
// It contains 2 realizations of Transforms in different representation: 1 for physics and 1 for graphics
// In case an entity is not moved by physics, but teleported instead, its still more convenient to describe the
// body motion in reactphysics3d::Transform rather than openglframework TransformMatrix
// When the body is moved by react physics, rp3d_transform in lazyECS::Transform3D will be updated by lazyECS::RigidBody3D's transform component
// When the body is moved kinematically, no need to update rp3d_prev_transform

class Transform3D  {

public:

    Transform3D(const rp3d::Vector3& init_pos, const rp3d::Quaternion& init_rot);
    
    // Transform for physics
    reactphysics3d::Transform rp3d_transform;

    // Transform from previous physics step
    reactphysics3d::Transform rp3d_prev_transform;    

    // Transform for graphics (rendering) --> local-space (model-space) to World-space transformation matrix
    openglframework::Object3D opengl_transform; // mTransformMatrix

    // Scaling (data above has no scaling info)
    openglframework::Matrix4 mScalingMatrix;

    // Shape size information : TODO: handle multiple shapes
    float halfExtent[3]; // used by PhysicsSystem to create collision shape accordingly

    // Convert the physics Transform to Graphics (OpenGL) Transform format
    void ConvertRP3DToOpenglTransform();
    void ConvertRP3DToOpenglTransformInterp(const float& interpolationFactor);

    // scales the 3d shape in 3 directions : TODO: handle multiple shapes
    // Note: This sets the absolute scale, does NOT amplify the existing size
    // @param xScale : total length of x side
    // @param yScale : total length of y side
    // @param zScale : total length of z size 
    void SetScale(const float& xScale, const float& yScale, const float& zScale);

    // Returns the Euler angles from the native quaternion orientation
    std::tuple<float, float, float> GetEulerOrientation() const;
    
};

}

