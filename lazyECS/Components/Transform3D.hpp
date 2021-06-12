#pragma once

#include <reactphysics3d/reactphysics3d.h>
#include "Object3D.h"
#include "maths/Matrix4.h"

namespace lazyECS {

// Class to represent the position & orientation of a generic 3D entity in the scene
class Transform3D  {

public:
    
    // Transform for physics
    reactphysics3d::Transform rp3d_transform;

    // Transform from previous physics step
    reactphysics3d::Transform rp3d_prev_transform;    

    // Transform for graphics (rendering) --> local-space to World-space transformation matrix
    openglframework::Object3D opengl_transform; // mTransformMatrix

    // Scaling (data above has no scaling info)
    openglframework::Matrix4 mScalingMatrix;

    // Shape size information : TODO: handle multiple shapes
    float mSize[3];

    // Convert the physics Transform to Graphics (OpenGL) Transform format
    openglframework::Matrix4 ConvertRP3DToOpenglTransform(const float& interpolationFactor);

};

}

