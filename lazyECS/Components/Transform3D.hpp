#pragma once

#include <reactphysics3d/reactphysics3d.h>
#include "openglframework.h"

namespace lazyECS {

// Class to represent the position & orientation of a generic 3D entity in the scene
class Transform3D  {

public:
    
    // Transform for physics
    reactphysics3d::Transform rp3d_transform;

    // Transform from previous physics step
    reactphysics3d::Transform rp3d_prev_transform;    

    // Transform for graphics (rendering) --> local-space to World-space transformation matrix
    openglframework::Object3D opengl_transform;

    // Convert the physics Transform to Graphics (OpenGL) Transform format
    openglframework::Matrix4 ConvertRP3DToOpenglTransform(const float& interpolationFactor, const openglframework::Matrix4& scalingMatrix);

};

openglframework::Matrix4 Transform3D::ConvertRP3DToOpenglTransform(const float& interpolationFactor, const openglframework::Matrix4& scalingMatrix) {
    
    // Interpolate the graphics transform between the previous one and the new one
    rp3d::Transform interpTrans = rp3d::Transform::interpolateTransforms(rp3d_prev_transform, rp3d_prev_transform, interpolationFactor);
    rp3d_prev_transform = rp3d_transform; // update the previous transform
    
    // Computer the transform used for rendering
    rp3d::decimal temp_matrix[16];
    interpTrans.getOpenGLMatrix(temp_matrix);
    openglframework::Matrix4 newInterpMatrix(temp_matrix[0], temp_matrix[4], temp_matrix[8],  temp_matrix[12],
                                             temp_matrix[1], temp_matrix[5], temp_matrix[9],  temp_matrix[13],
                                             temp_matrix[2], temp_matrix[6], temp_matrix[10], temp_matrix[14],
                                             temp_matrix[3], temp_matrix[7], temp_matrix[11], temp_matrix[15]);
    // Apply the scaling matrix
    return newInterpMatrix * scalingMatrix;
}

}

