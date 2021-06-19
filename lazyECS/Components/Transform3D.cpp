#include "Transform3D.hpp"

namespace lazyECS {

Transform3D::Transform3D(const rp3d::Vector3& init_pos, const rp3d::Quaternion& init_rot) :
    mScalingMatrix{openglframework::Matrix4::identity()} {
    rp3d_transform = reactphysics3d::Transform(init_pos, init_rot);
    rp3d_prev_transform = rp3d_transform;
}

void Transform3D::ConvertRP3DToOpenglTransformInterp(const float& interpolationFactor) {
    
    // Interpolate the graphics transform between the previous one and the new one
    rp3d::Transform interpTrans = rp3d::Transform::interpolateTransforms(rp3d_prev_transform, rp3d_transform, interpolationFactor);
    rp3d_prev_transform = rp3d_transform; // update the previous transform
    
    // Computer the transform used for rendering
    rp3d::decimal temp_matrix[16];
    interpTrans.getOpenGLMatrix(temp_matrix);
    openglframework::Matrix4 newInterpMatrix(temp_matrix[0], temp_matrix[4], temp_matrix[8],  temp_matrix[12],
                                             temp_matrix[1], temp_matrix[5], temp_matrix[9],  temp_matrix[13],
                                             temp_matrix[2], temp_matrix[6], temp_matrix[10], temp_matrix[14],
                                             temp_matrix[3], temp_matrix[7], temp_matrix[11], temp_matrix[15]);
    // Apply the scaling matrix
    opengl_transform.setTransformMatrix(newInterpMatrix * mScalingMatrix);
}

void Transform3D::ConvertRP3DToOpenglTransform() {
        
    // Compute the transform used for rendering
    rp3d::decimal temp_matrix[16];
    rp3d_transform.getOpenGLMatrix(temp_matrix); // (assuming already interpolated in the Physics System)
    openglframework::Matrix4 newTransMatrix(temp_matrix[0], temp_matrix[4], temp_matrix[8],  temp_matrix[12],
                                             temp_matrix[1], temp_matrix[5], temp_matrix[9],  temp_matrix[13],
                                             temp_matrix[2], temp_matrix[6], temp_matrix[10], temp_matrix[14],
                                             temp_matrix[3], temp_matrix[7], temp_matrix[11], temp_matrix[15]);
    // Apply the scaling matrix
    opengl_transform.setTransformMatrix(newTransMatrix * mScalingMatrix);
}

void Transform3D::SetScale(const float& xScale, const float& yScale, const float& zScale) {
    // Set the half-scale for rp3d collision shape creation (in Physics system)
    this->halfExtent[0] = xScale*0.5f;
    this->halfExtent[1] = yScale*0.5f;
    this->halfExtent[2] = zScale*0.5f;
    

    // Set the scaling matrix for the openglframework
    this->mScalingMatrix = openglframework::Matrix4(this->halfExtent[0], 0, 0, 0,
                                                    0, this->halfExtent[1], 0, 0,
                                                    0, 0, this->halfExtent[2], 0,
                                                    0, 0, 0, 1);

    // let the scaling take effect
    this->ConvertRP3DToOpenglTransform();
}


}