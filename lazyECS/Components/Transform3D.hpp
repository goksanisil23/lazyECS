#pragma once

#include <reactphysics3d/reactphysics3d.h>

namespace lazyECS {

// Class to represent the position & orientation of a generic 3D entity in the scene
class Transform3D : public reactphysics3d::Transform {

public:
    
    // Transform for physics
    std::shared_ptr<reactphysics3d::Transform> rp3d_transform;

};

}

