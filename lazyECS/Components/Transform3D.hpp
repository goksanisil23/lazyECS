#pragma once

#include <reactphysics3d/reactphysics3d.h>

namespace lazyECS {

class Transform3D : public reactphysics3d::Transform {

public:
    // This is uninitialized. PhysicsSystem needs to initialize via world->createRigidBody(Transform)  
    std::shared_ptr<reactphysics3d::Transform> rp3d_transform; 

};

}

