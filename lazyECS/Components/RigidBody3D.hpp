#pragma once

#include <reactphysics3d/reactphysics3d.h>

namespace lazyECS {

// Just a wrapper class for the RP3D rigid body component
class RigidBody3D {

public:
      
    std::shared_ptr<rp3d::RigidBody> rp3d_rigidBody; // uninitialized. PhysicsSystem needs to initialize via world->createRigidBody(Transform)
    std::shared_ptr<rp3d::Collider> rp3d_collider; // uninitialized. PhysicsSystem needs to initialize via rp3d::RigidBody::addCoolider
    rp3d::CollisionShape* rp3d_collision_shape; //uninitialized PhysicsSystem needs to initialize via rp3d::PhysicsCommon::createBoxShape

    bool isStatic;
    float bounciness;

};

}