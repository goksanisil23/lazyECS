#pragma once

#include <reactphysics3d/reactphysics3d.h>
#include <vector>

namespace lazyECS {

// Just a wrapper class for the RP3D rigid body component
class RigidBody3D {

public:
      
    std::shared_ptr<rp3d::RigidBody> rp3d_rigidBody; // uninitialized. PhysicsSystem needs to initialize via world->createRigidBody(Transform)
    std::vector<std::shared_ptr<rp3d::Collider>> rp3d_colliders; // uninitialized. PhysicsSystem needs to initialize via rp3d::RigidBody::addCoolider
    rp3d::CollisionShape* rp3d_collision_shape; //uninitialized PhysicsSystem needs to initialize via rp3d::PhysicsCommon::createBoxShape

    rp3d::BodyType rp3d_bodyType; // STATIC: infinite mass, zero vel., manually movable, only collides with DYNAMIC
                                  // KINEMATIC: Infinite mass, manually adjusted velocity, only collides with DYNAMIC
                                  // DYNAMIC: non-zero mass, velocity determined by forces, collides with ALL
    float bounciness;

};

}