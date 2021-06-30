#include "PhysicsSystem.hpp"

#include "ECSCore/Orchestrator.hpp"

#include <iostream>

extern lazyECS::Orchestrator gOrchestrator; // expected to be defined globally in main

namespace lazyECS {

PhysicsSystem::PhysicsSystem() : timeStep{PHYSICS_TIME_STEP}, timeAccumulator{0.0F}, 
                                prevFrameTime{std::chrono::high_resolution_clock::now()} 
                                {}


void PhysicsSystem::SetupSignature() {
    // Set the system signature based on the utilized Components below
    Signature signature;
    signature.set(gOrchestrator.GetComponentTypeId<RigidBody3D>(), true);
    signature.set(gOrchestrator.GetComponentTypeId<Transform3D>(), true);
    signature.set(gOrchestrator.GetComponentTypeId<Mesh>(), true);
    gOrchestrator.SetSystemSignature<PhysicsSystem>(signature);    
}

void PhysicsSystem::Init(){

    // Setup the React 3D Physics World
    rp3d::PhysicsWorld::WorldSettings settings; // default settings    
    this->physicsWorld = physicsCommon.createPhysicsWorld(settings);

    // Create rp3d rigid body for the entities that have rigid body components
    // Shape of the collider is obtained from the size of the Transform
    for(const auto& entity : m_entities) {
        auto& rigidBody = gOrchestrator.GetComponent<RigidBody3D>(entity); // uninitialized here
        auto& transform = gOrchestrator.GetComponent<Transform3D>(entity); // Initial Transform of entities given outside Physics system 
        auto& mesh = gOrchestrator.GetComponent<Mesh>(entity); // we get the shape of the mesh to create a matching collider
        // we assign the actual rigidBody here, whose initial position is determined by what was assigned to lazyECS::Transform component outside
        rigidBody.rp3d_rigidBody = std::shared_ptr<rp3d::RigidBody>(this->physicsWorld->createRigidBody(transform.rp3d_transform));
        // Assign collider based on the shape
        switch(mesh.mShape) {
            case Shape::Box:
                rigidBody.rp3d_collision_shape = physicsCommon.createBoxShape(rp3d::Vector3(transform.halfExtent[0], transform.halfExtent[1], transform.halfExtent[2]));
                break;
            case Shape::Sphere:
                rigidBody.rp3d_collision_shape = physicsCommon.createSphereShape(transform.halfExtent[0]);
                break;
            case Shape::Capsule:
                rigidBody.rp3d_collision_shape = physicsCommon.createCapsuleShape(transform.halfExtent[0],transform.halfExtent[1]);
                break;                
            case Shape::ConcaveMesh: {
                auto mPhysicsTriangleMesh = physicsCommon.createTriangleMesh();
                for(unsigned int i = 0; i < mesh.getNbParts(); i++) {
                    // Vertex and index array for the triangle mesh (data is shared from mesh)
                    rp3d::TriangleVertexArray* vertexArray = 
                        new rp3d::TriangleVertexArray(mesh.getNbVertices(), &(mesh.getVertex(0)), sizeof(openglframework::Vector3), 
                                                      mesh.getNbFaces(i), &(mesh.getIndices(i)[0]), 3 * sizeof(int),
                                                      rp3d::TriangleVertexArray::VertexDataType::VERTEX_FLOAT_TYPE,
                                                      rp3d::TriangleVertexArray::IndexDataType::INDEX_INTEGER_TYPE);
                    // Add this triangle vertex array subpart to triangle mesh
                    mPhysicsTriangleMesh->addSubpart(vertexArray); 
                }
                // Create a collider from the triangle mesh
                auto collShape = physicsCommon.createConcaveMeshShape(mPhysicsTriangleMesh);
                rigidBody.rp3d_collision_shape = physicsCommon.createConcaveMeshShape(mPhysicsTriangleMesh, rp3d::Vector3(transform.halfExtent[0],
                                                                                                                          transform.halfExtent[1],
                                                                                                                          transform.halfExtent[2]));

                break;
            }
            case Shape::Hfield: {
                float height_data[mesh.mHeightField->mNumPtsWidth*mesh.mHeightField->mNumPtsLength];
                std::copy(mesh.mHeightField->mHeightFieldData.begin(),mesh.mHeightField->mHeightFieldData.end(), height_data);
                rigidBody.rp3d_collision_shape = physicsCommon.createHeightFieldShape(mesh.mHeightField->mNumPtsWidth, 
                                                                                      mesh.mHeightField->mNumPtsLength,
                                                                                      mesh.mHeightField->mMinHeight,
                                                                                      mesh.mHeightField->mMaxHeight,
                                                                                      height_data,
                                                                                      rp3d::HeightFieldShape::HeightDataType::HEIGHT_FLOAT_TYPE);                 
                break;
            }
            default:
                assert(! "Invalid Shape enum in PhysicsSystem");
                break;
        }

        rigidBody.rp3d_collider = std::shared_ptr<rp3d::Collider>(rigidBody.rp3d_rigidBody->addCollider(rigidBody.rp3d_collision_shape, rp3d::Transform::identity()));
        rigidBody.rp3d_rigidBody->updateMassPropertiesFromColliders();            
        rigidBody.rp3d_rigidBody->setType(rigidBody.rp3d_bodyType);
    }
}

const float& PhysicsSystem::GetInterpFactor() {
    return interpFactor;
}

void PhysicsSystem::Update() {

    auto current_frame_time = std::chrono::high_resolution_clock::now();
    auto delta_time = std::chrono::duration<float, std::chrono::seconds::period>(current_frame_time-prevFrameTime).count();
    this->prevFrameTime = current_frame_time; // update previous time
    this->timeAccumulator += delta_time;

    while(this->timeAccumulator >= this->timeStep) {

        // Update physics world with constant time step
        this->physicsWorld->update(this->timeStep);

        // Decrease the accumulated time
        this->timeAccumulator -= this->timeStep;
    }

    // Compute time interpolation factor
    this->interpFactor = this->timeAccumulator / this->timeStep;

    for (auto const& entity : m_entities) {

        int ent_ctr = 0;
        for(const auto& entity : m_entities) {
            auto& rigid_body = gOrchestrator.GetComponent<RigidBody3D>(entity); // updated in the physicsWorld update above
            auto& transform = gOrchestrator.GetComponent<Transform3D>(entity);

            // get the updated transform of the body
            rp3d::Transform current_trans = rigid_body.rp3d_rigidBody->getTransform(); 

            // compute the interpolated transform of the rigid body based on the leftover time
            rp3d::Transform interp_trans = rp3d::Transform::interpolateTransforms(transform.rp3d_prev_transform, current_trans, interpFactor);

            // use the interpolated transform as the final transform value for the entity (for rendering, AI, etc)
            transform.rp3d_transform = interp_trans;

            // Update the previous transform
            transform.rp3d_prev_transform = current_trans;

            // Debug print
            // const reactphysics3d::Vector3& position = transform.rp3d_transform.getPosition();
            // float rot_angle;
            // rp3d::Vector3 rot_axis;
            // transform.rp3d_transform.getOrientation().getRotationAngleAxis(rot_angle, rot_axis);
            // std::cout << "entity: " << ent_ctr << " position: " << position.x << " " << position.y << " " << position.z << std::endl;
            // std::cout << "entity: " << ent_ctr << " rot angle: " << rot_angle << "axis: " << rot_axis.x << " " << rot_axis.y << " " << rot_axis.z << std::endl;
            ent_ctr++;
        }
    }
}

}




