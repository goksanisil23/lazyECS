#pragma once

#include <queue>
#include <array>
#include <unordered_map>
#include <assert.h>


#include "Types.hpp"

namespace lazyECS {

// EntityManager is responsible of assigning a unique id for each newcomer,
// and keeping a track of available/occupied entities (id's) in the scene
class EntityManager {

public:

    // Create a queue with all possible Entity id's
    EntityManager() {
        for (Entity entity = 0; entity < MAX_ENTITIES; entity++) {
            m_availableEntities.push(entity);
        }
    }

    Entity CreateEntity() {
        assert(m_spawnedEntityCount < MAX_ENTITIES && "Maximum number of Entities has been reached !");

        Entity new_entity = m_availableEntities.front();
        m_availableEntities.pop(); // remove the spawned entity id from the queue
        m_spawnedEntityCount++;

        return new_entity;
    }

    void DestroyEntity(const Entity& entity) {
        assert(entity < MAX_ENTITIES && "Entity to be destroyed is out of range !");
        assert(m_spawnedEntityCount > 0 && "No entity left to destroy !");

        m_entitySignatures[entity].reset(); // since this entity is destroyed, no components will be attached to that entity id anymore
        m_availableEntities.push(entity);
        m_spawnedEntityCount--;
    }

    void SetSignature(const Entity& entity, const Signature& signature) {
        assert(entity < MAX_ENTITIES && "Entity to receive signature is out of range !");
    
        m_entitySignatures[entity] = signature;
    }

    Signature GetSignature(const Entity& entity) {
        assert(entity < MAX_ENTITIES && "Entity whose signature to be retrieved is out of range !");
    
        return m_entitySignatures[entity];
    }

    void SetEntityTag(const std::string& Tag, const Entity& entity) {
        m_entityTags.insert(std::make_pair(Tag, entity));
    }

private:

    std::queue<Entity> m_availableEntities {}; // unoccupied Entity id's. Initially, queue is nicely ordered [0,1,..., (MAX_ENTITIES-1)]
                                               // When an Entity is added to the scene, first id from the top of the queue is picked
                                               // When an Entity is destroyed, that id (Entity) is pushed back of the queue 
    
    std::array<Signature, MAX_ENTITIES> m_entitySignatures{}; // each Entity contains a set of Components, whose id's are encoded in the Signature bitset

    uint32_t m_spawnedEntityCount{}; // number of entities alive in the current scene

    std::unordered_map<std::string, Entity> m_entityTags; // used to group entities in logical groups for easier scenario design & control

};

}