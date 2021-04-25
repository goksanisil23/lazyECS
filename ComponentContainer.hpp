#pragma once

#include "Types.hpp"

#include <array>
#include <unordered_map>

// A Component is plain-old-data (POD) structure
// e.g. struct Transform {Vec3 pos, Vec3 rot, Vec3 scale}

// ComponentArray is an array of instances of the same Component type
// e.g std::array<Transform, MAX_ENTITIES> ComponentArray;

// When Entities "attach" a Component, the instantiation of that struct (Component) is stored in the ComponentArray
// Since this component instance belongs specifically to a certain entity, we need to relate this component instance to the owner entity.
// Since the component instances are stored in the array, an easy way to keep track of owners(entities) and belongings(components) is
// to store this relationship in both ways inside "maps".

// ComponentArrayHandler is the class responsible of doing the entity <-> component mapping maintenance, and keeping ComponentArray "packed"

// WHY: As the systems iterate through each component instance along a ComponentArray, its important to keep data "packed" in order to minimize cache miss

template<typename TComponentType>
class ComponentArrayHandler {

public:
    ComponentArrayHandler(): m_size(0) {}

    void AddComponentToEntity(const Entity& entity, TComponentType component) {
        // Add the new component at the end of the array
        m_entityToComponentArrayIdx[entity] = m_size;
        m_size++; 
    }

    void RemoveComponentFromEntity(const Entity& entity) {
        // Move the component at the end of the array into the deleted component's index to maintain the density
        size_t idxOfRemovedComp = m_entityToComponentArrayIdx[entity];
        m_componentArray[idxOfRemovedComp] = m_componentArray[m_size-1]; // here, we don't know which entity's component we're moving within the array
                                                                         // That's why, we need a 2nd map to do CompArrayIdx -> Entity
        // Update the entity -> component array idx map
        Entity entityOfLastCompArrayElement = m_compArrayIdxToEntity[m_size-1]; // we get the entity of the component that we moved from the end of the array
        m_entityToComponentArrayIdx[entityOfLastCompArrayElement] = idxOfRemovedComp; // update map
        m_compArrayIdxToEntity[idxOfRemovedComp] = entityOfLastCompArrayElement;

        m_entityToComponentArrayIdx.erase(entity);
        m_compArrayIdxToEntity.erase(m_size-1);

        m_size--;
        
    }



private:
    std::array<TComponentType, MAX_ENTITIES> m_componentArray{}; // container of this Component Type's instances

    std::unordered_map<Entity, size_t> m_entityToComponentArrayIdx{}; // mapping between the instance inside ComponentArray and it's owner Entity
    std::unordered_map<size_t, Entity> m_compArrayIdxToEntity{}; // mirror of the above

    size_t m_size; // number of valid Entities in the ComponentArray --> all indices smaller than this should be occupied
};