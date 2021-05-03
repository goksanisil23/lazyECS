#pragma once

#include "Types.hpp"

#include <array>
#include <unordered_map>
#include <vector>

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

// This class is a means to be able to hold multiple template types within the same container (Abstract Class)
class IComponentArrayHandler {
public:
    virtual ~IComponentArrayHandler() = default;
    // virtual void RemoveEntityFromCompArray(const Entity& entity) = 0;
    virtual void RemoveComponentFromEntity(const Entity& entity) = 0;
};

template<typename TComponentType>
class ComponentArrayHandler {

public:

    void AddComponentToEntity(const Entity& entity, TComponentType component) {
        
        // Only 1 of this component type per entity is allowed
        assert(m_entityComponentLookup.find(entity) == m_entityComponentLookup.end() && "This entity already has this component type!")
        // For this component type, owner(entity) count must be the same number of component instances (due to rule above)
        assert(m_entityArray.size() == m_componentArray.size() && "owner (entity) size does not match component size for this component type!")
        assert(m_entityComponentLookup.size() == m_componentArray.size() && " entity->Component lookup size does not match component size for this component type!")

        // Add the new component at the end of component array
        m_componentArray.push_back(component);    
        // Add the owner id at the end of entity array
        m_entityArray.push_back(entity);

        // Update lookup
        m_entityComponentLookup[entity] = m_componentArray.size()-1; // since its added to the back
    }

    void RemoveComponentFromEntity(const Entity& entity) override {
        auto it = m_entityComponentLookup.find(entity);

        if (it != m_entityComponentLookup.end()) {
            const size_t compIdxToRemove = it->second;
            
            // if not already last element in the array, swap the component to be removed
            // with the last component in the array to maintain the compactness
            if (compIdxToRemove < (m_componentArray.size()-1)) {
                m_componentArray[compIdxToRemove] = std::move(m_componentArray.back()); // avoid copying since structs might be big
                m_entityArray[compIdxToRemove] = m_entityArray.back();
                
                // update the lookup after swapping
                m_entityComponentLookup[m_entityArray[compIdxToRemove]] = compIdxToRemove; // the element moved from the back of the array, takes 
                                                                                           // removed element's index
            }

            // shrink the containers
            m_componentArray.pop_back();
            m_entityArray.pop_back();
            m_entityComponentLookup.erase(entity);
        }
    }

    TComponentType& GetComponentData(const Entity& entity) {
        assert(m_entityComponentLookup.find(entity) != m_entityComponentLookup.end() && "This component has no such owner(entity)");
    }

    // void RemoveEntityFromCompArray(const Entity& entity) override {
    //     if (m_entityComponentLookup[entity] != m_entityComponentLookup.end()) { // make sure this entity is an owner of this type of component
    //         this->RemoveComponentFromEntity(entity);
    //     }
    // }

private:
    std::vector<TComponentType> m_componentArray; // stores the instances for this component type
    
    std::vector<Entity> m_entityArray; // stores the owners of the component instances
                                       // component in m_componentArray[i] belongs to entity in m_entityArray[i], so they evolve in parallel
    
    std::unordered_map<Entity, size_t> m_entityComponentLookup; // maps the component instances to the entities
                                                                // WHY we need lookup if entityArray and componentArray evolve in parallel?
                                                                // Because: position along the arrays != id of entity

};