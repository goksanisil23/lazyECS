#pragma once

#include "Types.hpp"

#include <array>
#include <unordered_map>
#include <vector>
#include <cassert>
#include <iostream>

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



namespace lazyECS {


// This class is a means to be able to hold multiple template types (ComponentArrayHandlers) within the same container (Abstract Class)
class IComponentArrayHandler {
public:
    virtual ~IComponentArrayHandler() = default;
    // virtual void RemoveEntityFromCompArray(const Entity& entity) = 0;
    virtual void RemoveComponentFromEntity(const Entity& entity) = 0;
};

template<typename TComponentType>
class ComponentArrayHandler : public IComponentArrayHandler {

public:

    void AddComponentToEntity(const Entity& entity, TComponentType component) {
        
        // Only 1 of this component type per entity is allowed
        assert(this->m_entityComponentLookup.find(entity) == this->m_entityComponentLookup.end() && "This entity already has this component type!");
        // For this component type, owner(entity) count must be the same number of component instances (due to rule above)
        assert(this->m_entityArray.size() == this->m_componentArray.size() && "owner (entity) size does not match component size for this component type!");
        assert(this->m_entityComponentLookup.size() == this->m_componentArray.size() && " entity->Component lookup size does not match component size for this component type!");

        // Add the new component at the end of component array
        this->m_componentArray.push_back(component);    
        // Add the owner id at the end of entity array
        this->m_entityArray.push_back(entity);

        // Update lookup
        this->m_entityComponentLookup[entity] = this->m_componentArray.size()-1; // since its added to the back
    }

    void RemoveComponentFromEntity(const Entity& entity) override {
        auto it = this->m_entityComponentLookup.find(entity);

        if (it != this->m_entityComponentLookup.end()) {
            const size_t compIdxToRemove = it->second;
            
            // if not already last element in the array, swap the component to be removed
            // with the last component in the array to maintain the compactness
            if (compIdxToRemove < (this->m_componentArray.size()-1)) {
                this->m_componentArray[compIdxToRemove] = std::move(this->m_componentArray.back()); // avoid copying since structs might be big
                this->m_entityArray[compIdxToRemove] = this->m_entityArray.back();
                
                // update the lookup after swapping
                this->m_entityComponentLookup[this->m_entityArray[compIdxToRemove]] = compIdxToRemove;  
                                                            // the element moved from the back of the array, takes removed element's index
            }

            // shrink the containers
            this->m_componentArray.pop_back();
            this->m_entityArray.pop_back();
            this->m_entityComponentLookup.erase(entity);
        }
    }

    TComponentType& GetComponentData(const Entity& entity) {
        assert(this->m_entityComponentLookup.find(entity) != this->m_entityComponentLookup.end() && "This component has no such owner(entity)");

        return this->m_componentArray[this->m_entityComponentLookup[entity]];
    }

private:
    std::vector<TComponentType> m_componentArray{}; // stores the instances for this component type
    
    std::vector<Entity> m_entityArray{}; // stores the owners of the component instances
                                       // component in m_componentArray[i] belongs to entity in m_entityArray[i], so they evolve in parallel
    
    std::unordered_map<Entity, size_t> m_entityComponentLookup{}; // maps the component instances to the entities
                                                                // WHY we need lookup if entityArray and componentArray evolve in parallel?
                                                                // Because: position along the arrays != id of entity

};

}