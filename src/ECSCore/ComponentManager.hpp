#pragma once

#include "Types.hpp"
#include "ComponentArrayHandler.hpp"

#include <unordered_map>
#include <memory>
#include <typeinfo>
#include <typeindex>


class ComponentManager {

public:
    template<typename TComponentType>
    void RegisterComponentType() {
        std::type_index typIdx = std::type_index(typeid(TComponentType));
        
        assert(m_componentTypes.find(typIdx) != m_componentTypes.end() && "Component type already registered!");

        m_componentTypes[typIdx] = m_nextCompTypeId;

        // also create a ComponentArrayHandler for this component type
        m_componentArrayHandlers[typIdx] = std::make_shared<ComponentArrayHandler<TComponentType>>();

        m_nextCompTypeId++;
    }

    template<typename TComponentType>
    ComponentTypeId GetComponentTypeId() {
        std::type_index typIdx = std::type_index(typeid(TComponentType));
        assert(m_componentTypes.find(typIdx) != m_componentTypes.end() && "Component type is not registered!");
        return m_componentTypes[typIdx];
    }

    template<typename TComponentType>
    void AddComponent(const Entity& entity, const TComponentType& component) {
        GetComponentArrayHandler<TComponentType>()->AddComponentToEntity(entity, std::move(component));
    }

    template<typename TComponentType>
    void RemoveComponent(const Entity& entity) {
        GetComponentArrayHandler<TComponentType>()->RemoveComponentFromEntity(entity);
    }

    template<typename TComponentType>
    TComponentType& GetComponent(const Entity& entity) {
        GetComponentArrayHandler<TComponentType>()->GetComponentData(entity);
    }

    // Removes the entity from all the component arrays for all types of components
    void RemoveEntity(const Entity& entity) {
        for (auto const& pair : m_componentArrayHandlers) {
            auto const& compArrayHandl = pair.second;
            compArrayHandl->RemoveComponentFromEntity(entity);
        }
    }

private:

    // mapping the component type to a unique component id number
    std::unordered_map<std::type_index, ComponentTypeId> m_componentTypes{};

    // we store a pointer to base class instead of class itself to avoid object slicing
    // mapping each component type to their array handler
    std::unordered_map<std::type_index, std::shared_ptr<IComponentArrayHandler>> m_componentArrayHandlers{};
    
    ComponentTypeId m_nextCompTypeId; // component types's are just unique numbers
                                      // we need this number to indicate component ownership inside Signatures


    // returns the specific componentArrayHandler for this component type
    template<typename TComponentType>
    std::shared_ptr<ComponentArrayHandler<TComponentType>> GetComponentArrayHandler() {
        std::type_index typIdx = std::type_index(typeid(TComponentType));

        assert(m_componentTypes.find(typIdx) != m_componentTypes.end() && "Component type is not registered!");

        // need to cast from parent to child since the function is templated with the specific component type
        return std::static_pointer_cast<ComponentArrayHandler<TComponentType>>(m_componentArrayHandlers[typIdx]);
    }

};