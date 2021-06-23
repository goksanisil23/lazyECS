#pragma once

#include "Types.hpp"
#include "ComponentManager.hpp"
#include "EntityManager.hpp"
#include "SystemManager.hpp"

#include <memory>

namespace lazyECS {

// This class is to allow managers to talk to each other and bundling high level user functionality together.
class Orchestrator {

public:
    void Init() {
        m_componentManager = std::make_unique<ComponentManager>();
        m_entityManager = std::make_unique<EntityManager>();
        m_systemManager = std::make_unique<SystemManager>();
    }

    /////////////////////////// ENTITY METHODS ////////////////////////////////
    Entity CreateEntity() {
        return m_entityManager->CreateEntity();
    }

    void DestroyEntity(const Entity& entity) {
        m_componentManager->RemoveEntity(entity);
        m_systemManager->RemoveEntity(entity);
        m_entityManager->DestroyEntity(entity);
    }

    void SetEntityTag(const std::string& Tag, const Entity& entity) {
        m_entityManager->SetEntityTag(Tag, entity);
    }

    /////////////////////////// COMPONENT METHODS //////////////////////////////// 
    template<typename TComponentType>
    void RegisterComponentType() {
        m_componentManager->RegisterComponentType<TComponentType>();
    }

    template<typename TComponentType>
    void AddComponent(const Entity& entity, const TComponentType& component) {
        m_componentManager->AddComponent(entity, component);

        // Update the signature of this entity after adding the component
        Signature signature = m_entityManager->GetSignature(entity);
        // given that this component type has unique id "x", we set the x'th bit in the signature to "1"
        signature.set(m_componentManager->GetComponentTypeId<TComponentType>(), true);
        m_entityManager->SetSignature(entity, signature);

        m_systemManager->EntitySignatureChanged(entity, signature);
    }

    template<typename TComponentType>
    TComponentType& GetComponent(const Entity& entity){
        return m_componentManager->GetComponent<TComponentType>(entity);
    }

    template<typename TComponentType>
    bool CheckComponentExistsInEntity(const Entity& entity){
        return m_componentManager->CheckComponentExistsInEntity<TComponentType>(entity);
    }    

    template<typename TComponentType>
    ComponentTypeId GetComponentTypeId(){
        return m_componentManager->GetComponentTypeId<TComponentType>();
    }

    /////////////////////////// SYSTEM METHODS ///////////////////////////

    // Variadic template since each system will need different sets of parameters
    template<typename TSytemType, typename... TSystemInputTypes>
    std::shared_ptr<TSytemType> RegisterSystem(TSystemInputTypes... inputVars) {
        return m_systemManager->RegisterSystem<TSytemType>(inputVars...);
    }

    template<typename TSytemType>
    void SetSystemSignature(const Signature& signature) {
        m_systemManager->SetSignature<TSytemType>(signature);
    }


private:
    // only Coordinator is meant to deal with the managers, hence unique ptr
    std::unique_ptr<ComponentManager> m_componentManager;
    std::unique_ptr<EntityManager> m_entityManager;
    std::unique_ptr<SystemManager> m_systemManager;
};

}