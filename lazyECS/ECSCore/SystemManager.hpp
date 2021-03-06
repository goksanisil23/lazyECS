#pragma once 

#include "Types.hpp"

#include <memory>
#include <unordered_map>
#include <typeinfo>
#include <typeindex>
#include <cassert>

namespace lazyECS {

class SystemManager{
public:

    // Variadic template since each system will need different sets of parameters
    template<typename TSystemType, typename... TSystemInputTypes>
    std::shared_ptr<TSystemType> RegisterSystem(TSystemInputTypes... inputVars) {
        std::type_index typIdx = std::type_index(typeid(TSystemType));

        assert(m_Systems.find(typIdx) == m_Systems.end() && "This system is already registered!");

        auto system = std::make_shared<TSystemType>(inputVars...);
        m_Systems[typIdx] = system;
        return system;
    }

    template<typename TSystemType>
    void SetSignature(const Signature& signature){
        std::type_index typIdx = std::type_index(typeid(TSystemType));
        assert(m_Systems.find(typIdx) != m_Systems.end() && "This system does not exist!");

        m_systemSignatures[typIdx] = signature;
    }

    // when an entity is destroyed, entity is removed from all systems
    void RemoveEntity(const Entity& entity) {
        for (auto const& pair : m_Systems) {
            auto const& system = pair.second;
            system->m_entities.erase(entity); 
        }
    }

    void EntitySignatureChanged(const Entity& entity, const Signature& newEntitySignature) {
        for (auto const& pair : m_Systems) {
            auto const& sysType = pair.first;
            auto const& system = pair.second;
            auto const& sysSig = m_systemSignatures[sysType];

            // Note that entity signature and system signature do not have to match exactly
            // For a system to act on entity, system needs to contain a SUBSET of the entity signature (components)
            if ( (newEntitySignature & sysSig) == sysSig) {
                system->m_entities.insert(entity);
            }
            else { // no need to check if the entity is existing in the System or not, since unordered_set allows erasing non-existent keys
                system->m_entities.erase(entity);
            }
        }
    }

private:
    std::unordered_map<std::type_index, Signature> m_systemSignatures{};

    std::unordered_map<std::type_index, std::shared_ptr<System>> m_Systems{};

};

}