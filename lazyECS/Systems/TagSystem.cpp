#include "TagSystem.h"
#include "ECSCore/Orchestrator.hpp"

#include <iostream>

extern lazyECS::Orchestrator gOrchestrator; // expected to be defined globally in main

namespace lazyECS {

void TagSystem::SetupSignature() {
    // Set the system signature based on the utilized Components below
    Signature signature;
    signature.set(gOrchestrator.GetComponentTypeId<Tag>(), true);
    gOrchestrator.SetSystemSignature<TagSystem>(signature);        
}

std::vector<Entity> TagSystem::GetEntitiesWithTag(const std::string& target_tag) {
    std::vector<Entity> entities_with_tag;

    for(const auto& entity : m_entities) {
        if(gOrchestrator.GetComponent<Tag>(entity).mTag == target_tag) {
            entities_with_tag.push_back(entity);
        }
    }

    return entities_with_tag;
}

}