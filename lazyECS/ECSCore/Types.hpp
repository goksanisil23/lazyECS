# pragma once

#include <bitset>
#include <unordered_set>
#include <json.hpp>

#include "ECSConstants.hpp"

using json = nlohmann::json;

// ---------- ECS types ---------- //

namespace lazyECS {

using Entity = std::uint32_t; // Entity is nothing but a unique id

using ComponentTypeId = std::uint8_t; // each unique component type is defined by a unique number

using Signature = std::bitset<MAX_COMPONENT_TYPES>;  // a signature contains information about which components exist/used in that Entity or System
                                                     // If a Component with unique id (ComponentTypeId) "n" exists in that System or Entity,
                                                     // "n"th bit of the Signature is set to 1.

// System is just a functionality that iterates along a set of entities that fit into a certain Signature (of component)
// Which entities are stored for this System, is handled by the SystemManager, based on if (entity signature == system signature)
class System {             
public:
    std::unordered_set<Entity> m_entities;
};

}

