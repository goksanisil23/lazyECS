# pragma once

#include <bitset>
#include <cstdint>
#include <unordered_set>
#include <json.hpp>

using json = nlohmann::json;

constexpr uint32_t MAX_ENTITIES = 5000;
constexpr uint32_t MAX_COMPONENT_TYPES = 32;

// ---------- ECS types ---------- //

namespace lazyECS {

using Entity = std::uint32_t; // Entity is nothing but a unique id

using ComponentTypeId = std::uint8_t; // each unique component type is defined by a unique number

using Signature = std::bitset<MAX_COMPONENT_TYPES>;  // a signature contains information about which components exist/used in that Entity or System
                                                     // If a Component with unique id (ComponentTypeId) "n" exists in that System or Entity,
                                                     // "n"th bit of the Signature is set to 1.

// System is just a functionality that iterates along a set of entities that fit into a certain Signature (of component)
// Which entities are stored for this System, is handled by the SystemManager, based on if system signature is a subset of entity signature
class System {             
public:
    std::unordered_set<Entity> m_entities;
};

// Events (defined here instead of inside systems to that systems do not need to know about each other)

struct Event {
    virtual ~Event() = default;
};


struct KeyboardEvent : public Event {
    enum KeyButton {
        KEY_A, KEY_S, KEY_D, KEY_W
    };

    KeyButton key_button;
    
    explicit KeyboardEvent(const KeyButton key_but) : key_button(key_but) {}
};


}

