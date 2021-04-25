# pragma once

#include <bitset>

//////////// ECS types /////////////

using Entity = std::uint32_t; // Entity is nothing but a unique id

using ComponentType = std::uint8_t; // each unique component type is defined by a unique number

using Signature = std::bitset<MAX_COMPONENTS>;  // a signature contains information about which components exist/used in that Entity or System
                                                // If a Component with unique id (ComponentType) "n" exists in that System or Entity,
                                                // "n"th bit of the Signature is set to 1.

const Entity MAX_ENTITIES = 5000;
const ComponentType MAX_COMPONENTS = 32;