#pragma once

#include "ECSCore/Types.hpp"

class PhysicsSystem : public System {
public:
    void Init();

    void Update(float dt);
};