#pragma once

#include "ECSCore/Types.hpp"


namespace lazyECS {

class PhysicsSystem : public System {
public:
    void Init();

    void Update(float dt);
};

}