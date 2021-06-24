#pragma once

#include "ECSCore/Types.hpp"
#include "Components/Tag.h"

namespace lazyECS {

class TagSystem : public System {

public:
    void SetupSignature();

    std::vector<Entity> GetEntitiesWithTag(const std::string& target_tag);
};

}