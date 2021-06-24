#pragma once

#include <string>

namespace lazyECS {

// Tag component is used to group entities in logical groups for easier scenario design & control
// Not all entities need to have a Tag
class Tag {

public:
    std::string mTag;

    Tag();

    explicit Tag(const std::string& tag);
};

}