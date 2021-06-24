#include "Tag.h"

namespace lazyECS {

Tag::Tag() = default;

Tag::Tag(const std::string& tag) : mTag(tag) {}

}