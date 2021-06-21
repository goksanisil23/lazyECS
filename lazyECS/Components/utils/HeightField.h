#pragma once

#include "PerlinNoise.h"
#include <vector>

namespace lazyECS {

class HeightField {

public:

    int mNumPtsWidth; 
    int mNumPtsLength;
    std::vector<float> mHeightFieldData;
    float mMinHeight;
    float mMaxHeight;

    HeightField(int num_pts_width, int num_pts_length);

    // height field generation methods
    void ApplyPerlinNoise();

    void Ramp();
};

}