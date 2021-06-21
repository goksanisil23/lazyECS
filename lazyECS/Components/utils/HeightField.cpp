#include "HeightField.h"
#include <iostream>

namespace lazyECS {

HeightField::HeightField(int num_pts_width, int num_pts_length) : mNumPtsWidth{num_pts_width}, mNumPtsLength{num_pts_length},
                                                                  mHeightFieldData{std::vector<float>(mNumPtsWidth*mNumPtsLength)} {

    ApplyPerlinNoise();
    // Ramp();
}

void HeightField::Ramp() {

    for(int i = 0; i < mNumPtsWidth; i++) {
        for(int j = 0; j < mNumPtsLength; j++) {
            int arrayIndex = j * mNumPtsWidth + i;
            if((i==0) && (j==0)) {
                this->mMinHeight = static_cast<float>((i+j)*10);
                this->mMaxHeight = static_cast<float>((i+j)*10);                
            }
            mHeightFieldData[arrayIndex] = static_cast<float>((i+j)*10);
            if(mHeightFieldData[arrayIndex] > mMaxHeight)
                mMaxHeight = mHeightFieldData[arrayIndex];
            if(mHeightFieldData[arrayIndex] < mMinHeight)
                mMinHeight = mHeightFieldData[arrayIndex];                
        }
    }      


}

void HeightField::ApplyPerlinNoise() {

    // Create Perlin Noise based height field
    double persistence = 9;
    double frequency = 0.28;
    double amplitude = 12;
    int octaves = 1;
    int randomseed = 23;
    PerlinNoise perlinNoise(persistence, frequency, amplitude, octaves, randomseed);

    float width = mNumPtsWidth -1;
    float length = mNumPtsLength -1;

    // Will be updated below
    this->mMinHeight = static_cast<float>(perlinNoise.GetHeight(-width, -length));
    this->mMaxHeight = static_cast<float>(perlinNoise.GetHeight(-width, -length));

    for(int i = 0; i < mNumPtsWidth; i++) {
        for(int j = 0; j < mNumPtsLength; j++) {
            int arrayIndex = j * mNumPtsWidth + i;
            mHeightFieldData[arrayIndex] = static_cast<float>(perlinNoise.GetHeight(-width*0.5+i, -length*0.5+j));
            if(mHeightFieldData[arrayIndex] > mMaxHeight)
                mMaxHeight = mHeightFieldData[arrayIndex];
            if(mHeightFieldData[arrayIndex] < mMinHeight)
                mMinHeight = mHeightFieldData[arrayIndex];                
        }
    }    
}

}