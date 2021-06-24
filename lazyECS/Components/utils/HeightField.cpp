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
            int array_index = j * mNumPtsWidth + i;
            if((i==0) && (j==0)) {
                this->mMinHeight = static_cast<float>((i+j)*10);
                this->mMaxHeight = static_cast<float>((i+j)*10);                
            }
            mHeightFieldData[array_index] = static_cast<float>((i+j)*10);
            if(mHeightFieldData[array_index] > mMaxHeight)
                mMaxHeight = mHeightFieldData[array_index];
            if(mHeightFieldData[array_index] < mMinHeight)
                mMinHeight = mHeightFieldData[array_index];                
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
    PerlinNoise perlin_noise(persistence, frequency, amplitude, octaves, randomseed);

    int width = mNumPtsWidth - 1;
    int length = mNumPtsLength - 1;

    // Will be updated below
    this->mMinHeight = static_cast<float>(perlin_noise.GetHeight(-width, -length));
    this->mMaxHeight = static_cast<float>(perlin_noise.GetHeight(-width, -length));

    for(int i = 0; i < mNumPtsWidth; i++) {
        for(int j = 0; j < mNumPtsLength; j++) {
            int array_index = j * mNumPtsWidth + i;
            mHeightFieldData[array_index] = static_cast<float>(perlin_noise.GetHeight(-width*0.5+i, -length*0.5+j));
            if(mHeightFieldData[array_index] > mMaxHeight)
                mMaxHeight = mHeightFieldData[array_index];
            if(mHeightFieldData[array_index] < mMinHeight)
                mMinHeight = mHeightFieldData[array_index];                
        }
    }    
}

}