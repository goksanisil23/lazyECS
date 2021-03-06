#pragma once

#include <vector>
#include <map>

#include <reactphysics3d/reactphysics3d.h>
// #include "openglframework.h"
#include "definitions.h"
#include "maths/Vector2.h"
#include "maths/Vector3.h"
#include "maths/Color.h"
#include "Texture2D.h"
#include "HeightField.h"
#include "Types.hpp"

namespace lazyECS {

enum Shape{Box, Sphere, ConcaveMesh, Capsule, Dumbbell, Hfield, Custom, Last};

class Mesh {

protected:
    // ----------------- Member variables ----------------- //

    std::vector<std::vector<unsigned int>> mIndices; // trio of vertex indices for each triangle
    std::vector<openglframework::Vector3> mVertices; // vertex coordinates (model space)
    std::vector<openglframework::Vector3> mNormals; // Normal coordinates
    std::vector<openglframework::Vector3> mTangents; // Tangent coordinates
    std::vector<openglframework::Color> mColors; // Color per vertex
    std::vector<openglframework::Vector2> mUVs; // UV texture coordinates
    std::map<uint, openglframework::Texture2D> mTextures; // Textures of the mesh per each part of the mesh

public:

    openglframework::Color mColor;
    openglframework::Color mSleepingColor;
    std::string meshPath; // path of the obj file where the mesh description is loaded
    Shape mShape;
    std::shared_ptr<HeightField> mHeightField; // only populated if the Shape is Heightfield

    // ----------------- Member functions ----------------- //

        // Constructor
        explicit Mesh(const Shape& mesh_shape);

        // Construct mesh from model file
        // Mesh(const std::string& meshPath);

        // Destructor
        virtual ~Mesh();

        // Destroy the mesh
        void destroy();

        // Compute the normals of the mesh
        void calculateNormals();

        // Compute the tangents of the mesh
        void calculateTangents();

        // Calculate the bounding box of the mesh
        void calculateBoundingBox(openglframework::Vector3& min, openglframework::Vector3& max) const;

        // Scale of vertices of the mesh using a given factor
        void scaleVertices(float factor);

        // Return the number of triangles
        uint getNbFaces(uint part = 0) const;

        // Return the number of vertices
        uint getNbVertices() const;

        // Return the number of parts in the mesh
        uint getNbParts() const;

        // Return a reference to the vertices
        const std::vector<openglframework::Vector3>& getVertices() const;

        // Set the vertices of the mesh
        void setVertices(std::vector<openglframework::Vector3>& vertices);

        // Return a reference to the normals
        const std::vector<openglframework::Vector3>& getNormals() const;

        // set the normals of the mesh
        void setNormals(std::vector<openglframework::Vector3>& normals);

        // Return a reference to the UVs
        const std::vector<openglframework::Vector2>& getUVs() const;

        // Set the UV texture coordinates of the mesh
        void setUVs(std::vector<openglframework::Vector2>& uvs);

        // Return a reference to the vertex indices
        const std::vector<uint>& getIndices(uint part = 0) const;

        // Set the vertices indices of the mesh
        void setIndices(std::vector<std::vector<uint> >& indices);

        // Return the coordinates of a given vertex
        const openglframework::Vector3& getVertex(uint i) const;

        // Set the coordinates of a given vertex
        void setVertex(uint i, const openglframework::Vector3& vertex);

        // Return the coordinates of a given normal
        const openglframework::Vector3& getNormal(uint i) const;

        // Set the coordinates of a given normal
        void setNormal(uint i, const openglframework::Vector3& normal);

        // Return the color of a given vertex
        const openglframework::Color& getVertexColor(uint i) const;

        // Set the color of a given vertex
        void setVertexColor(uint i, const openglframework::Color& color);

        // Set a color to all the vertices
        void setColorToAllVertices(const openglframework::Color& color);

        // Return the UV of a given vertex
        const openglframework::Vector2& getUV(uint i) const;

        // Set the UV of a given vertex
        void setUV(uint i, const openglframework::Vector2& uv);

        // Return the vertex index of the ith (i=0,1,2) vertex of a given face
        uint getVertexIndexInFace(uint faceIndex, uint i, uint part = 0) const;

        // Return true if the mesh has normals
        bool hasNormals() const;

        // Return true if the mesh has tangents
        bool hasTangents() const;

        // Return true if the mesh has vertex colors
        bool hasColors() const;

        // Return true if the mesh has UV texture coordinates
        bool hasUVTextureCoordinates() const;

        // Return true if the mesh has a texture for a given part of the mesh and if it
        // also have texture coordinates
        bool hasTextureForPart(uint part = 0) const;

        // Return true if the mesh has a texture (and texture coordinates) for at least one
        // part of the mesh
        bool hasTexture() const;

        // Return a pointer to the vertices data
        void* getVerticesPointer();

        // Return a pointer to the normals data
        void* getNormalsPointer();

        // Return a pointer to the colors data
        void* getColorsPointer();

        // Return a pointer to the tangents data
        void* getTangentsPointer();

        // Return a pointer to the UV texture coordinates data
        void* getUVTextureCoordinatesPointer();

        // Return a pointer to the vertex indicies data
        void* getIndicesPointer(uint part = 0);

        // Return a reference to a texture of the mesh
        openglframework::Texture2D &getTexture(uint part = 0);

        // Set a texture to a part of the mesh
        void setTexture(openglframework::Texture2D &texture, uint part = 0);

        // ---------------------- MESH READING/WRITING FUNCTIONS --------------------- //

        // // Load an OBJ file with a triangular or quad mesh
        // static void loadOBJFile(const std::string& filename);

        // // Store a mesh into a OBJ file
        // static void writeOBJFile(const std::string& filename);

        // // Read a mesh from a file
        // static void loadMeshFromFile(const std::string& filename);

        // // Write a mesh to a file
        // static void writeMeshToFile(const std::string& filename);        
};

// Return the number of triangles
inline uint Mesh::getNbFaces(uint part) const {
    return mIndices[part].size() / 3;
}

// Return the number of vertices
inline uint Mesh::getNbVertices() const {
    return mVertices.size();
}

// Return the number of parts in the mesh
inline uint Mesh::getNbParts() const {
    return mIndices.size();
}

// Return a reference to the vertices
inline const std::vector<openglframework::Vector3>& Mesh::getVertices() const {
    return mVertices;
}

// Set the vertices of the mesh
inline void Mesh::setVertices(std::vector<openglframework::Vector3>& vertices) {
    mVertices = vertices;
}

// Return a reference to the normals
inline const std::vector<openglframework::Vector3>& Mesh::getNormals() const {
    return mNormals;
}

// set the normals of the mesh
inline void Mesh::setNormals(std::vector<openglframework::Vector3>& normals) {
    mNormals = normals;
}

// Return a reference to the UVs
inline const std::vector<openglframework::Vector2>& Mesh::getUVs() const {
    return mUVs;
}

// Set the UV texture coordinates of the mesh
inline void Mesh::setUVs(std::vector<openglframework::Vector2>& uvs) {
    mUVs = uvs;
}

// Return a reference to the vertex indices
inline const std::vector<uint>& Mesh::getIndices(uint part) const {
    return mIndices[part];
}

// Set the vertices indices of the mesh
inline void Mesh::setIndices(std::vector<std::vector<uint> >& indices) {
    mIndices = indices;
}

// Return the coordinates of a given vertex
inline const openglframework::Vector3& Mesh::getVertex(uint i) const {
    assert(i < getNbVertices());
    return mVertices[i];
}

// Set the coordinates of a given vertex
inline void Mesh::setVertex(uint i, const openglframework::Vector3& vertex) {
    assert(i < getNbVertices());
    mVertices[i] = vertex;
}

// Return the coordinates of a given normal
inline const openglframework::Vector3& Mesh::getNormal(uint i) const {
    assert(i < getNbVertices());
    return mNormals[i];
}

// Set the coordinates of a given normal
inline void Mesh::setNormal(uint i, const openglframework::Vector3& normal) {
    assert(i < getNbVertices());
    mNormals[i] = normal;
}

// Return the color of a given vertex
inline const openglframework::Color& Mesh::getVertexColor(uint i) const {
    assert(i < getNbVertices());
    return mColors[i];
}

// Set the color of a given vertex
inline void Mesh::setVertexColor(uint i, const openglframework::Color& color) {

    // If the color array does not have the same size as
    // the vertices array
    if (mColors.size() != mVertices.size()) {

        // Create the color array with the same size
        mColors = std::vector<openglframework::Color>(mVertices.size());
    }

    mColors[i] = color;
}

// Set a color to all the vertices
inline void Mesh::setColorToAllVertices(const openglframework::Color& color) {

    // If the color array does not have the same size as
    // the vertices array
    if (mColors.size() != mVertices.size()) {

        // Create the color array with the same size
        mColors = std::vector<openglframework::Color>(mVertices.size());
    }

    for (size_t v=0; v<mVertices.size(); v++) {
        mColors[v] = color;
    }
}

// Return the UV of a given vertex
inline const openglframework::Vector2& Mesh::getUV(uint i) const {
    assert(i < getNbVertices());
    return mUVs[i];
}

// Set the UV of a given vertex
inline void Mesh::setUV(uint i, const openglframework::Vector2& uv) {
    assert(i < getNbVertices());
    mUVs[i] = uv;
}

// Return the vertex index of the ith (i=0,1,2) vertex of a given face
inline uint Mesh::getVertexIndexInFace(uint faceIndex, uint i, uint part) const {
    return (mIndices[part])[faceIndex*3 + i];
}

// Return true if the mesh has normals
inline bool Mesh::hasNormals() const {
    return mNormals.size() == mVertices.size();
}

// Return true if the mesh has tangents
inline bool Mesh::hasTangents() const {
    return mTangents.size() == mVertices.size();
}

// Return true if the mesh has vertex colors
inline bool Mesh::hasColors() const {
    return mColors.size() == mVertices.size();
}

// Return true if the mesh has UV texture coordinates
inline bool Mesh::hasUVTextureCoordinates() const {
    return mUVs.size() == mVertices.size();
}

// Return true if the mesh has a texture for a given part of the mesh and if it
// also have texture coordinates
inline bool Mesh::hasTextureForPart(uint part) const {
    return hasUVTextureCoordinates() && mTextures.count(part);
}

// Return true if the mesh has a texture (and texture coordinates) for at least one
// part of the mesh
inline bool Mesh::hasTexture() const {
    return hasUVTextureCoordinates() && (mTextures.size() > 0);
}

// Return a pointer to the vertices data
inline void* Mesh::getVerticesPointer() {
    return &(mVertices[0]);
}

// Return a pointer to the normals data
inline void* Mesh::getNormalsPointer() {
    return &(mNormals[0]);
}

// Return a pointer to the colors data
inline void* Mesh::getColorsPointer() {
    return &(mColors[0]);
}

// Return a pointer to the tangents data
inline void* Mesh::getTangentsPointer() {
    return &(mTangents[0]);
}

// Return a pointer to the UV texture coordinates data
inline void* Mesh::getUVTextureCoordinatesPointer() {
    return &(mUVs[0]);
}

// Return a pointer to the vertex indicies data
inline void* Mesh::getIndicesPointer(uint part) {
    return &(mIndices[part])[0];
}

// Return a reference to a texture of the mesh
inline openglframework::Texture2D& Mesh::getTexture(uint part) {
    return mTextures[part];
}

// Set a texture to a part of the mesh
inline void Mesh::setTexture(openglframework::Texture2D& texture, uint part) {
    mTextures[part] = texture;
}


} // namespace lazyECS