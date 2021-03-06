#include "Mesh.hpp"

extern json launch_obj;

namespace lazyECS {

// Constructor
Mesh::Mesh(const Shape& mesh_shape) : mColor{openglframework::Color(1,1,1,1)}, mSleepingColor{openglframework::Color(1,0,0,1)}
{
    mShape = mesh_shape;
    if(mesh_shape == Shape::Box) {
        meshPath = "../../meshes/cube.obj";
    } else if(mesh_shape ==  Shape::Sphere) {
        meshPath = "../../meshes/sphere.obj";
    } else if(mesh_shape ==  Shape::Capsule) {
        meshPath = "../../meshes/capsule.obj";
    }
    else if (mesh_shape == Shape::Dumbbell) {
        meshPath = "../../meshes/dumbbell.obj";
    }
    else if(mesh_shape == Shape::ConcaveMesh) {
        meshPath = "../../meshes/walls.obj";
    }
    else if (mesh_shape == Shape::Hfield) {
        json hf_json =  launch_obj.at("floor").at("heightfield");
        mHeightField = std::make_shared<HeightField>(static_cast<int>(hf_json.at("size")) , static_cast<int>(hf_json.at("size")));
    }
    else if(mesh_shape == Shape::Custom) {
        meshPath = launch_obj.at("entities").at("RenderOnlyEntities")[0].at("file");
    }    
    else {
        std::runtime_error("Shape is not assigned to mesh, or no such shape is found!");
    }
}

// Destructor
Mesh::~Mesh() = default;

// Destroy the mesh
void Mesh::destroy() {

    mVertices.clear();
    mNormals.clear();
    mTangents.clear();
    mIndices.clear();
    mColors.clear();
    mUVs.clear();
    mTextures.clear();
}

// Compute the normals of the mesh
void Mesh::calculateNormals() {

    mNormals = std::vector<openglframework::Vector3>(getNbVertices(), openglframework::Vector3(0, 0, 0));

    // For each triangular face
    for (uint i=0; i<getNbFaces(); i++) {

        // Get the three vertices index of the current face
        uint v1 = getVertexIndexInFace(i, 0);
        uint v2 = getVertexIndexInFace(i, 1);
        uint v3 = getVertexIndexInFace(i, 2);

        assert(v1 < getNbVertices());
        assert(v2 < getNbVertices());
        assert(v3 < getNbVertices());

        // Compute the normal of the face
        openglframework::Vector3 p = getVertex(v1);
        openglframework::Vector3 q = getVertex(v2);
        openglframework::Vector3 r = getVertex(v3);
        openglframework::Vector3 normal = (q-p).cross(r-p).normalize();

        // Add the face surface normal to the sum of normals at
        // each vertex of the face
        mNormals[v1] += normal;
        mNormals[v2] += normal;
        mNormals[v3] += normal;
    }

    // Normalize the normal at each vertex
    for (uint i=0; i<getNbVertices(); i++) {
        mNormals[i] = mNormals[i].normalize();
    }
}

// Compute the tangents of the mesh
void Mesh::calculateTangents() {

    mTangents = std::vector<openglframework::Vector3>(getNbVertices(), openglframework::Vector3(0, 0, 0));

    // For each face
    for (uint i=0; i<getNbFaces(); i++) {

        // Get the three vertices index of the face
        uint v1 = getVertexIndexInFace(i, 0);
        uint v2 = getVertexIndexInFace(i, 1);
        uint v3 = getVertexIndexInFace(i, 2);

        assert(v1 < getNbVertices());
        assert(v2 < getNbVertices());
        assert(v3 < getNbVertices());

        // Get the vertices positions
        openglframework::Vector3 p = getVertex(v1);
        openglframework::Vector3 q = getVertex(v2);
        openglframework::Vector3 r = getVertex(v3);

        // Get the texture coordinates of each vertex
        openglframework::Vector2 uvP = getUV(v1);
        openglframework::Vector2 uvQ = getUV(v2);
        openglframework::Vector2 uvR = getUV(v3);

        // Get the three edges
        openglframework::Vector3 edge1 = q - p;
        openglframework::Vector3 edge2 = r - p;
        openglframework::Vector2 edge1UV = uvQ - uvP;
        openglframework::Vector2 edge2UV = uvR - uvP;

        float cp = edge1UV.y * edge2UV.x - edge1UV.x * edge2UV.y;

        // Compute the tangent
        if (cp != 0.0f) {
            float factor = 1.0f / cp;
            openglframework::Vector3 tangent = (edge1 * -edge2UV.y + edge2 * edge1UV.y) * factor;
            tangent.normalize();
            mTangents[v1] = tangent;
            mTangents[v2] = tangent;
            mTangents[v3] = tangent;
        }
    }
}

// Calculate the bounding box of the mesh
void Mesh::calculateBoundingBox(openglframework::Vector3& min, openglframework::Vector3& max) const {

    // If the mesh contains vertices
    if (!mVertices.empty())  {

        min = mVertices[0];
        max = mVertices[0];

        std::vector<openglframework::Vector3>::const_iterator  it(mVertices.begin());

        // For each vertex of the mesh
        for (; it != mVertices.end(); ++it) {

            if( (*it).x < min.x ) min.x = (*it).x;
            else if ( (*it).x > max.x ) max.x = (*it).x;

            if( (*it).y < min.y ) min.y = (*it).y;
            else if ( (*it).y > max.y ) max.y = (*it).y;

            if( (*it).z < min.z ) min.z = (*it).z;
            else if ( (*it).z > max.z ) max.z = (*it).z;
        }
    }
    else {
        std::cerr << "Error : Impossible to calculate the bounding box of the mesh because there" <<
                    "are no vertices !" << std::endl;
        assert(false);
    }
}

// Scale of vertices of the mesh using a given factor
void Mesh::scaleVertices(float factor) {

    // For each vertex
    for (uint i=0; i<getNbVertices(); i++) {
        mVertices.at(i) *= factor;
    }
}

} // namespace LazyECS