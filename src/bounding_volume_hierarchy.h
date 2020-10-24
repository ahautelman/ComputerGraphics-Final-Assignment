#pragma once
#include "ray_tracing.h"
#include "scene.h"

// if split contains less than min_triangles, stop splitting
// this number was chosen arbitrarily
const int min_triangles = 100;

struct Split
{
    std::vector<Triangle> subdivision1;
    std::vector<Triangle> subdivision2;
};

struct Node
{
    AxisAlignedBox aabb;

    // array used to store the indeces of children in the tree
    std::vector<int> children;

    bool isLeaf();
};

struct LeafNode : public Node 
{
    // if the node is leaf, array is used to store the triangles contained
    // TODO change from vector<Triangle> to vector<int> (containing the INDICES of triangles ????)
    std::vector<Triangle> triangles;

    // only leaf nodes have the mesh attribute
    Mesh mesh;
};

class BoundingVolumeHierarchy {
public:
    BoundingVolumeHierarchy(Scene* scene);

    // Use this function to visualize your BVH. This can be useful for debugging.
    void debugDraw(int level);
    int numLevels() const;

    void fillTree(std::vector<Triangle>& triangles, int index, int axis, Mesh& mesh);

    // Return true if something is hit, returns false otherwise.
    // Only find hits if they are closer than t stored in the ray and the intersection
    // is on the correct side of the origin (the new t >= 0).
    bool intersect(Ray& ray, HitInfo& hitInfo) const;

    AxisAlignedBox getAABB(std::vector<Triangle> triangles, Mesh& mesh);

private:
    Scene* m_pScene;

    // num_meshes is equal to the number of individual meshes on the first level of the bvh
    int num_meshes;

    // remark: i have no idea how referencing works. Should use * or & for tree??
    // since each node has to store the INDICES of triangles, confusion may arise as to which mesh it refers to
    // level 0 of the bvh contains the whole scene
    // level 1 of the bvh contains all individual meshes
    // following levels are split binary
    // eg: level 0 ---------------- whole scene -------------------
    //                         /          |          \
    //     level 1 -------- mesh1 ----- mesh2 ----- mesh3 ------
    //                    /   |         /   \          |    \
    //     level 2 - Node -- Node --- Node -- Node --- Node -- Node----
    //
    // how to calculate index: if parent has index i => children have indexes 
    //                          2 * index + num_meshes - 1 and
    //                          2 * index + num_meshes
    std::vector<Node> tree;
};
