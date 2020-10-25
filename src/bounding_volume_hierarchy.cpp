#include "bounding_volume_hierarchy.h"
#include "draw.h"

bool Node::isLeaf() {
    if (children.empty())
        return true;
    else
        return false;
}

// made it so that axis can take values from 0 to 2
// change function as you see fit
Split doTheSplit(std::vector<Triangle> triangles, std::vector<Vertex> vertices, int axis) {
    std::vector<Triangle> subdivision1;
    std::vector<Triangle> subdivision2;
    float mean = 0;
    float sum = 0;

    for (Triangle triangle : triangles) {
        if (axis == 0) 
            sum += triangle.x;        
        if (axis == 1)          
            sum += triangle.y;        
        if (axis == 2) 
            sum += triangle.z;      
    }
    if (sum == 0) 
        mean = 0;
    else
        mean = sum / triangles.size();

    for (Triangle triangle : triangles) {
        if (axis == 0) {
            if (triangle.x >= mean)
                subdivision1.push_back(triangle);
            else
                subdivision2.push_back(triangle);
        }
        if (axis == 1) {
            if (triangle.y >= mean)
                subdivision1.push_back(triangle);
            else
                subdivision2.push_back(triangle);
        }
        if (axis == 2) {
            if (triangle.z >= mean)
                subdivision1.push_back(triangle);
            else
                subdivision2.push_back(triangle);
        }
    }

    return Split{subdivision1, subdivision2};
}

AxisAlignedBox BoundingVolumeHierarchy::getAABB(std::vector<Triangle> triangles, Mesh& mesh) {
    glm::vec3 lower = glm::vec3{ INFINITY };
    glm::vec3 upper = glm::vec3{ -INFINITY };

    // is there better way to do this?
    for (const auto triangle : triangles) {
        float min_x = std::min(mesh.vertices[triangle[0]].p.x, std::min(
            mesh.vertices[triangle[1]].p.x, mesh.vertices[triangle[2]].p.x));
        lower.x = std::min(lower.x, min_x);
        float min_y = std::min(mesh.vertices[triangle[0]].p.y, std::min(
            mesh.vertices[triangle[1]].p.y, mesh.vertices[triangle[2]].p.y));
        lower.y = std::min(lower.y, min_y);
        float min_z = std::min(mesh.vertices[triangle[0]].p.z, std::min(
            mesh.vertices[triangle[1]].p.z, mesh.vertices[triangle[2]].p.z));
        lower.z = std::min(lower.z, min_z);

        float max_x = std::max(mesh.vertices[triangle[0]].p.x, std::max(
            mesh.vertices[triangle[1]].p.x, mesh.vertices[triangle[2]].p.x));
        upper.x = std::max(upper.x, max_x);
        float max_y = std::max(mesh.vertices[triangle[0]].p.y, std::max(
            mesh.vertices[triangle[1]].p.y, mesh.vertices[triangle[2]].p.y));
        upper.y = std::max(upper.y, max_y);
        float max_z = std::max(mesh.vertices[triangle[0]].p.z, std::max(
            mesh.vertices[triangle[1]].p.z, mesh.vertices[triangle[2]].p.z));
        upper.z = std::max(upper.z, max_z);
    }
    return AxisAlignedBox{ lower, upper }; 
}

void BoundingVolumeHierarchy::fillTree(std::vector<Triangle>& triangles, int index, int axis, Mesh& mesh) {
    AxisAlignedBox aabb = getAABB(triangles, mesh);
    std::vector<int> children;
    if (triangles.size() < min_triangles) {
        LeafNode node = LeafNode{aabb, children, triangles, mesh};
        tree.insert(tree.begin() + index, node);
    }
    else
    {
        children.push_back(2 * index + num_meshes - 1);
        children.push_back(2 * index + num_meshes);
        Node node = Node{ aabb, children};
        tree.insert(tree.begin() + index, node);

        Split split = doTheSplit(triangles, mesh.vertices, axis);
        fillTree(split.subdivision1, 2 * index + num_meshes - 1, axis % 3 + 1, mesh);
        fillTree(split.subdivision2, 2 * index + num_meshes, axis % 3 + 1, mesh);
    }
}

BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene)
    : m_pScene(pScene)
{
    num_meshes = pScene->meshes.size();
    glm::vec3 lower = glm::vec3{ INFINITY };
    glm::vec3 upper = glm::vec3{ -INFINITY };

    std::vector<int> children;

    // since each node has to store the INDICES of triangles, confusion may arise as to which mesh
    // level 0 of the bvh contains the whole scene
    // level 1 of the bvh contains all individual meshes
    // following levels are split binary
    for (int i = 0; i < num_meshes; i++) {
        Mesh mesh = pScene->meshes.at(i);
        std::vector<Triangle> triangles;
        for (const auto& triangle : mesh.triangles) {
            triangles.push_back(triangle);

            // building aabb for root
            float min_x = std::min(mesh.vertices[triangle[0]].p.x, std::min(
                mesh.vertices[triangle[1]].p.x, mesh.vertices[triangle[2]].p.x));
            lower.x = std::min(lower.x, min_x);
            float min_y = std::min(mesh.vertices[triangle[0]].p.y, std::min(
                mesh.vertices[triangle[1]].p.y, mesh.vertices[triangle[2]].p.y));
            lower.y = std::min(lower.y, min_y);
            float min_z = std::min(mesh.vertices[triangle[0]].p.z, std::min(
                mesh.vertices[triangle[1]].p.z, mesh.vertices[triangle[2]].p.z));
            lower.z = std::min(lower.z, min_z);

            float max_x = std::max(mesh.vertices[triangle[0]].p.x, std::max(
                mesh.vertices[triangle[1]].p.x, mesh.vertices[triangle[2]].p.x));
            upper.x = std::max(upper.x, max_x);
            float max_y = std::max(mesh.vertices[triangle[0]].p.y, std::max(
                mesh.vertices[triangle[1]].p.y, mesh.vertices[triangle[2]].p.y));
            upper.y = std::max(upper.y, max_y);
            float max_z = std::max(mesh.vertices[triangle[0]].p.z, std::max(
                mesh.vertices[triangle[1]].p.z, mesh.vertices[triangle[2]].p.z));
            upper.z = std::max(upper.z, max_z);
        }
        fillTree(triangles, i + 1, 0, mesh);
        children.push_back(i);
    }

    AxisAlignedBox aabb = AxisAlignedBox{ lower, upper };
    Node root = Node{ aabb, children };
    // insert root at index 0
    tree.insert(tree.begin(), root);
}

// Use this function to visualize your BVH. This can be useful for debugging. Use the functions in
// draw.h to draw the various shapes. We have extended the AABB draw functions to support wireframe
// mode, arbitrary colors and transparency.
void BoundingVolumeHierarchy::debugDraw(int level)
{   
    std::vector<Node> tree = this->tree;
    if (level == 0) 
        drawAABB(this->tree[0].aabb, DrawMode::Filled, glm::vec3(0.05f, 1.0f, 0.05f), 0.1);    
    if (level == 1) {
        for (int i = 0; i < this->num_meshes; i++) {
            drawAABB(this->tree[i + 1].aabb, DrawMode::Filled, glm::vec3(0.05f, 1.0f, 0.05f), 0.1);
        }
    } else {   
        int indexLast = this->num_meshes;   
        for (int i = 2; i <= level; i++) {
            indexLast += this->num_meshes * pow(2, i - 1);                     
        }    
        int indexFirst = indexLast - this->num_meshes * pow(2, level - 1) + 1;
        for (int j = 0; j < this->num_meshes * pow(2, level - 1); j++) {
            AxisAlignedBox aabb = tree[indexFirst].aabb;
            indexFirst++;
            drawAABB(aabb, DrawMode::Filled, glm::vec3(0.05f, 1.0f, 0.05f), 0.1);
        }     
        // Draw the AABB as a transparent green box.
        //AxisAlignedBox aabb{ glm::vec3(-0.05f), glm::vec3(0.05f, 1.05f, 1.05f) };
        //drawShape(aabb, DrawMode::Filled, glm::vec3(0.0f, 1.0f, 0.0f), 0.2f);
    }  
}

int BoundingVolumeHierarchy::numLevels() const
{
    return 5;
}

// Return true if something is hit, returns false otherwise. Only find hits if they are closer than t stored
// in the ray and if the intersection is on the correct side of the origin (the new t >= 0). Replace the code
// by a bounding volume hierarchy acceleration structure as described in the assignment. You can change any
// file you like, including bounding_volume_hierarchy.h .
bool BoundingVolumeHierarchy::intersect(Ray& ray, HitInfo& hitInfo) const
{
    bool hit = false;
    // Intersect with all triangles of all meshes.
    for (const auto& mesh : m_pScene->meshes) {
        for (const auto& tri : mesh.triangles) {
            const auto v0 = mesh.vertices[tri[0]];
            const auto v1 = mesh.vertices[tri[1]];
            const auto v2 = mesh.vertices[tri[2]];
            if (intersectRayWithTriangle(v0.p, v1.p, v2.p, ray, hitInfo)) {
                hitInfo.material = mesh.material;
                hit = true;
            }
        }
    }
    // Intersect with spheres.
    for (const auto& sphere : m_pScene->spheres)
        hit |= intersectRayWithShape(sphere, ray, hitInfo);
    return hit;
}
