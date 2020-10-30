#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include <glm/geometric.hpp>

bool Node::isLeaf() {
    if (children.empty())
        return true;
    else
        return false;
}

std::vector<glm::vec3> getCentroids(std::vector<Triangle>& triangles, Mesh& mesh) {
    std::vector<glm::vec3> centroids;
    for (int i = 0; i < triangles.size(); i++) {
        Triangle tri = triangles[i];
        glm::vec3 P1 = mesh.vertices[tri[0]].p;
        glm::vec3 P2 = mesh.vertices[tri[1]].p;
        glm::vec3 P3 = mesh.vertices[tri[2]].p;
        glm::vec3 centroid = (P1 + P2 + P3) / 3.0f;
        centroids.push_back(centroid);
    }
    return centroids;
}

// made it so that axis can take values from 0 to 2
// change function as you see fit
Split doTheSplit(std::vector<Triangle> triangles, Mesh& mesh, int axis) {
    std::vector<Triangle> subdivision1;
    std::vector<Triangle> subdivision2;
    glm::vec3 mean;
    glm::vec3 sum = glm::vec3{ 0 };
    int middle;

    std::vector<glm::vec3> centroids = getCentroids(triangles, mesh);
    for (auto centroid : centroids) {
        sum += centroid;
    }
    mean = sum / (float)centroids.size();

    // x axis
    if (axis == 0) {
        middle = mean.x;
        for (int i = 0; i < centroids.size(); i++) {
            if (centroids[i].x <= middle) {
                subdivision1.push_back(triangles[i]);
            }
            else {
                subdivision2.push_back(triangles[i]);
            }
        }
    }
    // y axis
    else if (axis == 1) {
        middle = mean.y;
        for (int i = 0; i < centroids.size(); i++) {
            if (centroids[i].y <= middle) {
                subdivision1.push_back(triangles[i]);
            }
            else {
                subdivision2.push_back(triangles[i]);
            }
        }
    }
    // z axis
    else {
        middle = mean.z;
        for (int i = 0; i < centroids.size(); i++) {
            if (centroids[i].z <= middle) {
                subdivision1.push_back(triangles[i]);
            }
            else {
                subdivision2.push_back(triangles[i]);
            }
        }
    }

    return Split{ subdivision1, subdivision2 };
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


void BoundingVolumeHierarchy::fillTree(std::vector<Triangle>& triangles, int index, int axis, Mesh& mesh, int level) {
    AxisAlignedBox aabb = getAABB(triangles, mesh);
    std::vector<int> children;

    if (level == numLevels()) {
        Node node = Node{ aabb, children, triangles, mesh };
        tree[index] = node;
    }
    else
    {
        children.push_back(2 * index + num_meshes - 1);
        children.push_back(2 * index + num_meshes);
        Node node = Node{ aabb, children, triangles, mesh };
        tree[index] = node;

        Split split = doTheSplit(triangles, mesh, axis);
        fillTree(split.subdivision1, 2 * index + num_meshes - 1, axis % 3 + 1, mesh, level + 1);
        fillTree(split.subdivision2, 2 * index + num_meshes, axis % 3 + 1, mesh, level + 1);
    }
}

BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene)
    : m_pScene(pScene)
{
    // i resort to using this garbage code cus vectors are frustrating
    // you might want to increase this number if your custom model has lots of meshes
    for (int i = 0; i < 1000; i++) {
        tree.push_back(Node{});
    }

    num_meshes = pScene->meshes.size();
    glm::vec3 lower = glm::vec3{ INFINITY };
    glm::vec3 upper = glm::vec3{ -INFINITY };

    std::vector<int> children;

    for (int i = 0; i < num_meshes; i++) {
        Mesh mesh = pScene->meshes.at(i);
        std::vector<Triangle> triangles;
        for (const auto& triangle : mesh.triangles) {
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
        children.push_back(i + 1);
    }
    AxisAlignedBox aabb = AxisAlignedBox{ lower, upper };
    Node root = Node{ aabb, children };
    // insert root at index 0
    tree[0] = root;

    // since each node has to store the INDICES of triangles, confusion may arise as to which mesh
    // level 0 of the bvh contains the whole scene
    // level 1 of the bvh contains all individual meshes
    // following levels are split binary
    for (int i = 0; i < num_meshes; i++) {
        Mesh mesh = pScene->meshes.at(i);
        std::vector<Triangle> triangles;
        for (const auto& triangle : mesh.triangles) {
            triangles.push_back(triangle);
        }
        fillTree(triangles, i + 1, 0, mesh, 1);
    }
}

// Use this function to visualize your BVH. This can be useful for debugging. Use the functions in
// draw.h to draw the various shapes. We have extended the AABB draw functions to support wireframe
// mode, arbitrary colors and transparency.
void BoundingVolumeHierarchy::debugDraw(int level)
{
    // Draw the AABB as a transparent green box.
    //AxisAlignedBox aabb{ glm::vec3(-0.05f), glm::vec3(0.05f, 1.05f, 1.05f) };
    //drawShape(aabb, DrawMode::Filled, glm::vec3(0.0f, 1.0f, 0.0f), 0.2f);

    // Draw the AABB as a (white) wireframe box.
    //AxisAlignedBox aabb { glm::vec3(-0.05f), glm::vec3(0.05f, 1.05f, 1.05f) };
    //drawAABB(aabb, DrawMode::Wireframe);
    //drawAABB(aabb, DrawMode::Filled, glm::vec3(0.05f, 1.0f, 0.05f), 0.1);
    NodeLevel zero = NodeLevel{ tree.at(0), 0 };
    std::list<NodeLevel> queue;
    queue.push_back(zero);
    std::list<NodeLevel> atree;
    while (!queue.empty()) {
        NodeLevel n = queue.front();
        if (n.level == level) {
            atree.push_back(n);
        }
        if (n.level > level) {
            break;
        }
        queue.pop_front();
        if (!n.node.isLeaf()) {
            for (int i : n.node.children) {
                queue.push_back({ tree.at(i),n.level + 1 });
            }
        }
    }
    while (!atree.empty()) {
        NodeLevel n = atree.front();
        AxisAlignedBox box = n.node.aabb;
        drawAABB(box, DrawMode::Filled, glm::vec3(0.05f, 1.0f, 0.05f), 0.1);
        atree.pop_front();
    }
}

int BoundingVolumeHierarchy::numLevels() const
{
    return 1;
}

static glm::vec3 interpolateNormal(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& p, const glm::vec3& n0, const glm::vec3& n1, const glm::vec3& n2) {
    //Barycentric coordinates:
     //p = v0 + alpha(v1 - v0) + beta(v2 - v0)
     //p - v0 = alpha(v1 - v0) + beta(v2 - v0)
     //A = v1 - v0, B = v2 - v0, C = p - v0
     //C = alpha*A + beta*B
     //
     //We can dot both sides with A and B
     //Cdot(A) = alpha*(Adot(A)) + beta*(Adot(B))
     //Cdot(B) = alpha*(Adot(B)) + beta*(Bdot(B))
     //
     //dot0 Adot(A), dot1 = Adot(B), dot2 = Adot(C), dot3 = Bdot(B), dot4 = Bdot(C)
     //
     //substitute:
     //dot2 = alpha(dot0) + beta(dot1)
     //dot4 = alpha(dot1) + beta(dot3)
     //
     //dot2 - beta(dot1) = alpha(dot0)
     //alpha = (dot2 - beta(dot1))/dot0
     // 
     //dot4 - alpha(dot1) = beta(dot3)
     //beta = (dot4 - alpha(dot1)) / dot3
     //
     //alpha = (dot2 - dot1((dot4 - alpha(dot1)) / dot3)/dot0
     //alpha*dot0 = dot2 - dot1((dot4 - alpha(dot1)) / dot3
     //alpha*dot0*dot3 = dot2*dot3 - dot1((dot4 - alpha(dot1)
     //alpha*dot0*dot3 = dot2*dot3 - dot1*dot4 - alpha(dot1*dot1)
     //alpha*dot0*dot3 + alpha(dot1*dot1) = dot2*dot3 - dot1*dot4
     //alpha(dot0*dot3 + dot1*dot1) = dot2*dot3 - dot1*dot4
     //alpha = (dot2*dot3 - dot1*dot4) / (dot0*dot3 + dot1*dot1)
     // 
     //do the same thing for beta and you get:
     //beta = (dot0*dot4 - dot1*dot2) / (dot0*dot3 + dot1*dot1)    
     //to get gamma, we can simply do 1 - alpha - beta
     //
     //to interpolate the normal, we can multiply the normals with either alpha, beta or gamma.
     //v0 + alpha(v1 - v0) + beta(v2 - v0)
     //v0 - alpha*v0 - beta* v0 + alpha*v1 + beta*v2
     //v0(1-alpha-beta) + alpha*v1 + beta*v2
     //so: n0*gamma + alpha*n1 + beta*n2
    glm::vec3 A = v1 - v0;
    glm::vec3 B = v2 - v0;
    glm::vec3 C = p - v0;

    double dot0 = glm::dot(A, A);
    double dot1 = glm::dot(A, B);
    double dot2 = glm::dot(A, C);
    double dot3 = glm::dot(B, B);
    double dot4 = glm::dot(B, C);

    double alpha = (dot2 * dot3 - dot1 * dot4) / (dot0 * dot3 - dot1 * dot1);
    double beta = (dot0 * dot4 - dot1 * dot2) / (dot0 * dot3 - dot1 * dot1);
    double gamma = 1 - alpha - beta;
    glm::vec3 avgNormal = n0 * glm::vec3(gamma) + n1 * glm::vec3(alpha) + n2 * glm::vec3(beta);
    return avgNormal;
}

// Return true if something is hit, returns false otherwise. Only find hits if they are closer than t stored
// in the ray and if the intersection is on the correct side of the origin (the new t >= 0). Replace the code
// by a bounding volume hierarchy acceleration structure as described in the assignment. You can change any
// file you like, including bounding_volume_hierarchy.h .
bool BoundingVolumeHierarchy::intersect(Ray& ray, HitInfo& hitInfo) const
{
    bool hit = false;
    
    std::vector<Node> stack;
    stack.push_back(tree.at(0));
    while (!stack.empty()) {        
        Node node = stack.front();
        stack.erase(stack.begin());
        float originT = ray.t;
        if (!intersectRayWithShape(node.aabb, ray)) {
            continue;
        }
        ray.t = originT;
        if (node.isLeaf()) {
            for (const auto& tri : node.triangles) {
                const auto v0 = node.mesh.vertices[tri[0]];
                const auto v1 = node.mesh.vertices[tri[1]];
                const auto v2 = node.mesh.vertices[tri[2]];
                bool currentHit = intersectRayWithTriangle(v0.p, v1.p, v2.p, ray, hitInfo);
                if (interpolated && currentHit) {
                    glm::vec3 normal = interpolateNormal(v0.p, v1.p, v2.p, (ray.origin + ray.direction * ray.t), v0.n, v1.n, v2.n);
                    hitInfo.normal = normal;
                    hitInfo.material = node.mesh.material;
                }
                else if (currentHit) {                    
                    hitInfo.material = node.mesh.material;
                    hitInfo.normal = glm::normalize(glm::cross((v1.p - v0.p), (v2.p - v0.p)));
                }
                hit |= currentHit;
            }
        }
        else {
            for (int index : node.children) {
                stack.push_back(tree.at(index));
            }
        }
    }
    return hit;
}
