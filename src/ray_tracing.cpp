
#include "ray_tracing.h"
#include "disable_all_warnings.h"
// Suppress warnings in third-party code.
DISABLE_WARNINGS_PUSH()
#include <glm/geometric.hpp>
#include <glm/gtx/component_wise.hpp>
#include <glm/vector_relational.hpp>
DISABLE_WARNINGS_POP()
#include <cmath>
#include <iostream>
#include <limits>

bool intersectRayWithScene(const Scene& scene, Ray& ray)
{
    bool hit = false;
    for (const auto& mesh : scene.meshes)
        hit |= intersectRayWithShape(mesh, ray);
    for (const auto& sphere : scene.spheres)
        hit |= intersectRayWithShape(sphere, ray);
    for (const auto& box : scene.boxes)
        hit |= intersectRayWithShape(box, ray);
    return hit;
}
bool intersectRayWithShape(const Mesh& mesh, Ray& ray)
{
    bool hit = false;
    for (const auto& tri : mesh.triangles) {
        const auto v0 = mesh.vertices[tri[0]];
        const auto v1 = mesh.vertices[tri[1]];
        const auto v2 = mesh.vertices[tri[2]];
        hit |= intersectRayWithTriangle(v0.p, v1.p, v2.p, ray);
    }
    return hit;
}
Plane trianglePlane(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2)
{
    //We take v2 as starting point for the 2 base vectors of the triangle
    //From v2 we go to vertices v0, v1, so you have 2 vectors, (v0 - v2), (v1 - v2)
    //Taking the cross product with those vectors you get the perpendicular vector to the triangle 
    //(which is also the plane normal, since the plane contains the triangle)
    //If we subtract dot(D, n) from any point P in the plane the resulting vector will be parallel to the plane
    //This is why the length of the orthogonal projection of any point P in the plane will result in D. (in this case we take vertex v0)
    glm::vec3 numerator = glm::cross((v0 - v2), (v1 - v2));
    float denominator = glm::length(glm::cross((v0 - v2), (v1 - v2)));
    glm::vec3 normal = numerator / denominator;
    float distance = glm::dot(normal, v0);
    Plane plane{ distance, normal };

    return plane;
}
bool intersectRayWithPlane(const Plane& plane, Ray& ray)
{
    //plane: pdot(n) - D = 0
    //ray: o + t*d
    //if we substitute we get:
    //(o + t*d)dot(n) - D = 0
    // From this follows: (D - odot(n)) / (ddot(n)
    //If t < 0, the the point is behind the origin, which we don't want, so we return false.
    //If the ray direction and the normal of the plane are parallel, the dot product of the 2 will result in 0, so in that case we return false.
    //If the ray origin lays on the plane, but the direction is parallel to the plane normal, we set ray.t to 0 else we return false.
    //If the new calculated t > ray.t the new t will be a intersection which is farther, so we return false.
    //Else we return true and we update ray.t
    double numerator = plane.D - glm::dot(ray.origin, plane.normal);
    double denominator = glm::dot(ray.direction, plane.normal);
    if (denominator == 0 && denominator <= 0 + pow(10, -6) && denominator >= 0 - pow(10, -6)) {
        float compare = glm::dot(ray.origin, plane.normal) - plane.D;
        if (compare == 0 && compare <= 0 + pow(10, -6) && compare >= 0 - pow(10, -6)) {
            ray.t = 0;
            return true;
        }
        else {
            return false;
        }
    }
    float t = numerator / denominator;
    if (t > ray.t) {
        return false;
    }
    if (t < 0) {
        return false;
    }
    else {
        ray.t = t;
        return true;
    }
}
bool pointInTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& n, const glm::vec3& p)
{
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

    if (alpha >= 0 && beta >= 0 && (alpha + beta) <= 1) {
        return true;
    }
    else {
        return false;
    }
}
/// Input: the three vertices of the triangle
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, Ray& ray)
{
    Plane plane = trianglePlane(v0, v1, v2);
    float t = ray.t;
    if (!intersectRayWithPlane(plane, ray))
        return false;
    if (pointInTriangle(v0, v1, v2, plane.normal, (ray.origin + ray.direction * ray.t)))
        return true;
    ray.t = t;
    return false;
}
/// Input: a sphere with the following attributes: sphere.radius, sphere.center
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const Sphere& sphere, Ray& ray)
{
    //ray: p = o + td 
    //sphere: (p - c)^2 = r^2
    //subtitute:
    //(o + td - c)^2 = r^2
    //v = o - c
    //(v + td)^2 = r^2
    //v^2 + 2vtd + t^2d^2 = r^2
    //v^2 + 2vdt + t^2d^2 - r^2 = 0
    //d^2t^2 + 2vdt + v^2 - r^2 = 0
    //a = d^2 
    //b = 2vd
    //c = v^2 - r^2 
    //Now we can solve this with the ABC formula, (-b +/- sqrt(b^2 - 4ac))/2a
    //When the discriminant < 0, we return false, since there are no solution to this equation
    //If tp or tn < 0, we are inside of the sphere (if they're both < 0, we return false)
    //t < ray.t has to hold, because else it'll be a intersection that is farther away
    //If both tp and tn are positive, we set ray.t to the smallest of the 2, t < ray.t also has to hold here 
    //Else return false
    glm::vec3 o = ray.origin;
    glm::vec3 d = glm::normalize(ray.direction);
    glm::vec3 v = o - sphere.center;
    float r = sphere.radius;
    float a = glm::dot(d, d);
    float b = 2 * glm::dot(v, d);
    float c = glm::dot(v, v) - pow(r, 2);
    if ((pow(b, 2) - 4 * a * c) < 0)
        return false;
    float tp = ((-b + sqrt(pow(b, 2) - 4 * a * c)) / 2 * a);
    float tn = ((-b - sqrt(pow(b, 2) - 4 * a * c)) / 2 * a);

    if (tp < 0 && tn >= 0 && tn < ray.t) {
        ray.t = tn;
        return true;
    }
    if (tn < 0 && tp >= 0 && tp < ray.t) {
        ray.t = tp;
        return true;
    }
    if (tp >= tn && tn < ray.t && tn >= 0) {
        ray.t = tn;
        return true;
    }
    if (tn >= tp && tp < ray.t && tp >= 0) {
        ray.t = tp;
        return true;
    }
    else {
        return false;
    }
}
/// Input: an axis-aligned bounding box with the following parameters: minimum coordinates box.lower and maximum coordinates box.upper
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const AxisAlignedBox& box, Ray& ray)
{
    //plane: x - xmin = 0
    //ray: o + t*d 
    //substitute:
    //o + t*d - xmin = 0;
    //t = (xmin - ox)/d
    //repeat for x, y, z (both min, max)
    //to get to the tin and tout, we can simply do for tin min(tmin, tmax) and for tout max(tmin, tmax) 
    //repeat for x,y,z (both in, out)
    //to determine if we actually intersect:
    //tin: we take the last intersection (so max())
    //tout: we take the first intersection (so min())
    //If tout < tin, we miss the box entirely
    //If tout is negative, the intersection is before the origin of the ray
    //If tin is negative, the ray starts inside of the AABB, so we take tout, since that is the first intersection
    glm::vec3 tmin = (box.lower - ray.origin) / ray.direction;
    glm::vec3 tmax = (box.upper - ray.origin) / ray.direction;

    glm::vec3 tinxyz = glm::min(tmin, tmax);
    glm::vec3 toutxyz = glm::max(tmin, tmax);

    float tin = std::max(std::max(tinxyz.x, tinxyz.y), tinxyz.z);
    float tout = std::min(std::min(toutxyz.x, toutxyz.y), toutxyz.z);

    if (tout < 0)
        return false;
    if (tout < tin)
        return false;
    if (tin > ray.t)
        return false;
    else {
        if (tin < 0)
            ray.t = tout;
        else
            ray.t = tin;
        return true;
    }
}