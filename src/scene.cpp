#include "scene.h"
#include <iostream>
DISABLE_WARNINGS_PUSH()
#include <glm/gtc/type_ptr.hpp>

//TODO: update to a larger number once acceleration is done
const int number_of_lights = 6;

// reference: https://scholar.rose-hulman.edu/cgi/viewcontent.cgi?article=1387&context=rhumj
// how to place N (almost) equally spaced points on a sphere
static glm::vec3 sphericalCoordinate(float x, float y, SphericalLight& spherical_light) {
    return glm::vec3(glm::cos(x) * glm::cos(y),
        glm::sin(x) * glm::cos(y),
        glm::sin(y)) 
        * spherical_light.radius;
}

// replace a spherical light with N (=number_of_lights) point lights
static void placeSphericalLight(Scene& scene, SphericalLight& spherical_light) {
    glm::vec3 color = spherical_light.color / (float)number_of_lights;
    float x = 0.1 + 1.2 * number_of_lights;
    float start = (-1.0f + 1.0f / (float)(number_of_lights - 1));
    float increment = (2.0f - 2.0f / (float)(number_of_lights - 1)) / (float)(number_of_lights - 1);
    for (int j = 0; j < number_of_lights; j++) {
        float s = start + j * increment;
        glm::vec3 point_position = sphericalCoordinate(s * x,
            glm::pi<float>() / 2.0f * std::copysign(1.0f, s) * (1.0f - glm::sqrt(1.0f - glm::abs(s))),
            spherical_light) + spherical_light.position;
        scene.pointLights.push_back(PointLight { point_position, color});
    }
}

Scene loadScene(SceneType type, const std::filesystem::path& dataDir)
{
    Scene scene;
    switch (type) {
    case SingleTriangle: {
        // Load a 3D model with a single triangle
        auto subMeshes = loadMesh(dataDir / "triangle.obj");
        subMeshes[0].material.kd = glm::vec3(1.0f);
        std::move(std::begin(subMeshes), std::end(subMeshes), std::back_inserter(scene.meshes));
        scene.pointLights.push_back(PointLight { glm::vec3(-1, 1, -1), glm::vec3(1) });
    } break;
    case Cube: {
        // Load a 3D model of a cube with 12 triangles
        auto subMeshes = loadMesh(dataDir / "cube.obj");
        std::move(std::begin(subMeshes), std::end(subMeshes), std::back_inserter(scene.meshes));
        scene.pointLights.push_back(PointLight { glm::vec3(-1, 1, -1), glm::vec3(1) });
    } break;
    case CornellBox: {
        // Load a 3D model of a Dragon
        auto subMeshes = loadMesh(dataDir / "CornellBox-Mirror-Rotated.obj", true);
        std::move(std::begin(subMeshes), std::end(subMeshes), std::back_inserter(scene.meshes));
        scene.pointLights.push_back(PointLight { glm::vec3(0, 0.58f, 0), glm::vec3(1) }); // Light at the top of the box
    } break;
    case CornellBoxSphericalLight: {
        // Load a 3D model of a Dragon
        auto subMeshes = loadMesh(dataDir / "CornellBox-Mirror-Rotated.obj", true);
        std::move(std::begin(subMeshes), std::end(subMeshes), std::back_inserter(scene.meshes));
        scene.sphericalLight.push_back(SphericalLight { glm::vec3(0, 0.45f, 0), 0.1f, glm::vec3(1) }); // Light at the top of the box
    } break;
    case Monkey: {
        // Load a 3D model of a Dragon
        auto subMeshes = loadMesh(dataDir / "monkey-rotated.obj", true);
        std::move(std::begin(subMeshes), std::end(subMeshes), std::back_inserter(scene.meshes));
        scene.pointLights.push_back(PointLight { glm::vec3(-1, 1, -1), glm::vec3(1) });
        scene.pointLights.push_back(PointLight { glm::vec3(1, -1, -1), glm::vec3(1) });
    } break;
    case Dragon: {
        // Load a 3D model of a Dragon
        auto subMeshes = loadMesh(dataDir / "dragon.obj", true);
        std::move(std::begin(subMeshes), std::end(subMeshes), std::back_inserter(scene.meshes));
        scene.pointLights.push_back(PointLight { glm::vec3(-1, 1, -1), glm::vec3(1) });
    } break;
    /*case AABBs: {
        //scene.boxes.push_back(AxisAlignedBox { glm::vec3(-2.0f, -2.0f, 5.0f), glm::vec3(-1.0f, -1.0f, 6.0f) });
        //scene.boxes.push_back(AxisAlignedBox { glm::vec3(0.0f, 0.0f, 5.0f), glm::vec3(1.5f, 1.5f, 7.0f) });
        //scene.boxes.push_back(AxisAlignedBox { glm::vec3(0.5f, 0.5f, 2.0f), glm::vec3(0.9f, 0.9f, 2.5f) });
    } break;*/
    case Spheres: {
        scene.spheres.push_back(Sphere { glm::vec3(3.0f, -2.0f, 10.2f), 1.0f, Material { glm::vec3(0.8f, 0.2f, 0.2f) } });
        scene.spheres.push_back(Sphere { glm::vec3(-2.0f, 2.0f, 4.0f), 2.0f, Material { glm::vec3(0.6f, 0.8f, 0.2f) } });
        scene.spheres.push_back(Sphere { glm::vec3(0.0f, 0.0f, 6.0f), 0.75f, Material { glm::vec3(0.2f, 0.2f, 0.8f) } });
        scene.pointLights.push_back(PointLight { glm::vec3(3, 0, 3), glm::vec3(15) });
    } break;
    case Custom: {
        // === Replace custom.obj by your own 3D model (or call your 3D model custom.obj) ===
        auto subMeshes = loadMesh(dataDir / "custom.obj");
        std::move(std::begin(subMeshes), std::end(subMeshes), std::back_inserter(scene.meshes));
        // === CHANGE THE LIGHTING IF DESIRED ===
        scene.pointLights.push_back(PointLight { glm::vec3(-1, 1, -1), glm::vec3(1) });
        // Spherical light: position, radius, color
        //scene.sphericalLight.push_back(SphericalLight{ glm::vec3(0, 1.5f, 0), 0.2f, glm::vec3(1) });
    } break;
    };

    for (SphericalLight spherical_light : scene.sphericalLight) {
        placeSphericalLight(scene, spherical_light);
    }

    return scene;
}

/*
*/
