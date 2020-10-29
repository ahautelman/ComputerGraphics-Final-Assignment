#include "bounding_volume_hierarchy.h"
#include "disable_all_warnings.h"
#include "draw.h"
#include "image.h"
#include "ray_tracing.h"
#include "screen.h"
#include "trackball.h"
#include "window.h"
// Disable compiler warnings in third-party code (which we cannot change).
DISABLE_WARNINGS_PUSH()
#include <glm/gtc/type_ptr.hpp>
#include <glm/vec2.hpp>
#include <glm/vec4.hpp>
#include <imgui.h>
DISABLE_WARNINGS_POP()
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <optional>
#include <random>
#include <string>
#include <type_traits>
#ifdef USE_OPENMP
#include <omp.h>
#endif

// This is the main application. The code in here does not need to be modified.
constexpr glm::ivec2 windowResolution{ 800, 800 };
const std::filesystem::path dataPath{ DATA_DIR };
const std::filesystem::path outputPath{ OUTPUT_DIR };
static glm::vec3 Shade(Scene scene, int level, Ray ray, glm::vec3 color, BoundingVolumeHierarchy bvh, HitInfo hitInfo);
static glm::vec3 Trace(Scene scene, int level, Ray ray, glm::vec3 color, BoundingVolumeHierarchy bvh);

enum class ViewMode {
    Rasterization = 0,
    RayTracing = 1
};

glm::vec3 diffuseOnly(const glm::vec3 Kd, const glm::vec3& vertexPos, const glm::vec3& normal, const glm::vec3& lightPos)
{
    glm::vec3 normaln = glm::normalize(normal);
    glm::vec3 lightPosn = glm::normalize(lightPos - vertexPos);
    if (glm::dot(normaln, lightPosn) <= 0)
        return glm::vec3(0);
    return Kd * glm::dot(normaln, lightPosn);
}
glm::vec3 phongSpecularOnly(const glm::vec3 Ks, const float shininess, const glm::vec3& vertexPos, const glm::vec3& normal, const glm::vec3& lightPos, const glm::vec3& cameraPos)
{
    glm::vec3 normaln = glm::normalize(normal);
    glm::vec3 lightn = glm::normalize(lightPos - vertexPos);
    glm::vec3 reflection = glm::vec3(2) * (glm::dot(lightn, normaln)) * normaln - lightn;
    glm::vec3 view = glm::normalize(cameraPos - vertexPos);
    if (glm::dot(reflection, view) <= 0)
        return glm::vec3(0);
    glm::vec3 specularity = Ks;
    float result = pow(glm::dot(reflection, view), shininess);

    return specularity * result;
}
/// Returns true if there has is something between the light and the intersection
static bool hardShadows(glm::vec3 intersection, const Scene& scene, const BoundingVolumeHierarchy& bvh, glm::vec3 lightPos)
{
    Ray shadowRay;
    HitInfo hitInfoSR;
    shadowRay.direction = glm::normalize(lightPos - intersection);
    shadowRay.origin = intersection + shadowRay.direction * glm::vec3(0.00001);

    if (bvh.intersect(shadowRay, hitInfoSR)) {
        glm::vec3 intersectionSR = shadowRay.origin + shadowRay.direction * shadowRay.t;
        if (glm::distance(shadowRay.origin, intersectionSR) >= glm::distance(shadowRay.origin, lightPos))
            return false;
        if (glm::dot((intersectionSR - shadowRay.origin), hitInfoSR.normal) > 0)
            return false;
        drawRay(shadowRay, glm::vec3(1.0f, 0.0f, 0.0f));
        return true;
    }
    return false;
}
// NOTE(Mathijs): separate function to make recursion easier (could also be done with lambda + std::function).
static glm::vec3 getFinalColor(const Scene& scene, const BoundingVolumeHierarchy& bvh, Ray ray)
{
    HitInfo hitInfo;

    if (bvh.intersect(ray, hitInfo)) {
        glm::vec3 result = glm::vec3(0);
        for (PointLight light : scene.pointLights) {
            if (!hardShadows((ray.origin + ray.direction * ray.t), scene, bvh, light.position)) {
                glm::vec3 diffuse = diffuseOnly(hitInfo.material.kd, (ray.origin + ray.t * ray.direction), hitInfo.normal, light.position);
                glm::vec3 specular = phongSpecularOnly(hitInfo.material.ks, hitInfo.material.shininess, (ray.origin + ray.t * ray.direction), hitInfo.normal, light.position, ray.origin);
                result += diffuse * light.color + specular * light.color;
            }
        }
        if (result.x > 1.0) result.x = 1.0;
        if (result.y > 1.0) result.y = 1.0;
        if (result.z > 1.0) result.z = 1.0;

        drawRay(ray, result);
        return result;
    }
    else {
        // Draw a red debug ray if the ray missed.
        drawRay(ray, glm::vec3(1.0f, 0.0f, 0.0f));
        // Set the color of the pixel to black if the ray misses.
        return glm::vec3(0.0f);
    }
}

static void setOpenGLMatrices(const Trackball& camera);
static void renderOpenGL(const Scene& scene, const Trackball& camera, int selectedLight);

// This is the main rendering function. You are free to change this function in any way (including the function signature).
static void renderRayTracing(const Scene& scene, const Trackball& camera, const BoundingVolumeHierarchy& bvh, Screen& screen)
{
#ifdef USE_OPENMP
#pragma omp parallel for schedule(dynamic,2)
#endif
    for (int y = 0; y < windowResolution.y; y++) {
        for (int x = 0; x != windowResolution.x; x++) {
            // NOTE: (-1, -1) at the bottom left of the screen, (+1, +1) at the top right of the screen.
            const glm::vec2 normalizedPixelPos{
                    float(x) / windowResolution.x * 2.0f - 1.0f,
                    float(y) / windowResolution.y * 2.0f - 1.0f
            };
            const Ray cameraRay = camera.generateRay(normalizedPixelPos);
            screen.setPixel(x, y, Trace(scene, 0, cameraRay, { 0,0,0 }, bvh));
        }
    }
}
static void motionBlur(const Scene& scene, Screen& screen, Trackball& cam, const BoundingVolumeHierarchy& bvh, int iterations, bool direction, int axis)
{
#ifdef USE_OPENMP
#pragma omp parallel for schedule(dynamic,2)
#endif
    for (int y = 0; y < windowResolution.y; y++) {
        for (int x = 0; x != windowResolution.x; x++) {
            glm::vec3 sum{ 0 };
            // NOTE: (-1, -1) at the bottom left of the screen, (+1, +1) at the top right of the screen.
            const glm::vec2 normalizedPixelPos{
                    float(x) / windowResolution.x * 2.0f - 1.0f,
                    float(y) / windowResolution.y * 2.0f - 1.0f
            };
            Ray cameraRay = cam.generateRay(normalizedPixelPos);

            for (int i = 0; i < iterations; i++) {
                glm::vec3 change{ 0 };
                if (!direction)
                    change[axis] = -0.1f / (iterations);
                else
                    change[axis] = 0.1f / (iterations);
                cameraRay.origin += change;

                glm::vec3 color = getFinalColor(scene, bvh, cameraRay);
                sum += color;
            }         
            glm::vec3 finalcolor = sum / glm::vec3(iterations);
            glm::vec3 reflection = Trace(scene, 0, cameraRay, finalcolor, bvh) * glm::vec3(0.5);
            screen.setPixel(x, y, reflection);
        }
    }
}
static glm::vec3 Trace(Scene scene, int level, Ray ray, glm::vec3 color, BoundingVolumeHierarchy bvh) {
    HitInfo hitInfo;
    if (level <= 10) {
        if (bvh.intersect(ray, hitInfo)) {
            enableDrawRay = true;
            drawRay(ray, glm::vec3{ 1.0f });
            color = color + Shade(scene, level, ray, color, bvh, hitInfo);
        }
        else {
            return color;
        }
    }
    return color;
}
static glm::vec3 Shade(Scene scene, int level, Ray ray, glm::vec3 color, BoundingVolumeHierarchy bvh, HitInfo hitInfo) {
    glm::vec3 direct = getFinalColor(scene, bvh, ray);
    float x = ray.origin.x + 0.000001 * glm::normalize(glm::reflect(ray.direction, hitInfo.normal)).x;
    float y = ray.origin.y + 0.000001 * glm::normalize(glm::reflect(ray.direction, hitInfo.normal)).y;
    float z = ray.origin.z + 0.000001 * glm::normalize(glm::reflect(ray.direction, hitInfo.normal)).z;
    glm::vec3 offset = { x,y,z };
    if (hitInfo.material.ks.x != 0 || hitInfo.material.ks.y != 0 || hitInfo.material.ks.z != 0) {
        Ray reflectray = { offset + ray.t * ray.direction,glm::normalize(glm::reflect(ray.direction, hitInfo.normal)),std::numeric_limits<float>::max() };
        glm::vec3 reflectedcolor = Trace(scene, level + 1, reflectray, color, bvh);
        color = direct + hitInfo.material.ks * reflectedcolor;
    }
    else {
        color = direct;
    }
    return color;
}


int main(int argc, char** argv)
{
    Trackball::printHelp();
    std::cout << "\n Press the [R] key on your keyboard to create a ray towards the mouse cursor" << std::endl
        << std::endl;

    Window window{ "Final Project - Part 2", windowResolution, OpenGLVersion::GL2 };
    Screen screen{ windowResolution };
    Trackball camera{ &window, glm::radians(50.0f), 3.0f };
    camera.setCamera(glm::vec3(0.0f, 0.0f, 0.0f), glm::radians(glm::vec3(20.0f, 20.0f, 0.0f)), 3.0f);

    SceneType sceneType{ SceneType::SingleTriangle };
    std::optional<Ray> optDebugRay;
    Scene scene = loadScene(sceneType, dataPath);
    BoundingVolumeHierarchy bvh{ &scene };

    int bvhDebugLevel = 0;
    bool debugBVH{ false };
    bool mBlur{ false };
    int iterations = 5;
    int axis = 2;
    bool direction = true;

    ViewMode viewMode{ ViewMode::Rasterization };

    window.registerKeyCallback([&](int key, int /* scancode */, int action, int /* mods */) {
        if (action == GLFW_PRESS) {
            switch (key) {
            case GLFW_KEY_R: {
                // Shoot a ray. Produce a ray from camera to the far plane.
                const auto tmp = window.getNormalizedCursorPos();
                optDebugRay = camera.generateRay(tmp * 2.0f - 1.0f);
                viewMode = ViewMode::Rasterization;
            } break;
            case GLFW_KEY_ESCAPE: {
                window.close();
            } break;
            };
        }
        });

    int selectedLight{ 0 };
    while (!window.shouldClose()) {
        window.updateInput();

        // === Setup the UI ===
        ImGui::Begin("Final Project - Part 2");
        {
            constexpr std::array items{ "SingleTriangle", "Cube", "Cornell Box (with mirror)", "Cornell Box (spherical light and mirror)", "Monkey", "Dragon", /* "AABBs",*/ "Spheres", /*"Mixed",*/ "Custom" };
            if (ImGui::Combo("Scenes", reinterpret_cast<int*>(&sceneType), items.data(), int(items.size()))) {
                optDebugRay.reset();
                scene = loadScene(sceneType, dataPath);
                bvh = BoundingVolumeHierarchy(&scene);
                if (optDebugRay) {
                    HitInfo dummy{};
                    bvh.intersect(*optDebugRay, dummy);
                }
            }
        }
        {
            constexpr std::array items{ "Rasterization", "Ray Traced" };
            ImGui::Combo("View mode", reinterpret_cast<int*>(&viewMode), items.data(), int(items.size()));
        }
        if (ImGui::Button("Render to file")) {
            {
                using clock = std::chrono::high_resolution_clock;
                const auto start = clock::now();
                if (mBlur) {
                    motionBlur(scene, screen, camera, bvh, iterations, direction, axis);
                }
                else {
                    renderRayTracing(scene, camera, bvh, screen);
                }

                const auto end = clock::now();
                std::cout << "Time to render image: " << std::chrono::duration<float, std::milli>(end - start).count() << " milliseconds" << std::endl;
            }
            screen.writeBitmapToFile(outputPath / "render.bmp");
        }
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("Debugging");
        if (viewMode == ViewMode::Rasterization) {
            ImGui::Checkbox("Draw BVH", &debugBVH);
            if (debugBVH)
                ImGui::SliderInt("BVH Level", &bvhDebugLevel, 0, bvh.numLevels() - 1);
            ImGui::Checkbox("Motion Blur", &mBlur);
            if (mBlur) {
                ImGui::SliderInt("Iterations", &iterations, 1, 200);
                ImGui::Checkbox("Positive direction", &direction);
                ImGui::SliderInt("Axis", &axis, 0, 2);
            }
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("Lights");
        if (!scene.pointLights.empty() || !scene.sphericalLight.empty()) {
            {
                std::vector<std::string> options;
                for (size_t i = 0; i < scene.pointLights.size(); i++) {
                    options.push_back("Point Light " + std::to_string(i + 1));
                }
                for (size_t i = 0; i < scene.sphericalLight.size(); i++) {
                    options.push_back("Spherical Light " + std::to_string(i + 1));
                }

                std::vector<const char*> optionsPointers;
                std::transform(std::begin(options), std::end(options), std::back_inserter(optionsPointers),
                    [](const auto& str) { return str.c_str(); });

                ImGui::Combo("Selected light", &selectedLight, optionsPointers.data(), static_cast<int>(optionsPointers.size()));
            }

            {
                const auto showLightOptions = [](auto& light) {
                    ImGui::DragFloat3("Light position", glm::value_ptr(light.position), 0.01f, -3.0f, 3.0f);
                    ImGui::ColorEdit3("Light color", glm::value_ptr(light.color));
                    if constexpr (std::is_same_v<std::decay_t<decltype(light)>, SphericalLight>) {
                        ImGui::DragFloat("Light radius", &light.radius, 0.01f, 0.01f, 0.5f);
                    }
                };
                if (selectedLight < static_cast<int>(scene.pointLights.size())) {
                    // Draw a big yellow sphere and then the small light sphere on top.
                    showLightOptions(scene.pointLights[selectedLight]);
                }
                else {
                    // Draw a big yellow sphere and then the smaller light sphere on top.
                    showLightOptions(scene.sphericalLight[selectedLight - scene.pointLights.size()]);
                }
            }
        }

        if (ImGui::Button("Add point light")) {
            scene.pointLights.push_back(PointLight{ glm::vec3(0.0f), glm::vec3(1.0f) });
            selectedLight = int(scene.pointLights.size() - 1);
        }
        if (ImGui::Button("Add spherical light")) {
            scene.sphericalLight.push_back(SphericalLight{ glm::vec3(0.0f), 0.1f, glm::vec3(1.0f) });
            selectedLight = int(scene.pointLights.size() + scene.sphericalLight.size() - 1);
        }
        if (ImGui::Button("Remove selected light")) {
            if (selectedLight < static_cast<int>(scene.pointLights.size())) {
                scene.pointLights.erase(std::begin(scene.pointLights) + selectedLight);
            }
            else {
                scene.sphericalLight.erase(std::begin(scene.sphericalLight) + (selectedLight - scene.pointLights.size()));
            }
            selectedLight = 0;
        }

        // Clear screen.
        glClearDepth(1.0f);
        glClearColor(0.0, 0.0, 0.0, 0.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Draw either using OpenGL (rasterization) or the ray tracing function.
        switch (viewMode) {
        case ViewMode::Rasterization: {
            glPushAttrib(GL_ALL_ATTRIB_BITS);
            renderOpenGL(scene, camera, selectedLight);
            if (optDebugRay) {
                // Call getFinalColor for the debug ray. Ignore the result but tell the function that it should
                // draw the rays instead.
                enableDrawRay = true;
                (void)getFinalColor(scene, bvh, *optDebugRay);
                (void)Trace(scene, 0, *optDebugRay, { 0,0,0 }, bvh);
                enableDrawRay = false;
            }
            glPopAttrib();
        } break;
        case ViewMode::RayTracing: {
            screen.clear(glm::vec3(0.0f));
            renderRayTracing(scene, camera, bvh, screen);
            screen.setPixel(0, 0, glm::vec3(1.0f));
            screen.draw(); // Takes the image generated using ray tracing and outputs it to the screen using OpenGL.
        } break;
        default:
            break;
        };

        if (debugBVH) {
            glPushAttrib(GL_ALL_ATTRIB_BITS);
            setOpenGLMatrices(camera);
            glDisable(GL_LIGHTING);
            glEnable(GL_DEPTH_TEST);

            // Enable alpha blending. More info at:
            // https://learnopengl.com/Advanced-OpenGL/Blending
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            bvh.debugDraw(bvhDebugLevel);
            glPopAttrib();
        }

        ImGui::End();
        window.swapBuffers();
    }

    return 0; // execution never reaches this point
}

static void setOpenGLMatrices(const Trackball& camera)
{
    // Load view matrix.
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    const glm::mat4 viewMatrix = camera.viewMatrix();
    glMultMatrixf(glm::value_ptr(viewMatrix));

    // Load projection matrix.
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    const glm::mat4 projectionMatrix = camera.projectionMatrix();
    glMultMatrixf(glm::value_ptr(projectionMatrix));
}

static void renderOpenGL(const Scene& scene, const Trackball& camera, int selectedLight)
{
    // Normals will be normalized in the graphics pipeline.
    glEnable(GL_NORMALIZE);
    // Activate rendering modes.
    glEnable(GL_DEPTH_TEST);
    // Draw front and back facing triangles filled.
    glPolygonMode(GL_FRONT, GL_FILL);
    glPolygonMode(GL_BACK, GL_FILL);
    // Interpolate vertex colors over the triangles.
    glShadeModel(GL_SMOOTH);
    setOpenGLMatrices(camera);

    glDisable(GL_LIGHTING);
    // Render point lights as very small dots
    for (const auto& light : scene.pointLights)
        drawSphere(light.position, 0.01f, light.color);
    for (const auto& light : scene.sphericalLight)
        drawSphere(light.position, light.radius, light.color);

    if (!scene.pointLights.empty() || !scene.sphericalLight.empty()) {
        if (selectedLight < static_cast<int>(scene.pointLights.size())) {
            // Draw a big yellow sphere and then the small light sphere on top.
            const auto& light = scene.pointLights[selectedLight];
            drawSphere(light.position, 0.05f, glm::vec3(1, 1, 0));
            glDisable(GL_DEPTH_TEST);
            drawSphere(light.position, 0.01f, light.color);
            glEnable(GL_DEPTH_TEST);
        }
        else {
            // Draw a big yellow sphere and then the smaller light sphere on top.
            const auto& light = scene.sphericalLight[selectedLight - scene.pointLights.size()];
            drawSphere(light.position, light.radius + 0.01f, glm::vec3(1, 1, 0));
            glDisable(GL_DEPTH_TEST);
            drawSphere(light.position, light.radius, light.color);
            glEnable(GL_DEPTH_TEST);
        }
    }

    // Activate the light in the legacy OpenGL mode.
    glEnable(GL_LIGHTING);

    int i = 0;
    const auto enableLight = [&](const auto& light) {
        glEnable(GL_LIGHT0 + i);
        const glm::vec4 position4{ light.position, 1 };
        glLightfv(GL_LIGHT0 + i, GL_POSITION, glm::value_ptr(position4));
        const glm::vec4 color4{ glm::clamp(light.color, 0.0f, 1.0f), 1.0f };
        const glm::vec4 zero4{ 0.0f, 0.0f, 0.0f, 1.0f };
        glLightfv(GL_LIGHT0 + i, GL_AMBIENT, glm::value_ptr(zero4));
        glLightfv(GL_LIGHT0 + i, GL_DIFFUSE, glm::value_ptr(color4));
        glLightfv(GL_LIGHT0 + i, GL_SPECULAR, glm::value_ptr(zero4));
        // NOTE: quadratic attenuation doesn't work like you think it would in legacy OpenGL.
        // The distance is not in world space but in NDC space!
        glLightf(GL_LIGHT0 + i, GL_CONSTANT_ATTENUATION, 1.0f);
        glLightf(GL_LIGHT0 + i, GL_LINEAR_ATTENUATION, 0.0f);
        glLightf(GL_LIGHT0 + i, GL_QUADRATIC_ATTENUATION, 0.0f);
        i++;
    };
    for (const auto& light : scene.pointLights)
        enableLight(light);
    for (const auto& light : scene.sphericalLight)
        enableLight(light);

    // Draw the scene and the ray (if any).
    drawScene(scene);

    // Draw a colored sphere at the location at which the trackball is looking/rotating around.
    glDisable(GL_LIGHTING);
    drawSphere(camera.lookAt(), 0.01f, glm::vec3(0.2f, 0.2f, 1.0f));
}
