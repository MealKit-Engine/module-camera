#pragma once

#include "mealkit/asset/Cutscene.h"
#include "mealkit/core/Math.h"
#include "mealkit/render/Renderer.h"
#include "mealkit/scene/Scene.h"

#include <functional>
#include <string>
#include <string_view>
#include <vector>

// Camera brain (docs/ROADMAP.md stage 4 "Virtual cameras + cutscenes", docs/CODING_MODEL.md "Camera control and staging").
// Virtual cameras are scene objects (scene::kTypeVirtualCamera, fields scene::VirtualCameraFields). The brain shows the active camera
// with the highest priority (ties: the later object) and blends to a new one over its blendSeconds. Game code never moves cameras;
// it only calls setActive/setPriority, and playCutscene for Blender camera animations (asset/Cutscene.h).
// View-side only: nothing here enters the simulation. Wall collision rays are view-side queries given by the game (BrainInput::ray).
namespace mealkit::camera {

struct CameraPose {
    Vec3 position;
    Quat rotation;            // looks along rotate(rotation, +z)
    f32 fovDegrees = 60.0f;   // vertical
    Vec3 forward() const { return rotate(rotation, Vec3{0, 0, 1}); }
};

// What the game gives the brain every frame
struct BrainInput {
    f32 dt = 0.0f;
    Vec2 look;            // turn the player-controlled bodies (radians: x = yaw, y = pitch up)
    f32 zoom = 0.0f;      // distance change (m, + = farther)
    Vec2 move;            // Free body movement (x = right, y = forward, length ≤ 1)
    f32 rise = 0.0f;      // Free body up/down (-1..1)
    bool hasTarget = false;
    Vec3 target;          // the possessed Pawn's position (default follow target); the last one stays when it disappears
    f32 defaultFov = 60.0f; // fov for cameras with fov 0 (player setting)
    Vec3 shake;           // impulses added to the result (feedback::Player::cameraShake)
    // Named follow/look-at targets that move (game objects). Return false to fall back to scene objects of that name
    std::function<bool(std::string_view name, Vec3& out)> findTarget;
    // Wall query for spring arms: from → to, true and the hit fraction (0..1) if something is in between
    std::function<bool(Vec3 from, Vec3 to, f32& fraction)> ray;
};

class Brain {
public:
    // Collects the virtual cameras of a flattened scene (scene::flattened). Keeps nothing else from the scene except rail points/targets
    void load(const scene::Scene& flat);
    // Camera used when no virtual camera is active (e.g. a scene without cameras). Its position is the origin
    void setFallback(const scene::VirtualCameraFields& fields) { m_fallback.fields = fields; }

    bool setActive(std::string_view name, bool active);  // false if no camera has that name
    bool setPriority(std::string_view name, i32 priority);
    bool active(std::string_view name) const;

    // Plays a cutscene now (blends in over its blendIn). Solo staging calls it directly; shared cutscenes from a tick event
    bool playCutscene(const asset::Cutscenes& cutscenes, std::string_view name);
    void stopCutscene(); // blends back over blendOut
    bool cutscenePlaying() const { return m_cutscene.clip != nullptr; }
    bool blocksInput() const { return m_cutscene.clip && m_cutscene.blockInput; }

    // Every frame. events gets the feedback rows of cutscene events crossed this update
    void update(const BrainInput& in, std::vector<std::string>* events = nullptr);

    const CameraPose& pose() const { return m_pose; }
    // Horizontal forward of the shown camera (for turning move input into a world direction)
    Vec3 flatForward() const;
    // Name of the camera currently shown ("" = fallback, "cutscene:<name>" during a cutscene)
    std::string currentName() const;
    // Fills viewport, view/projection, camera position and listener direction
    void apply(render::Viewport viewport, f32 nearPlane, f32 farPlane, render::FrameParams& frame) const;

    struct Camera {
        std::string name;
        scene::VirtualCameraFields fields;
        Vec3 position;          // placed transform (Fixed, Orbit without a target)
        Quat rotation;
        // runtime
        f32 yaw = 0.0f, pitch = 0.0f, distance = 0.0f; // player-controlled bodies (radians, m)
        f32 spin = 0.0f;                               // Orbit angle
        Vec3 focus;                                    // follow point after dead zone/damping
        Vec3 freePosition;
        bool placed = false;
        u32 order = 0; // scene order (priority ties)
    };

private:
    CameraPose evaluate(Camera& camera, const BrainInput& in);
    i32 pick() const; // index into m_cameras, -1 = fallback
    bool targetOf(const std::string& name, const BrainInput& in, Vec3& out) const;
    void startBlend(f32 seconds, scene::CameraEase ease);

    std::vector<Camera> m_cameras;
    std::vector<std::pair<std::string, Vec3>> m_objects; // scene object positions by name (targets, rail points)
    Camera m_fallback;
    i32 m_current = -2; // -2 = nothing shown yet
    Vec3 m_target;
    bool m_hasTarget = false;

    struct Playing {
        const asset::CameraClip* clip = nullptr;
        std::shared_ptr<const asset::CameraClip> hold; // keeps the clip alive across table reloads
        std::string name;
        std::vector<std::pair<f32, std::string>> events;
        f32 time = 0.0f;
        f32 blendOut = 0.0f;
        bool blockInput = false;
    } m_cutscene;

    CameraPose m_pose, m_from;
    f32 m_blend = 1.0f, m_blendSeconds = 0.0f; // progress 0..1 from m_from
    scene::CameraEase m_ease = scene::CameraEase::EaseInOut;
};

// Rotation looking along forward with y up (forward is normalized inside)
Quat lookRotation(Vec3 forward, Vec3 up = Vec3{0, 1, 0});

} // namespace mealkit::camera
