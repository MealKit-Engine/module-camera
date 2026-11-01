#include "mealkit/camera/Brain.h"

#include <algorithm>
#include <cmath>

namespace mealkit::camera {
namespace {

constexpr f32 kDegToRad = 0.0174532925f; // unit conversion

using scene::CameraBody;
using scene::CameraEase;

// Rotation whose +x, +y, +z axes are the given orthonormal columns
Quat fromBasis(Vec3 x, Vec3 y, Vec3 z) {
    const f32 trace = x.x + y.y + z.z;
    Quat q;
    if (trace > 0.0f) {
        const f32 s = std::sqrt(trace + 1.0f) * 2.0f;
        q = {(y.z - z.y) / s, (z.x - x.z) / s, (x.y - y.x) / s, 0.25f * s};
    } else if (x.x > y.y && x.x > z.z) {
        const f32 s = std::sqrt(1.0f + x.x - y.y - z.z) * 2.0f;
        q = {0.25f * s, (y.x + x.y) / s, (z.x + x.z) / s, (y.z - z.y) / s};
    } else if (y.y > z.z) {
        const f32 s = std::sqrt(1.0f + y.y - x.x - z.z) * 2.0f;
        q = {(y.x + x.y) / s, 0.25f * s, (z.y + y.z) / s, (z.x - x.z) / s};
    } else {
        const f32 s = std::sqrt(1.0f + z.z - x.x - y.y) * 2.0f;
        q = {(z.x + x.z) / s, (z.y + y.z) / s, 0.25f * s, (x.y - y.x) / s};
    }
    return normalize(q);
}

// Direction from the follow point to the eye for yaw/pitch (same convention as the old sandbox orbit camera)
Vec3 armDirection(f32 yaw, f32 pitch) {
    return {std::sin(yaw) * std::cos(pitch), std::sin(pitch), std::cos(yaw) * std::cos(pitch)};
}

f32 ease(CameraEase e, f32 t) {
    t = std::clamp(t, 0.0f, 1.0f);
    if (e == CameraEase::Linear) return t;
    if (e == CameraEase::Cut) return 1.0f;
    return t * t * (3.0f - 2.0f * t);
}

CameraPose looking(Vec3 from, Vec3 at, f32 fov) {
    const Vec3 d = at - from;
    return {from, lengthSq(d) > 1e-8f ? lookRotation(d) : Quat{}, fov};
}

// Closest point to p on a polyline
Vec3 closestOnPath(const std::vector<Vec3>& path, Vec3 p) {
    if (path.size() == 1) return path[0];
    Vec3 best = path[0];
    f32 bestDistance = lengthSq(p - best);
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        const Vec3 a = path[i], ab = path[i + 1] - path[i];
        const f32 len = lengthSq(ab);
        const f32 t = len > 1e-8f ? std::clamp(dot(p - a, ab) / len, 0.0f, 1.0f) : 0.0f;
        const Vec3 q = a + ab * t;
        if (lengthSq(p - q) < bestDistance) best = q, bestDistance = lengthSq(p - q);
    }
    return best;
}

std::vector<std::string> words(const std::string& s) {
    std::vector<std::string> out;
    for (size_t p = 0; p < s.size();) {
        while (p < s.size() && s[p] == ' ') ++p;
        const size_t start = p;
        while (p < s.size() && s[p] != ' ') ++p;
        if (p > start) out.push_back(s.substr(start, p - start));
    }
    return out;
}

} // namespace

Quat lookRotation(Vec3 forward, Vec3 up) {
    const Vec3 z = normalize(forward);
    Vec3 x = cross(up, z);
    if (lengthSq(x) < 1e-8f) x = cross(Vec3{0, 0, 1}, z); // looking straight up/down
    x = normalize(x);
    return fromBasis(x, cross(z, x), z);
}

void Brain::load(const scene::Scene& flat) {
    m_cameras.clear();
    m_objects.clear();
    for (const scene::Entity& e : flat.entities) {
        if (!e.name.empty()) m_objects.emplace_back(e.name, e.position);
        if (!e.is(scene::kTypeVirtualCamera)) continue;
        Camera c;
        c.name = e.name;
        c.fields = scene::readObject<scene::VirtualCameraFields>(e);
        c.position = e.position;
        c.rotation = e.rotation;
        c.order = static_cast<u32>(m_cameras.size());
        m_cameras.push_back(std::move(c));
    }
    m_current = -2;
    m_cutscene = {};
    m_blend = 1.0f;
}

bool Brain::setActive(std::string_view name, bool active) {
    bool found = false;
    for (Camera& c : m_cameras)
        if (c.name == name) c.fields.active = active, found = true;
    return found;
}

bool Brain::setPriority(std::string_view name, i32 priority) {
    bool found = false;
    for (Camera& c : m_cameras)
        if (c.name == name) c.fields.priority = priority, found = true;
    return found;
}

bool Brain::active(std::string_view name) const {
    for (const Camera& c : m_cameras)
        if (c.name == name) return c.fields.active;
    return false;
}

bool Brain::playCutscene(const asset::Cutscenes& cutscenes, std::string_view name) {
    const asset::Cutscenes::Cutscene* c = cutscenes.find(name);
    if (!c || !c->clip) return false;
    m_cutscene.hold = c->clip;
    m_cutscene.clip = c->clip.get();
    m_cutscene.name = c->name;
    m_cutscene.events = c->events;
    m_cutscene.time = 0.0f;
    m_cutscene.blendOut = c->row.blendOut;
    m_cutscene.blockInput = c->row.blockInput;
    startBlend(c->row.blendIn, CameraEase::EaseInOut);
    return true;
}

void Brain::stopCutscene() {
    if (!m_cutscene.clip) return;
    startBlend(m_cutscene.blendOut, CameraEase::EaseInOut);
    m_cutscene = {};
    m_current = -2; // re-pick without starting another blend
}

void Brain::startBlend(f32 seconds, CameraEase e) {
    m_from = m_pose;
    m_blendSeconds = seconds;
    m_ease = e;
    m_blend = seconds > 0.0f && e != CameraEase::Cut ? 0.0f : 1.0f;
}

i32 Brain::pick() const {
    i32 best = -1;
    for (size_t i = 0; i < m_cameras.size(); ++i) {
        const Camera& c = m_cameras[i];
        if (c.fields.active && (best < 0 || c.fields.priority >= m_cameras[static_cast<size_t>(best)].fields.priority))
            best = static_cast<i32>(i);
    }
    return best;
}

bool Brain::targetOf(const std::string& name, const BrainInput& in, Vec3& out) const {
    if (name.empty()) {
        out = m_target;
        return m_hasTarget;
    }
    if (in.findTarget && in.findTarget(name, out)) return true;
    for (const auto& [objectName, position] : m_objects)
        if (objectName == name) return out = position, true;
    return false;
}

CameraPose Brain::evaluate(Camera& c, const BrainInput& in) {
    const scene::VirtualCameraFields& f = c.fields;
    const f32 fov = f.fov > 0.0f ? f.fov : in.defaultFov;
    if (!c.placed) {
        c.yaw = f.yaw * kDegToRad;
        c.pitch = f.pitch * kDegToRad;
        c.distance = f.distance;
        c.freePosition = c.position;
    }
    Vec3 target = c.position;
    const bool hasFollow = targetOf(f.follow, in, target);
    const Vec3 wanted = target + Vec3{0, f.height, 0};
    if (!c.placed) c.focus = wanted;
    c.placed = true;
    // Dead zone and damping move the follow point toward where it wants to be
    if (lengthSq(c.focus - wanted) > 0.0f) {
        Vec3 goal = wanted;
        const Vec3 off = wanted - c.focus;
        const f32 len = length(off);
        if (f.deadZone > 0.0f) goal = len > f.deadZone ? c.focus + off * ((len - f.deadZone) / len) : c.focus;
        const f32 k = f.damping > 0.0f ? 1.0f - std::exp(-in.dt / f.damping) : 1.0f;
        c.focus = lerp(c.focus, goal, k);
    }
    Vec3 lookPoint = c.focus;
    const bool hasLookAt = !f.lookAt.empty() && targetOf(f.lookAt, in, lookPoint);

    const bool turnable = f.body == CameraBody::ThirdPerson || f.body == CameraBody::FirstPerson || f.body == CameraBody::Free;
    if (turnable) {
        c.yaw += in.look.x;
        c.pitch = std::clamp(c.pitch + in.look.y, f.minPitch * kDegToRad, f.maxPitch * kDegToRad);
    }
    if (f.body != CameraBody::Free && f.body != CameraBody::FirstPerson)
        c.distance = std::clamp(c.distance + in.zoom, f.minDistance, f.maxDistance);

    switch (f.body) {
    case CameraBody::ThirdPerson: {
        const Vec3 dir = armDirection(c.yaw, c.pitch);
        f32 distance = c.distance;
        f32 fraction = 1.0f;
        if (in.ray && in.ray(c.focus, c.focus + dir * (distance + f.collisionRadius), fraction)) // spring arm
            distance = std::max(0.0f, std::min(distance, fraction * (distance + f.collisionRadius) - f.collisionRadius));
        return looking(c.focus + dir * distance, hasLookAt ? lookPoint : c.focus, fov);
    }
    case CameraBody::FirstPerson: {
        const Vec3 forward = armDirection(c.yaw, c.pitch) * -1.0f;
        return {c.focus, lookRotation(forward), fov};
    }
    case CameraBody::TopDown:
        return looking(c.focus + armDirection(c.yaw, c.pitch) * c.distance, hasLookAt ? lookPoint : c.focus, fov);
    case CameraBody::Orbit: {
        c.spin += f.orbitSpeed * kDegToRad * in.dt;
        const Vec3 center = hasFollow ? c.focus : c.position;
        return looking(center + armDirection(c.yaw + c.spin, c.pitch) * c.distance, hasLookAt ? lookPoint : center, fov);
    }
    case CameraBody::Rail: {
        std::vector<Vec3> path;
        for (const std::string& name : words(f.rail)) {
            Vec3 p;
            if (targetOf(name, in, p)) path.push_back(p);
        }
        const Vec3 at = path.empty() ? c.position : closestOnPath(path, c.focus);
        return looking(at, hasLookAt ? lookPoint : c.focus, fov);
    }
    case CameraBody::Fixed:
        if (hasLookAt || !f.follow.empty() || !f.lookAt.empty()) return looking(c.position, lookPoint, fov);
        return {c.position, c.rotation, fov};
    case CameraBody::Free: {
        const Vec3 forward = armDirection(c.yaw, c.pitch) * -1.0f;
        const Vec3 flat = normalize(Vec3{forward.x, 0.0f, forward.z});
        const Vec3 right = cross(Vec3{0, 1, 0}, flat);
        c.freePosition += (forward * in.move.y + right * in.move.x + Vec3{0, in.rise, 0}) * (f.moveSpeed * in.dt);
        return {c.freePosition, lookRotation(forward), fov};
    }
    }
    return {c.position, c.rotation, fov};
}

void Brain::update(const BrainInput& in, std::vector<std::string>* events) {
    if (in.hasTarget) {
        m_target = in.target;
        m_hasTarget = true;
    }
    BrainInput quiet = in; // for cameras not shown: follow points keep moving, player input doesn't turn them
    quiet.look = {};
    quiet.zoom = 0.0f;
    quiet.move = {};
    quiet.rise = 0.0f;
    if (m_cutscene.clip) {
        const f32 before = m_cutscene.time;
        m_cutscene.time += in.dt;
        if (events)
            for (const auto& [at, row] : m_cutscene.events)
                if (at >= before && at < m_cutscene.time) events->push_back(row);
        if (m_cutscene.time >= m_cutscene.clip->duration) {
            const asset::CameraKey last = asset::sampleCamera(*m_cutscene.clip, m_cutscene.clip->duration);
            m_pose = {last.position, last.rotation, last.fovDegrees};
            stopCutscene(); // blends from the last frame back to the game camera
        }
    }
    CameraPose live;
    if (m_cutscene.clip) {
        const asset::CameraKey k = asset::sampleCamera(*m_cutscene.clip, m_cutscene.time);
        live = {k.position, k.rotation, k.fovDegrees};
        for (Camera& c : m_cameras) evaluate(c, quiet);
    } else {
        const i32 next = pick();
        if (next != m_current) {
            if (m_current != -2) {
                const Camera& to = next >= 0 ? m_cameras[static_cast<size_t>(next)] : m_fallback;
                startBlend(to.fields.blendSeconds, to.fields.ease);
            }
            m_current = next;
        }
        live = evaluate(next >= 0 ? m_cameras[static_cast<size_t>(next)] : m_fallback, in);
        for (size_t i = 0; i < m_cameras.size(); ++i)
            if (static_cast<i32>(i) != next && m_cameras[i].fields.active) evaluate(m_cameras[i], quiet);
    }
    if (m_blend < 1.0f) {
        m_blend = std::min(1.0f, m_blend + (m_blendSeconds > 0.0f ? in.dt / m_blendSeconds : 1.0f));
        const f32 t = ease(m_ease, m_blend);
        live = {lerp(m_from.position, live.position, t), nlerp(m_from.rotation, live.rotation, t),
                lerp(m_from.fovDegrees, live.fovDegrees, t)};
    }
    m_pose = live;
    m_pose.position += in.shake;
}

Vec3 Brain::flatForward() const {
    const Vec3 f = m_pose.forward();
    const Vec3 flat{f.x, 0.0f, f.z};
    return lengthSq(flat) > 1e-8f ? normalize(flat) : Vec3{0, 0, 1};
}

std::string Brain::currentName() const {
    if (m_cutscene.clip) return "cutscene:" + m_cutscene.name;
    return m_current >= 0 ? m_cameras[static_cast<size_t>(m_current)].name : std::string();
}

void Brain::apply(render::Viewport viewport, f32 nearPlane, f32 farPlane, render::FrameParams& frame) const {
    const f32 width = viewport.width > 0.0f ? viewport.width : 1.0f;
    const f32 height = viewport.height > 0.0f ? viewport.height : 1.0f;
    const Vec3 forward = m_pose.forward();
    frame.viewport = viewport;
    frame.viewProj = lookAtLH(m_pose.position, m_pose.position + forward, rotate(m_pose.rotation, Vec3{0, 1, 0})) *
                     perspectiveReversedZ(m_pose.fovDegrees * kDegToRad, width / height, nearPlane, farPlane);
    frame.cameraPos = m_pose.position;
    frame.cameraForward = forward;
}

} // namespace mealkit::camera
