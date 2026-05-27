#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace gltfmaya::ir {

using Vec2 = std::array<double, 2>;
using Vec3 = std::array<double, 3>;
using Vec4 = std::array<double, 4>;
using Mat4 = std::array<double, 16>;

// Quaternion stored xyzw to match glTF node rotation ordering.
struct Quat {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
};

struct Transform {
    Vec3 translation{0.0, 0.0, 0.0};
    Quat rotation{};
    Vec3 scale{1.0, 1.0, 1.0};
};

// A node is any entry in the glTF node array: a joint, a mesh holder, or a
// pure grouping transform. `mesh`/`skin` index into Document when >= 0.
struct Node {
    std::string name;
    int parent = -1;
    Transform local{};
    std::vector<int> children;
    int mesh = -1;
    int skin = -1;
    bool isJoint = false;
};

// Per-corner attributes. Joint indices reference positions within a Skin's
// joints array (not raw node indices); MayaScene resolves them via the skin.
struct Vertex {
    Vec3 position{0.0, 0.0, 0.0};
    Vec3 normal{0.0, 0.0, 1.0};
    Vec2 uv{0.0, 0.0};
    std::array<int, 4> joints{0, 0, 0, 0};
    Vec4 weights{0.0, 0.0, 0.0, 0.0};
    bool hasNormal = false;
    bool hasUv = false;
    bool hasSkin = false;
};

struct Primitive {
    std::vector<Vertex> vertices;
    std::vector<int> indices; // triangle list
    int material = -1;
};

struct Mesh {
    std::string name;
    std::vector<Primitive> primitives;
};

struct Skin {
    std::string name;
    std::vector<int> joints;        // node indices
    std::vector<Mat4> inverseBind;  // parallel to joints; empty if absent
    int skeleton = -1;
};

struct Material {
    std::string name;
    Vec4 baseColor{1.0, 1.0, 1.0, 1.0};
    double metallic = 1.0;
    double roughness = 1.0;
    bool doubleSided = false;
};

enum class AnimPath { Translation, Rotation, Scale };
enum class AnimInterp { Linear, Step, CubicSpline };

struct AnimSampler {
    std::vector<double> input;   // keyframe times in seconds
    std::vector<double> output;  // flattened component values
    AnimInterp interpolation = AnimInterp::Linear;
};

struct AnimChannel {
    int node = -1;
    AnimPath path = AnimPath::Translation;
    int sampler = -1;
};

struct Animation {
    std::string name;
    std::vector<AnimSampler> samplers;
    std::vector<AnimChannel> channels;
};

struct Document {
    std::string generator = "GLTFMaya";
    std::vector<Node> nodes;
    std::vector<Mesh> meshes;
    std::vector<Skin> skins;
    std::vector<Material> materials;
    std::vector<Animation> animations;
    std::vector<int> sceneRoots;
};

} // namespace gltfmaya::ir
