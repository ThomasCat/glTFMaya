#include "scene/MayaScene.h"

#include <maya/MAnimControl.h>
#include <maya/MDagModifier.h>
#include <maya/MDagPath.h>
#include <maya/MDagPathArray.h>
#include <maya/MDGModifier.h>
#include <maya/MDoubleArray.h>
#include <maya/MEulerRotation.h>
#include <maya/MFloatArray.h>
#include <maya/MFloatPointArray.h>
#include <maya/MFnAnimCurve.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnIkJoint.h>
#include <maya/MFnMatrixData.h>
#include <maya/MFnMesh.h>
#include <maya/MFnSingleIndexedComponent.h>
#include <maya/MFnSkinCluster.h>
#include <maya/MFnTransform.h>
#include <maya/MGlobal.h>
#include <maya/MItDag.h>
#include <maya/MItDependencyGraph.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MMatrix.h>
#include <maya/MObjectArray.h>
#include <maya/MObjectHandle.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MPoint.h>
#include <maya/MPointArray.h>
#include <maya/MQuaternion.h>
#include <maya/MSelectionList.h>
#include <maya/MStringArray.h>
#include <maya/MTime.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MVector.h>
#include <maya/MVectorArray.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <map>
#include <set>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace gltfmaya::scene {
namespace {

std::string asString(const MString& s) { return s.asChar() ? s.asChar() : ""; }

std::string shortName(const MString& name) {
    std::string s = asString(name);
    size_t pipe = s.find_last_of('|');
    if (pipe != std::string::npos) s = s.substr(pipe + 1);
    size_t colon = s.find_last_of(':');
    if (colon != std::string::npos) s = s.substr(colon + 1);
    return s;
}

// Maya node names cannot contain '.', spaces or most punctuation. Collapse any
// run of disallowed characters to one underscore so source names survive in a
// recoverable form (the comparison canonicalises both sides before matching).
std::string sanitizeNodeName(const std::string& in) {
    std::string out;
    out.reserve(in.size());
    bool lastUnderscore = false;
    for (unsigned char c : in) {
        const bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                        (c >= '0' && c <= '9') || c == '_';
        if (ok) {
            out.push_back(static_cast<char>(c));
            lastUnderscore = false;
        } else if (!lastUnderscore) {
            out.push_back('_');
            lastUnderscore = true;
        }
    }
    while (!out.empty() && out.back() == '_') out.pop_back();
    if (out.empty()) out = "node";
    if (out[0] >= '0' && out[0] <= '9') out.insert(out.begin(), '_');
    return out;
}

// Matching key for resolving a source joint to one already in the scene. The
// source asset may name joints "ValveBiped.Bip01 Spine", which Maya stores as
// "ValveBiped_Bip01_Spine"; lower-casing and dropping every non-alphanumeric
// character makes those compare equal regardless of dots, spaces or the
// underscores Maya substitutes for them.
std::string canonicalNodeKey(const std::string& in) {
    std::string out;
    out.reserve(in.size());
    for (unsigned char c : in) {
        if (c >= 'A' && c <= 'Z') out.push_back(static_cast<char>(c - 'A' + 'a'));
        else if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')) out.push_back(static_cast<char>(c));
    }
    return out;
}

MString melQuote(const std::string& s) {
    MString out("\"");
    for (char c : s) {
        if (c == '"' || c == '\\') out += "\\";
        out += MString(std::string(1, c).c_str());
    }
    out += "\"";
    return out;
}

void applyLocalTransform(MFnTransform& fn, const ir::Transform& xf) {
    fn.setTranslation(MVector(xf.translation[0], xf.translation[1], xf.translation[2]),
                      MSpace::kTransform);
    fn.setRotation(MQuaternion(xf.rotation.x, xf.rotation.y, xf.rotation.z, xf.rotation.w));
    const double scale[3] = {xf.scale[0], xf.scale[1], xf.scale[2]};
    fn.setScale(scale);
}

ir::Transform decompose(const MMatrix& m) {
    MTransformationMatrix tm(m);
    ir::Transform xf;
    MVector t = tm.getTranslation(MSpace::kTransform);
    xf.translation = {t.x, t.y, t.z};
    MQuaternion q = tm.rotation();
    xf.rotation = {q.x, q.y, q.z, q.w};
    double s[3] = {1, 1, 1};
    tm.getScale(s, MSpace::kTransform);
    xf.scale = {s[0], s[1], s[2]};
    return xf;
}

MObject findSkinCluster(const MDagPath& meshDag) {
    MObject shape = meshDag.node();
    MItDependencyGraph it(shape, MFn::kSkinClusterFilter, MItDependencyGraph::kUpstream,
                          MItDependencyGraph::kDepthFirst, MItDependencyGraph::kNodeLevel);
    return it.isDone() ? MObject::kNullObj : it.currentItem();
}

// -- Global transform / combine-mesh IR passes ---------------------------

struct GlobalXform {
    MMatrix M;        // full row-vector transform: p' = p * M  (scale then rotate)
    MMatrix R;        // rotation-only component (for normals)
    bool identity  = true;
    bool hasScale  = false;  // any globalScale component != 1
    bool hasRotate = false;  // any globalRotateDeg component != 0
};

GlobalXform buildGlobalXform(const SceneOptions& opts) {
    GlobalXform g;
    g.hasScale  = !(opts.globalScale[0] == 1.0 && opts.globalScale[1] == 1.0 &&
                    opts.globalScale[2] == 1.0);
    g.hasRotate = !(opts.globalRotateDeg[0] == 0.0 && opts.globalRotateDeg[1] == 0.0 &&
                    opts.globalRotateDeg[2] == 0.0);
    g.identity  = !g.hasScale && !g.hasRotate;
    if (g.identity) {
        g.M.setToIdentity();
        g.R.setToIdentity();
        return g;
    }
    const double deg2rad = 3.14159265358979323846 / 180.0;
    MEulerRotation e(opts.globalRotateDeg[0] * deg2rad,
                     opts.globalRotateDeg[1] * deg2rad,
                     opts.globalRotateDeg[2] * deg2rad,
                     MEulerRotation::kXYZ);
    g.R = e.asMatrix();
    MMatrix S;
    S.setToIdentity();
    S(0, 0) = opts.globalScale[0];
    S(1, 1) = opts.globalScale[1];
    S(2, 2) = opts.globalScale[2];
    // Row-vector convention: p' = p * S * R applies scale before rotation.
    g.M = S * g.R;
    return g;
}

// Apply the global scale/rotate to a complete IR document.
//
// Scale  — applied to vertex positions AND to the translation component of
//          every joint's local transform (rotation and scale components are
//          left unchanged, so root bones carry no extra TRS from this option).
//          This mirrors CoDMayaTools, which multiplies both vertex world
//          positions and joint world translations by the same factor.
//          Inverse-bind matrices and animation translation channels are
//          rescaled to remain consistent with the new joint positions.
// Rotate — applied only to vertex positions and normals. Joint transforms,
//          inverse-bind matrices, and animation channels are not touched —
//          the user wants the skeleton to stay in Maya's coordinate system
//          ("treat it like changing the up axis of an object"), and the
//          game engine refuses non-identity rotations on root bones anyway.
//
// Skinning correctness:
//   Uniform scale (sx==sy==sz==s): exact at every pose — scaling all joint
//     local translations by s scales every world-space joint position by s,
//     so the entire deformed mesh comes out scaled by s with no distortion.
//   Non-uniform scale or rotation: bind pose is correct; non-bind poses may
//     show some distortion. Same trade-off CoDMayaTools accepts.
void applyGlobalTransform(ir::Document& doc, const SceneOptions& opts) {
    GlobalXform g = buildGlobalXform(opts);
    if (g.identity) return;

    // 1. Vertex positions (full M) and normals (rotation only).
    for (auto& mesh : doc.meshes) {
        for (auto& prim : mesh.primitives) {
            for (auto& v : prim.vertices) {
                MPoint p(v.position[0], v.position[1], v.position[2]);
                p = p * g.M;
                v.position = {p.x, p.y, p.z};
                if (v.hasNormal) {
                    MVector n(v.normal[0], v.normal[1], v.normal[2]);
                    n = n * g.R;
                    if (n.length() > 1e-12) n.normalize();
                    v.normal = {n.x, n.y, n.z};
                }
            }
        }
    }

    // Rotation never touches the skeleton — only scale does.
    if (!g.hasScale) return;

    // 2. Scale the translation component of every joint's local transform.
    // For uniform scale this scales every joint's world position by s; for
    // non-uniform scale it is correct for roots and approximate for children.
    // Rotation and scale components are deliberately left unchanged so the
    // joint's local TRS carries no residual from the global-transform option.
    const size_t nodeCount = doc.nodes.size();
    for (size_t i = 0; i < nodeCount; ++i) {
        ir::Node& n = doc.nodes[i];
        if (!n.isJoint) continue;
        n.local.translation[0] *= opts.globalScale[0];
        n.local.translation[1] *= opts.globalScale[1];
        n.local.translation[2] *= opts.globalScale[2];
    }

    // 3. Recompute inverse-bind matrices from the new joint world matrices.
    // The IB is by definition the inverse of the joint's bind-pose world
    // matrix; since we just moved the joints we have to refresh it or the
    // bind pose will look wrong in the importer / game engine.
    std::vector<MMatrix> world(nodeCount);
    MMatrix I;
    I.setToIdentity();
    std::function<void(int, const MMatrix&)> walk = [&](int idx, const MMatrix& parentWorld) {
        if (idx < 0 || idx >= int(nodeCount)) return;
        const ir::Node& n = doc.nodes[idx];
        MTransformationMatrix tm;
        tm.setTranslation(MVector(n.local.translation[0], n.local.translation[1],
                                  n.local.translation[2]),
                          MSpace::kTransform);
        MQuaternion q(n.local.rotation.x, n.local.rotation.y, n.local.rotation.z,
                      n.local.rotation.w);
        tm.setRotationQuaternion(q.x, q.y, q.z, q.w);
        double s[3] = {n.local.scale[0], n.local.scale[1], n.local.scale[2]};
        tm.setScale(s, MSpace::kTransform);
        MMatrix L = tm.asMatrix();
        world[idx] = L * parentWorld;
        for (int c : n.children) walk(c, world[idx]);
    };
    for (size_t i = 0; i < nodeCount; ++i) {
        if (doc.nodes[i].parent < 0) walk(int(i), I);
    }
    for (auto& skin : doc.skins) {
        if (skin.inverseBind.size() != skin.joints.size()) continue;
        for (size_t j = 0; j < skin.joints.size(); ++j) {
            int nodeIdx = skin.joints[j];
            if (nodeIdx < 0 || nodeIdx >= int(nodeCount)) continue;
            MMatrix newIb = world[nodeIdx].inverse();
            ir::Mat4& ib = skin.inverseBind[j];
            for (int r = 0; r < 4; ++r)
                for (int c = 0; c < 4; ++c) ib[r * 4 + c] = newIb(r, c);
        }
    }

    // 4. Animation translation channels on joints: scale by S. Rotation and
    // scale channels are untouched — they're already in joint-local space and
    // the joint's local rotation/scale isn't changing.
    for (auto& anim : doc.animations) {
        for (auto& channel : anim.channels) {
            if (channel.path != ir::AnimPath::Translation) continue;
            if (channel.node < 0 || channel.node >= int(nodeCount)) continue;
            if (!doc.nodes[channel.node].isJoint) continue;
            if (channel.sampler < 0 || channel.sampler >= int(anim.samplers.size())) continue;
            auto& sampler = anim.samplers[channel.sampler];
            int group = 3;
            int offset = 0;
            if (sampler.interpolation == ir::AnimInterp::CubicSpline) {
                group = 9;
                offset = 3;
            }
            const int keys = int(sampler.input.size());
            for (int k = 0; k < keys; ++k) {
                double* out = sampler.output.data() + size_t(k) * group + offset;
                out[0] *= opts.globalScale[0];
                out[1] *= opts.globalScale[1];
                out[2] *= opts.globalScale[2];
            }
        }
    }
}

// Rewrites every node reference in the document after a subset of nodes is
// removed. `remap` maps old index -> new index, with -1 marking a deleted node.
void remapNodeIndices(ir::Document& doc, const std::vector<int>& remap) {
    auto fix = [&](int& idx) {
        if (idx < 0 || idx >= int(remap.size())) { idx = -1; return; }
        idx = remap[idx];
    };
    for (auto& n : doc.nodes) {
        fix(n.parent);
        std::vector<int> kept;
        kept.reserve(n.children.size());
        for (int c : n.children) {
            int m = (c >= 0 && c < int(remap.size())) ? remap[c] : -1;
            if (m >= 0) kept.push_back(m);
        }
        n.children = std::move(kept);
    }
    for (auto& s : doc.skins) {
        for (auto& j : s.joints) fix(j);
        fix(s.skeleton);
    }
    for (auto& a : doc.animations) {
        std::vector<ir::AnimChannel> kept;
        kept.reserve(a.channels.size());
        for (auto& ch : a.channels) {
            int n = (ch.node >= 0 && ch.node < int(remap.size())) ? remap[ch.node] : -1;
            if (n < 0) continue;
            ch.node = n;
            kept.push_back(ch);
        }
        a.channels = std::move(kept);
    }
    std::vector<int> roots;
    roots.reserve(doc.sceneRoots.size());
    for (int r : doc.sceneRoots) {
        int m = (r >= 0 && r < int(remap.size())) ? remap[r] : -1;
        if (m >= 0) roots.push_back(m);
    }
    doc.sceneRoots = std::move(roots);
}

// Removes the given node indices and compacts the node array.
void removeNodes(ir::Document& doc, const std::unordered_set<int>& victims) {
    if (victims.empty()) return;
    std::vector<int> remap(doc.nodes.size(), -1);
    std::vector<ir::Node> kept;
    kept.reserve(doc.nodes.size() - victims.size());
    for (size_t i = 0; i < doc.nodes.size(); ++i) {
        if (victims.count(int(i))) continue;
        remap[i] = int(kept.size());
        kept.push_back(std::move(doc.nodes[i]));
    }
    doc.nodes = std::move(kept);
    remapNodeIndices(doc, remap);
}

// Collapse every mesh-bearing node down to a single mesh holding the
// concatenated primitives. Per-primitive material indices are preserved so
// multi-material assignments survive the merge. Skins are not remapped: when
// multiple distinct skins exist we keep the first and warn — the typical use
// case is several rigid props or one rigged character, not many rigs.
void combineMeshesPass(ir::Document& doc) {
    if (doc.meshes.size() <= 1) {
        int meshNodeCount = 0;
        for (const auto& n : doc.nodes)
            if (n.mesh >= 0) ++meshNodeCount;
        if (meshNodeCount <= 1) return;
    }

    // mesh index -> skin index used by the node that holds that mesh.
    std::vector<int> meshNodes;
    std::unordered_map<int, int> meshToSkin;
    for (size_t i = 0; i < doc.nodes.size(); ++i) {
        if (doc.nodes[i].mesh < 0) continue;
        meshNodes.push_back(int(i));
        meshToSkin.emplace(doc.nodes[i].mesh, doc.nodes[i].skin);
    }
    if (meshNodes.empty()) return;

    // Build a master union skin: every unique joint that appears in any of the
    // source skins gets one slot. perSkinRemap[oldSkin][oldSlot] -> masterSlot
    // is how we translate a primitive's vertex joint indices when we merge it
    // into the combined mesh. The inverse-bind matrix for a joint is taken
    // from whichever source skin first introduces it — if two skins disagree
    // on a shared joint's IB the binding is inherently ambiguous and the
    // user's rig would have rendered inconsistently to begin with.
    ir::Skin master;
    master.name = "combined";
    std::unordered_map<int, int> nodeToMasterSlot;
    std::vector<std::vector<int>> perSkinRemap(doc.skins.size());
    for (size_t s = 0; s < doc.skins.size(); ++s) {
        const ir::Skin& src = doc.skins[s];
        perSkinRemap[s].assign(src.joints.size(), -1);
        for (size_t j = 0; j < src.joints.size(); ++j) {
            int node = src.joints[j];
            int slot;
            auto it = nodeToMasterSlot.find(node);
            if (it == nodeToMasterSlot.end()) {
                slot = int(master.joints.size());
                nodeToMasterSlot[node] = slot;
                master.joints.push_back(node);
                if (j < src.inverseBind.size())
                    master.inverseBind.push_back(src.inverseBind[j]);
            } else {
                slot = it->second;
            }
            perSkinRemap[s][j] = slot;
        }
    }
    // inverseBind must stay parallel to joints. If any source skin lacked an
    // IB at the slot that introduced a joint, fill it with identity.
    if (!master.inverseBind.empty() && master.inverseBind.size() < master.joints.size()) {
        ir::Mat4 ident{};
        ident[0] = ident[5] = ident[10] = ident[15] = 1.0;
        master.inverseBind.resize(master.joints.size(), ident);
    }

    // Concatenate primitives, remapping each vertex's per-skin joint slots
    // through perSkinRemap of its source mesh's skin so the merged primitive
    // can index a single (master) joint list.
    ir::Mesh combined;
    combined.name = "combined";
    for (size_t m = 0; m < doc.meshes.size(); ++m) {
        int srcSkin = -1;
        auto it = meshToSkin.find(int(m));
        if (it != meshToSkin.end()) srcSkin = it->second;
        const bool haveRemap = srcSkin >= 0 && srcSkin < int(perSkinRemap.size()) &&
                               !perSkinRemap[srcSkin].empty();
        for (auto prim : doc.meshes[m].primitives) {
            if (haveRemap) {
                const auto& remap = perSkinRemap[srcSkin];
                for (auto& v : prim.vertices) {
                    if (!v.hasSkin) continue;
                    for (int k = 0; k < 4; ++k) {
                        if (v.weights[k] <= 0.0) { v.joints[k] = 0; continue; }
                        int old = v.joints[k];
                        if (old >= 0 && old < int(remap.size()) && remap[old] >= 0) {
                            v.joints[k] = remap[old];
                        } else {
                            v.joints[k] = 0;
                            v.weights[k] = 0.0;
                        }
                    }
                }
            }
            combined.primitives.push_back(std::move(prim));
        }
    }

    doc.meshes.clear();
    doc.meshes.push_back(std::move(combined));

    // Replace the source skins with the single master skin (or no skin at all
    // if nothing was skinned).
    int masterSkinIdx = -1;
    doc.skins.clear();
    if (!master.joints.empty()) {
        masterSkinIdx = 0;
        doc.skins.push_back(std::move(master));
    }

    bool kept = false;
    for (int ni : meshNodes) {
        if (!kept) {
            doc.nodes[ni].mesh = 0;
            doc.nodes[ni].skin = masterSkinIdx;
            // Verts are bindpose world-space and the merged mesh sits at the
            // scene root; identity local + no parent means nothing else
            // sneaks an extra transform into the skinning math.
            doc.nodes[ni].local = ir::Transform{};
            doc.nodes[ni].parent = -1;
            doc.nodes[ni].name = "combined";
            kept = true;
        } else {
            doc.nodes[ni].mesh = -1;
            doc.nodes[ni].skin = -1;
        }
    }

    // The orphan transforms only existed to carry their (now merged) mesh; if
    // nothing else references them they would still serialize as stray
    // transform nodes alongside the combined mesh. Walk the doc and drop any
    // node that has no remaining purpose. Iterate until stable so chains of
    // parent-only nodes collapse cleanly.
    auto referenced = [&](int idx) {
        for (const auto& s : doc.skins)
            for (int j : s.joints) if (j == idx) return true;
        for (const auto& a : doc.animations)
            for (const auto& c : a.channels) if (c.node == idx) return true;
        return false;
    };
    while (true) {
        std::unordered_set<int> remove;
        for (size_t i = 0; i < doc.nodes.size(); ++i) {
            const ir::Node& n = doc.nodes[i];
            if (n.mesh >= 0 || n.skin >= 0 || n.isJoint) continue;
            if (!n.children.empty()) continue;
            if (referenced(int(i))) continue;
            remove.insert(int(i));
        }
        if (remove.empty()) break;
        removeNodes(doc, remove);
    }
}

MObject componentForAllVertices(int count) {
    MIntArray indices;
    indices.setLength(count);
    for (int i = 0; i < count; ++i) indices[i] = i;
    MFnSingleIndexedComponent compFn;
    MObject comp = compFn.create(MFn::kMeshVertComponent);
    compFn.addElements(indices);
    return comp;
}

// -- Import --------------------------------------------------------------

class Importer {
public:
    explicit Importer(const ir::Document& doc) : doc_(doc) {}

    MStatus run() {
        nodePaths_.assign(doc_.nodes.size(), MDagPath());
        nodeObjects_.assign(doc_.nodes.size(), MObject::kNullObj);

        // An animation-only file is usually retargeted onto a rig already in the
        // scene. Bind its joint nodes to the existing joints by canonical name
        // before keying so the clip lands on the live skeleton instead of
        // spawning a duplicate one. Anything that fails to resolve falls back to
        // creating fresh nodes.
        const bool meshless = !documentHasMesh();
        int resolved = 0;
        if (meshless && !doc_.animations.empty()) resolved = resolveExistingJoints();

        if (resolved == 0) {
            for (int root : doc_.sceneRoots) createNode(root, MObject::kNullObj);
            for (size_t i = 0; i < doc_.nodes.size(); ++i) {
                if (doc_.nodes[i].mesh >= 0) bindMeshSkin(int(i));
            }
        } else {
            clearAnimationOnResolvedJoints();
        }

        applyAnimations();
        MGlobal::displayInfo(MString("[GLTFMaya] Imported nodes=") +
                             int(doc_.nodes.size()) + " meshes=" + int(doc_.meshes.size()) +
                             " anims=" + int(doc_.animations.size()) +
                             (resolved ? MString(" (retargeted ") + resolved + " joints)" : ""));
        return MS::kSuccess;
    }

private:
    const ir::Document& doc_;
    std::vector<MDagPath> nodePaths_;
    std::vector<MObject> nodeObjects_;

    bool documentHasMesh() const {
        for (const auto& node : doc_.nodes)
            if (node.mesh >= 0) return true;
        return false;
    }

    // Maps each joint already in the scene by its canonical name key.
    static std::unordered_map<std::string, MDagPath> sceneJointMap() {
        std::unordered_map<std::string, MDagPath> out;
        for (MItDag it(MItDag::kDepthFirst, MFn::kJoint); !it.isDone(); it.next()) {
            MDagPath path;
            if (it.getPath(path) != MS::kSuccess) continue;
            out[canonicalNodeKey(shortName(MFnDagNode(path).partialPathName()))] = path;
        }
        return out;
    }

    int resolveExistingJoints() {
        std::unordered_map<std::string, MDagPath> existing = sceneJointMap();
        if (existing.empty()) return 0;
        int matches = 0;
        for (size_t i = 0; i < doc_.nodes.size(); ++i) {
            if (!doc_.nodes[i].isJoint) continue;
            auto it = existing.find(canonicalNodeKey(doc_.nodes[i].name));
            if (it != existing.end()) {
                nodePaths_[i] = it->second;
                ++matches;
            }
        }
        return matches;
    }

    // Retargeting onto an already-animated rig would interleave new keys with
    // old ones; tear out the existing translate/rotate/scale curves first.
    void clearAnimationOnResolvedJoints() {
        static const char* attrs[] = {"translateX", "translateY", "translateZ",
                                      "rotateX",    "rotateY",    "rotateZ",
                                      "scaleX",     "scaleY",     "scaleZ"};
        MDGModifier modifier;
        bool any = false;
        for (const MDagPath& path : nodePaths_) {
            if (!path.isValid()) continue;
            MFnDependencyNode dep(path.node());
            for (const char* attr : attrs) {
                MStatus st;
                MPlug plug = dep.findPlug(attr, true, &st);
                if (st != MS::kSuccess) continue;
                MPlugArray srcs;
                plug.connectedTo(srcs, true, false);
                for (unsigned i = 0; i < srcs.length(); ++i) {
                    if (srcs[i].node().hasFn(MFn::kAnimCurve)) {
                        modifier.deleteNode(srcs[i].node());
                        any = true;
                    }
                }
            }
        }
        if (any) modifier.doIt();
    }

    void createNode(int index, MObject parent) {
        if (index < 0 || index >= int(doc_.nodes.size())) return;
        const ir::Node& node = doc_.nodes[index];
        MObject obj;
        if (node.mesh >= 0 && !node.isJoint) {
            obj = createMesh(node, parent);
        } else if (node.isJoint) {
            MFnIkJoint fn;
            obj = fn.create(parent);
            fn.setName(MString(sanitizeNodeName(node.name).c_str()));
            applyLocalTransform(fn, node.local);
        } else {
            MDagModifier mod;
            obj = mod.createNode("transform", parent);
            mod.doIt();
            MFnTransform fn(obj);
            fn.setName(MString(sanitizeNodeName(node.name.empty() ? "node" : node.name).c_str()));
            applyLocalTransform(fn, node.local);
        }
        nodeObjects_[index] = obj;
        MFnDagNode dag(obj);
        dag.getPath(nodePaths_[index]);
        for (int child : node.children) createNode(child, obj);
    }

    MObject createMesh(const ir::Node& node, MObject parent) {
        const ir::Mesh& mesh = doc_.meshes[node.mesh];

        // Combine primitives into a single shape, offsetting indices so each
        // primitive's vertices occupy a contiguous block.
        std::vector<ir::Vertex> verts;
        std::vector<int> connects;
        std::vector<int> faceMaterial;
        for (const auto& prim : mesh.primitives) {
            int base = int(verts.size());
            verts.insert(verts.end(), prim.vertices.begin(), prim.vertices.end());
            for (size_t i = 0; i + 2 < prim.indices.size(); i += 3) {
                connects.push_back(base + prim.indices[i + 0]);
                connects.push_back(base + prim.indices[i + 1]);
                connects.push_back(base + prim.indices[i + 2]);
                faceMaterial.push_back(prim.material);
            }
        }

        const int numVertices = int(verts.size());
        const int numFaces = int(connects.size() / 3);
        MFloatPointArray points;
        points.setLength(numVertices);
        for (int i = 0; i < numVertices; ++i) {
            points[i] = MFloatPoint(float(verts[i].position[0]), float(verts[i].position[1]),
                                    float(verts[i].position[2]));
        }
        MIntArray polyCounts(numFaces, 3);
        MIntArray polyConnects(numFaces * 3);
        for (int i = 0; i < numFaces * 3; ++i) polyConnects[i] = connects[i];

        MFnMesh meshFn;
        MStatus status;
        MObject created = meshFn.create(numVertices, numFaces, points, polyCounts, polyConnects,
                                        MObject::kNullObj, &status);
        if (status != MS::kSuccess) return MObject::kNullObj;

        // create() returns the shape's parent transform; reparent it under the
        // glTF node's parent so the scene hierarchy matches the source asset.
        MObject transform = created;
        if (!created.hasFn(MFn::kTransform)) transform = MFnDagNode(created).parent(0);
        if (!parent.isNull()) {
            MDagModifier mod;
            mod.reparentNode(transform, parent);
            mod.doIt();
        }

        MFnTransform xfFn(transform);
        xfFn.setName(MString(sanitizeNodeName(node.name.empty() ? mesh.name : node.name).c_str()));
        applyLocalTransform(xfFn, node.local);

        MDagPath meshDag;
        MFnDagNode(transform).getPath(meshDag);
        meshDag.extendToShape();
        meshFn.setObject(meshDag);

        applyNormals(meshFn, verts, connects, numFaces);
        applyUVs(meshFn, verts, connects, numFaces);
        assignMaterials(meshFn, meshDag, faceMaterial);
        return transform;
    }

    void applyNormals(MFnMesh& meshFn, const std::vector<ir::Vertex>& verts,
                      const std::vector<int>& connects, int numFaces) {
        if (verts.empty() || !verts[0].hasNormal) return;
        const int corners = numFaces * 3;
        MVectorArray normals;
        MIntArray faceList, vertexList;
        normals.setLength(corners);
        faceList.setLength(corners);
        vertexList.setLength(corners);
        for (int f = 0; f < numFaces; ++f) {
            for (int c = 0; c < 3; ++c) {
                int idx = f * 3 + c;
                int vid = connects[idx];
                normals[idx] = MVector(verts[vid].normal[0], verts[vid].normal[1],
                                       verts[vid].normal[2]);
                faceList[idx] = f;
                vertexList[idx] = vid;
            }
        }
        MStatus st = meshFn.setFaceVertexNormals(normals, faceList, vertexList);
        if (!st) MGlobal::displayWarning(MString("[GLTFMaya] setFaceVertexNormals: ") +
                                         st.errorString());
    }

    void applyUVs(MFnMesh& meshFn, const std::vector<ir::Vertex>& verts,
                  const std::vector<int>& connects, int numFaces) {
        if (verts.empty() || !verts[0].hasUv) return;
        MFloatArray u, v;
        u.setLength(int(verts.size()));
        v.setLength(int(verts.size()));
        for (size_t i = 0; i < verts.size(); ++i) {
            u[int(i)] = float(verts[i].uv[0]);
            v[int(i)] = float(1.0 - verts[i].uv[1]);
        }
        meshFn.setUVs(u, v);
        MIntArray uvCounts(numFaces, 3);
        MIntArray uvIds(int(connects.size()));
        for (size_t i = 0; i < connects.size(); ++i) uvIds[int(i)] = connects[i];
        meshFn.assignUVs(uvCounts, uvIds);
    }

    void assignMaterials(MFnMesh&, const MDagPath& meshDag, const std::vector<int>& faceMaterial) {
        std::unordered_map<int, std::vector<int>> byMaterial;
        for (size_t f = 0; f < faceMaterial.size(); ++f) byMaterial[faceMaterial[f]].push_back(int(f));
        const std::string meshPath = asString(meshDag.fullPathName());
        for (const auto& [mat, faces] : byMaterial) {
            std::string name = (mat >= 0 && mat < int(doc_.materials.size()))
                                   ? doc_.materials[mat].name
                                   : "lambert1";
            if (mat < 0) continue;
            MString shader, sg;
            const std::string safe = sanitizeNodeName(name.empty() ? "material" : name);
            MGlobal::executeCommand(MString("shadingNode -asShader lambert -name ") + melQuote(safe),
                                    shader);
            MGlobal::executeCommand(
                MString("sets -renderable true -noSurfaceShader true -empty -name ") +
                    melQuote(asString(shader) + "SG"),
                sg);
            MGlobal::executeCommand(MString("connectAttr -f ") + shader + ".outColor " + sg +
                                    ".surfaceShader");
            // `faces` is ascending (built in face order). Collapse runs of
            // consecutive faces into f[a:b] components and assign them all in
            // one `sets` call. Issuing one command per face turns import into
            // thousands of MEL round-trips and dominates load time.
            MString cmd = MString("sets -e -forceElement ") + sg;
            for (size_t i = 0; i < faces.size();) {
                size_t j = i;
                while (j + 1 < faces.size() && faces[j + 1] == faces[j] + 1) ++j;
                cmd += " \"";
                cmd += meshPath.c_str();
                cmd += ".f[";
                cmd += faces[i];
                if (j > i) { cmd += ":"; cmd += faces[j]; }
                cmd += "]\"";
                i = j + 1;
            }
            MGlobal::executeCommand(cmd);
        }
    }

    void bindMeshSkin(int nodeIndex) {
        const ir::Node& node = doc_.nodes[nodeIndex];
        if (node.skin < 0 || node.skin >= int(doc_.skins.size())) return;
        if (!nodePaths_[nodeIndex].isValid()) return;
        const ir::Skin& skin = doc_.skins[node.skin];
        const ir::Mesh& mesh = doc_.meshes[node.mesh];

        std::vector<ir::Vertex> verts;
        for (const auto& prim : mesh.primitives)
            verts.insert(verts.end(), prim.vertices.begin(), prim.vertices.end());
        if (verts.empty() || !verts[0].hasSkin) return;

        std::set<int> usedSlots;
        for (const auto& v : verts)
            for (int k = 0; k < 4; ++k)
                if (v.weights[k] > 0.0) usedSlots.insert(v.joints[k]);
        if (usedSlots.empty()) return;

        std::vector<int> influenceSlots(usedSlots.begin(), usedSlots.end());
        std::unordered_map<int, int> slotToColumn;
        MString cmd = "skinCluster -tsb -mi 4 -nw 1 ";
        int valid = 0;
        for (int slot : influenceSlots) {
            if (slot < 0 || slot >= int(skin.joints.size())) continue;
            int nodeIdx = skin.joints[slot];
            if (nodeIdx < 0 || nodeIdx >= int(nodePaths_.size()) || !nodePaths_[nodeIdx].isValid())
                continue;
            slotToColumn[slot] = valid++;
            cmd += "\"";
            cmd += nodePaths_[nodeIdx].fullPathName();
            cmd += "\" ";
        }
        if (valid == 0) return;
        MString meshPath = nodePaths_[nodeIndex].fullPathName();
        cmd += "\"";
        cmd += meshPath;
        cmd += "\"";

        MStringArray result;
        if (MGlobal::executeCommand(cmd, result) != MS::kSuccess || result.length() == 0) {
            MGlobal::displayWarning(MString("[GLTFMaya] skinCluster failed: ") + cmd);
            return;
        }

        MSelectionList sel;
        sel.add(result[0]);
        MObject scObj;
        sel.getDependNode(0, scObj);
        MFnSkinCluster sc(scObj);

        MDagPath meshDag = nodePaths_[nodeIndex];
        meshDag.extendToShape();

        MIntArray columns(valid);
        for (int c = 0; c < valid; ++c) columns[c] = c;

        const int numVerts = int(verts.size());
        MDoubleArray weights(numVerts * valid, 0.0);
        for (int v = 0; v < numVerts; ++v) {
            double total = 0.0;
            for (int k = 0; k < 4; ++k)
                if (verts[v].weights[k] > 0.0 && slotToColumn.count(verts[v].joints[k]))
                    total += verts[v].weights[k];
            if (total <= 0.0) total = 1.0;
            for (int k = 0; k < 4; ++k) {
                if (verts[v].weights[k] <= 0.0) continue;
                auto it = slotToColumn.find(verts[v].joints[k]);
                if (it == slotToColumn.end()) continue;
                weights[v * valid + it->second] = verts[v].weights[k] / total;
            }
        }
        sc.setWeights(meshDag, componentForAllVertices(numVerts), columns, weights, true);

        // skinCluster -tsb derives bind matrices from the joints' current
        // transforms, which assumes the rig sits at its bind pose. When the
        // source posed the joints, override bindPreMatrix with the file's
        // inverse-bind matrices so the mesh deforms to that pose instead of
        // snapping back to bind.
        if (!skin.inverseBind.empty()) {
            MFnDependencyNode scDep(scObj);
            MPlug bindPre = scDep.findPlug("bindPreMatrix", false);
            for (const auto& [slot, column] : slotToColumn) {
                if (slot < 0 || slot >= int(skin.inverseBind.size())) continue;
                unsigned idx = sc.indexForInfluenceObject(nodePaths_[skin.joints[slot]]);
                const ir::Mat4& ib = skin.inverseBind[slot];
                double d[4][4];
                for (int r = 0; r < 4; ++r)
                    for (int c = 0; c < 4; ++c) d[r][c] = ib[r * 4 + c];
                MFnMatrixData md;
                MObject mo = md.create(MMatrix(d));
                bindPre.elementByLogicalIndex(idx).setValue(mo);
            }
        }
    }

    void applyAnimations() {
        if (doc_.animations.empty()) return;
        double maxTime = 0.0;
        for (const auto& anim : doc_.animations) {
            for (const auto& channel : anim.channels) {
                if (channel.node < 0 || channel.node >= int(nodePaths_.size())) continue;
                if (!nodePaths_[channel.node].isValid()) continue;
                if (channel.sampler < 0 || channel.sampler >= int(anim.samplers.size())) continue;
                maxTime = std::max(maxTime, applyChannel(nodePaths_[channel.node], channel.path,
                                                         anim.samplers[channel.sampler]));
            }
        }
        MTime start(0.0, MTime::uiUnit());
        MTime end(maxTime, MTime::uiUnit());
        MAnimControl::setMinMaxTime(start, end);
        MAnimControl::setAnimationStartEndTime(start, end);
        MAnimControl::setCurrentTime(start);
    }

    double applyChannel(const MDagPath& path, ir::AnimPath which, const ir::AnimSampler& sampler) {
        const int keyCount = int(sampler.input.size());
        if (keyCount == 0) return 0.0;
        int stride = (which == ir::AnimPath::Rotation) ? 4 : 3;
        int group = stride;
        int offset = 0;
        if (sampler.interpolation == ir::AnimInterp::CubicSpline) {
            group = stride * 3;
            offset = stride; // middle tuple is the value
        }

        MObject node = path.node();
        MFnAnimCurve::TangentType tangent = sampler.interpolation == ir::AnimInterp::Step
                                                ? MFnAnimCurve::kTangentStep
                                                : MFnAnimCurve::kTangentLinear;
        double lastFrame = 0.0;
        std::vector<std::array<const char*, 3>> attrSets;
        std::vector<MFnAnimCurve::AnimCurveType> curveTypes;
        if (which == ir::AnimPath::Translation) {
            attrSets = {{"translateX", "translateY", "translateZ"}};
            curveTypes = {MFnAnimCurve::kAnimCurveTL};
        } else if (which == ir::AnimPath::Scale) {
            attrSets = {{"scaleX", "scaleY", "scaleZ"}};
            curveTypes = {MFnAnimCurve::kAnimCurveTU};
        } else {
            attrSets = {{"rotateX", "rotateY", "rotateZ"}};
            curveTypes = {MFnAnimCurve::kAnimCurveTA};
        }

        MFnAnimCurve curveX, curveY, curveZ;
        bool ok = makeCurve(node, attrSets[0][0], curveTypes[0], curveX) &&
                  makeCurve(node, attrSets[0][1], curveTypes[0], curveY) &&
                  makeCurve(node, attrSets[0][2], curveTypes[0], curveZ);
        if (!ok) return 0.0;

        for (int k = 0; k < keyCount; ++k) {
            MTime t(MTime(sampler.input[k], MTime::kSeconds).as(MTime::uiUnit()), MTime::uiUnit());
            lastFrame = std::max(lastFrame, t.value());
            const double* out = sampler.output.data() + size_t(k) * group + offset;
            if (which == ir::AnimPath::Rotation) {
                MQuaternion q(out[0], out[1], out[2], out[3]);
                MEulerRotation e = q.asEulerRotation();
                curveX.addKey(t, e.x, tangent, tangent);
                curveY.addKey(t, e.y, tangent, tangent);
                curveZ.addKey(t, e.z, tangent, tangent);
            } else {
                curveX.addKey(t, out[0], tangent, tangent);
                curveY.addKey(t, out[1], tangent, tangent);
                curveZ.addKey(t, out[2], tangent, tangent);
            }
        }
        return lastFrame;
    }

    static bool makeCurve(const MObject& node, const char* attr,
                          MFnAnimCurve::AnimCurveType type, MFnAnimCurve& curve) {
        MFnDependencyNode dep(node);
        MStatus st;
        MPlug plug = dep.findPlug(attr, true, &st);
        if (st != MS::kSuccess) return false;
        curve.create(plug, type, nullptr, &st);
        return st == MS::kSuccess;
    }
};

// -- Export --------------------------------------------------------------

class Exporter {
public:
    using NodeSet = std::unordered_set<unsigned int>;

    MStatus run(ir::Document& doc, const SceneOptions& opts, bool exportSelected,
                std::string& error) const {
        const bool exportAnimation = opts.exportAnimation;
        doc = ir::Document{};

        // When exporting the active selection, restrict traversal to the
        // selected nodes and their descendants. Without this the MItDag walks
        // below collect the whole scene regardless of selection.
        NodeSet allowed;
        const NodeSet* filter = nullptr;
        if (exportSelected) {
            if (!buildSelectionFilter(allowed)) {
                error = "Nothing selected to export.";
                return MS::kFailure;
            }
            filter = &allowed;
        }

        std::vector<MDagPath> joints;
        std::unordered_map<unsigned int, int> jointNodeIndex; // joint MObject hash -> doc node
        collectJoints(joints, jointNodeIndex, doc, filter);

        // Each skinCluster becomes its own glTF skin. Sharing one global skin
        // breaks when a joint is bound at different poses by different clusters
        // (e.g. a weapon base and an attachment), because a joint can then hold
        // only one inverse-bind matrix. Per-mesh skins keep each binding exact.
        collectMeshes(doc, jointNodeIndex, filter);

        // glTF flags a node as a joint only by its presence in some skin's
        // joints list. Joints that no skinCluster binds (end joints, helper
        // bones) are otherwise dropped from every skin and reimport as plain
        // transforms, so make sure each one is still listed.
        ensureAllJointsExported(doc, joints);

        if (exportAnimation && !joints.empty()) sampleAnimation(doc, joints);

        for (size_t i = 0; i < doc.nodes.size(); ++i)
            if (doc.nodes[i].parent < 0) doc.sceneRoots.push_back(int(i));

        if (doc.nodes.empty()) {
            error = "Nothing to export: no joints or meshes found.";
            return MS::kFailure;
        }

        if (opts.combineMeshes) combineMeshesPass(doc);
        applyGlobalTransform(doc, opts);
        return MS::kSuccess;
    }

private:
    // Restricts the export to the active selection plus every descendant. The
    // selection is taken at face value: if a selected mesh is skinned to joints
    // outside the selection, those joints are NOT silently pulled in, and the
    // mesh exports as-is with whatever subset of its influences the user picked.
    // This matches "Export Selection" semantics — what you select is what you get.
    bool buildSelectionFilter(NodeSet& allowed) const {
        MSelectionList sel;
        MGlobal::getActiveSelectionList(sel);
        for (unsigned i = 0; i < sel.length(); ++i) {
            MDagPath root;
            if (sel.getDagPath(i, root) != MS::kSuccess) continue;
            MItDag it;
            it.reset(root, MItDag::kDepthFirst);
            for (; !it.isDone(); it.next()) {
                MDagPath p;
                if (it.getPath(p) == MS::kSuccess)
                    allowed.insert(MObjectHandle(p.node()).hashCode());
            }
        }
        return !allowed.empty();
    }

    void collectJoints(std::vector<MDagPath>& joints,
                       std::unordered_map<unsigned int, int>& jointNodeIndex,
                       ir::Document& doc, const NodeSet* filter) const {
        MItDag it(MItDag::kDepthFirst, MFn::kJoint);
        std::unordered_map<unsigned int, int> pathToIndex;
        for (; !it.isDone(); it.next()) {
            MDagPath path;
            if (it.getPath(path) != MS::kSuccess) continue;
            if (filter && !filter->count(MObjectHandle(path.node()).hashCode())) continue;
            MFnIkJoint fn(path);
            ir::Node node;
            node.name = shortName(fn.partialPathName());
            node.isJoint = true;
            node.local = decompose(localMatrix(path));
            int index = int(doc.nodes.size());
            pathToIndex[MObjectHandle(path.node()).hashCode()] = index;
            jointNodeIndex[MObjectHandle(path.node()).hashCode()] = index;
            doc.nodes.push_back(std::move(node));
            joints.push_back(path);
        }
        // Resolve parent/children links among joints.
        for (size_t i = 0; i < joints.size(); ++i) {
            MDagPath parent = joints[i];
            if (parent.pop() == MS::kSuccess && parent.length() > 0 &&
                parent.hasFn(MFn::kJoint)) {
                auto it2 = pathToIndex.find(MObjectHandle(parent.node()).hashCode());
                if (it2 != pathToIndex.end()) {
                    doc.nodes[i].parent = it2->second;
                    doc.nodes[it2->second].children.push_back(int(i));
                }
            }
        }
    }

    static MMatrix localMatrix(const MDagPath& path) {
        MMatrix world = path.inclusiveMatrix();
        MDagPath parent = path;
        if (parent.pop() == MS::kSuccess && parent.length() > 0) {
            return world * parent.inclusiveMatrixInverse();
        }
        return world;
    }

    // collectJoints pushes joint nodes first, so joints[i] is doc node i.
    void ensureAllJointsExported(ir::Document& doc, const std::vector<MDagPath>& joints) const {
        if (joints.empty()) return;

        std::set<int> inSkin;
        for (const auto& s : doc.skins)
            for (int j : s.joints) inSkin.insert(j);

        std::vector<int> missing;
        for (size_t i = 0; i < joints.size(); ++i)
            if (!inSkin.count(int(i))) missing.push_back(int(i));
        if (missing.empty()) return;

        bool freshSkin = doc.skins.empty();
        if (freshSkin) {
            ir::Skin skel;
            skel.name = "skeleton";
            doc.skins.push_back(std::move(skel));
        }
        ir::Skin& s = doc.skins.front();
        // Keep inverseBind parallel to joints: append matrices only when the
        // target skin already carries them (a bound skin always does).
        bool needInverse = freshSkin || !s.inverseBind.empty();
        for (int idx : missing) {
            s.joints.push_back(idx);
            if (!needInverse) continue;
            MMatrix inv = joints[idx].inclusiveMatrixInverse();
            ir::Mat4 m{};
            for (int r = 0; r < 4; ++r)
                for (int c = 0; c < 4; ++c) m[r * 4 + c] = inv(r, c);
            s.inverseBind.push_back(m);
        }
    }

    void collectMeshes(ir::Document& doc,
                       const std::unordered_map<unsigned int, int>& jointNodeIndex,
                       const NodeSet* filter) const {
        MItDag it(MItDag::kDepthFirst, MFn::kMesh);
        std::set<unsigned int> seen;
        for (; !it.isDone(); it.next()) {
            MDagPath dag;
            if (it.getPath(dag) != MS::kSuccess) continue;
            if (filter && !filter->count(MObjectHandle(dag.node()).hashCode())) continue;
            MFnDagNode dagFn(dag);
            if (dagFn.isIntermediateObject()) continue;
            if (!seen.insert(MObjectHandle(dag.node()).hashCode()).second) continue;

            // Prefer the transform's name over the shape's name: in Maya the
            // transform is what the user named (e.g. "CastMesh21") while the
            // shape underneath usually shares a generic suffix like
            // "CastShape" across many meshes — non-unique and useless for
            // round-tripping or matching back to the source asset.
            MDagPath xformPath = dag;
            std::string displayName = shortName(dagFn.partialPathName());
            if (xformPath.pop() == MS::kSuccess && xformPath.length() > 0) {
                displayName = shortName(MFnDagNode(xformPath).partialPathName());
            }

            ir::Mesh mesh;
            mesh.name = displayName;
            std::vector<ir::Primitive> prims;
            int skinIndex = buildPrimitives(dag, jointNodeIndex, prims, doc);
            if (prims.empty()) continue;
            for (auto& p : prims) mesh.primitives.push_back(std::move(p));

            int meshIndex = int(doc.meshes.size());
            doc.meshes.push_back(std::move(mesh));

            ir::Node node;
            node.name = displayName;
            node.mesh = meshIndex;
            node.skin = skinIndex;
            doc.nodes.push_back(std::move(node));
        }
    }

    // Builds one glTF skin per skinCluster. The skin's inverse-bind matrices are
    // taken straight from the cluster's bindPreMatrix (which is exactly glTF's
    // inverse bind matrix), and JOINTS_0 indices are local slots into this
    // skin's joint list. Returns the skin index, or -1 when the mesh is rigid.
    int buildSkin(const MObject& skinObj, MFnSkinCluster& skin, MDagPathArray& influences,
                  std::vector<int>& influenceSlot,
                  const std::unordered_map<unsigned int, int>& jointNodeIndex,
                  ir::Document& doc) const {
        skin.setObject(skinObj);
        skin.influenceObjects(influences);
        MFnDependencyNode dep(skinObj);
        MPlug bindPre = dep.findPlug("bindPreMatrix", false);

        ir::Skin s;
        s.name = shortName(dep.name());
        influenceSlot.assign(influences.length(), -1);
        for (unsigned i = 0; i < influences.length(); ++i) {
            auto it = jointNodeIndex.find(MObjectHandle(influences[i].node()).hashCode());
            if (it == jointNodeIndex.end()) continue; // non-joint influence: unsupported, skip
            influenceSlot[i] = int(s.joints.size());
            s.joints.push_back(it->second);

            unsigned idx = skin.indexForInfluenceObject(influences[i]);
            MPlug elem = bindPre.elementByLogicalIndex(idx);
            MObject data = elem.asMObject();
            MMatrix inv;
            if (!data.isNull())
                inv = MFnMatrixData(data).matrix();
            else
                inv = influences[i].inclusiveMatrixInverse();
            ir::Mat4 m{};
            for (int r = 0; r < 4; ++r)
                for (int c = 0; c < 4; ++c) m[r * 4 + c] = inv(r, c);
            s.inverseBind.push_back(m);
        }
        if (s.joints.empty()) return -1;
        int index = int(doc.skins.size());
        doc.skins.push_back(std::move(s));
        return index;
    }

    // A skinCluster's visible output mesh carries normals recomputed from the
    // deformed geometry, which loses the authored per-vertex normals. Those
    // survive on the intermediate "Orig" shape, so read geometry from it when
    // present (topology and vertex ordering match the output shape).
    static MDagPath geometrySource(const MDagPath& outputShape) {
        MDagPath xform = outputShape;
        if (xform.pop() != MS::kSuccess) return outputShape;
        MFnDagNode tf(xform);
        for (unsigned c = 0; c < tf.childCount(); ++c) {
            MObject child = tf.child(c);
            if (!child.hasFn(MFn::kMesh)) continue;
            if (MFnDagNode(child).isIntermediateObject()) {
                MDagPath origPath;
                MFnDagNode(child).getPath(origPath);
                return origPath;
            }
        }
        return outputShape;
    }

    int buildPrimitives(const MDagPath& dag,
                        const std::unordered_map<unsigned int, int>& jointNodeIndex,
                        std::vector<ir::Primitive>& outPrims, ir::Document& doc) const {
        MObject skinObj = findSkinCluster(dag);
        MFnSkinCluster skin;
        MDagPathArray influences;
        std::vector<int> influenceSlot; // influence index -> this skin's joint slot
        int skinIndex = -1;
        if (!skinObj.isNull())
            skinIndex = buildSkin(skinObj, skin, influences, influenceSlot, jointNodeIndex, doc);

        // Read geometry from the OUTPUT shape (the user-visible mesh) — not
        // the intermediate `Orig` — and recover bindpose positions by
        // inverting the per-vertex skin matrix. This is robust against any
        // deformer chain between Orig and output (deleteComponent nodes that
        // strip verts, blendShapes, etc.); reading Orig directly desyncs from
        // the skin's weight table the moment topology differs between the
        // two, which manifests as wholesale mesh-positioning bugs.
        MFnMesh fnMesh(dag);
        const int numVertices = fnMesh.numVertices();
        MPointArray points;
        fnMesh.getPoints(points, MSpace::kWorld);

        // Precompute each influence's current "skin matrix" = bindPreMatrix *
        // jointWorldMatrix. The combined per-vertex transform is the weighted
        // sum of these; we invert it per vertex to undo the skinning and get
        // the bindpose world position the renderer will re-skin from.
        std::vector<MMatrix> influenceSkin;
        if (skinIndex >= 0) {
            MPlug bindPre = MFnDependencyNode(skinObj).findPlug("bindPreMatrix", false);
            influenceSkin.reserve(influences.length());
            for (unsigned i = 0; i < influences.length(); ++i) {
                unsigned logical = skin.indexForInfluenceObject(influences[i]);
                MPlug elem = bindPre.elementByLogicalIndex(logical);
                MObject data = elem.asMObject();
                MMatrix bpm;
                if (!data.isNull()) bpm = MFnMatrixData(data).matrix();
                else bpm = influences[i].inclusiveMatrixInverse();
                influenceSkin.push_back(bpm * influences[i].inclusiveMatrix());
            }
        }

        std::vector<ir::Vertex> masterVerts(numVertices);
        for (int v = 0; v < numVertices; ++v) {
            MPoint pCurrent = points[v];
            MPoint pBind = pCurrent;
            if (skinIndex >= 0) {
                MFnSingleIndexedComponent compFn;
                MObject comp = compFn.create(MFn::kMeshVertComponent);
                compFn.addElement(v);
                MDoubleArray ws;
                unsigned ic = 0;
                skin.getWeights(dag, comp, ws, ic);
                MMatrix combined;
                for (int r = 0; r < 4; ++r)
                    for (int c = 0; c < 4; ++c) combined(r, c) = 0.0;
                double wsum = 0.0;
                for (unsigned i = 0; i < ws.length() && i < influenceSkin.size(); ++i) {
                    if (ws[i] <= 0.0) continue;
                    const MMatrix& M = influenceSkin[i];
                    for (int r = 0; r < 4; ++r)
                        for (int c = 0; c < 4; ++c) combined(r, c) += ws[i] * M(r, c);
                    wsum += ws[i];
                }
                if (wsum > 1e-9) pBind = pCurrent * combined.inverse();
                applyVertexWeights(skin, dag, influences, influenceSlot, v, masterVerts[v]);
            }
            masterVerts[v].position = {pBind.x, pBind.y, pBind.z};
        }

        // shaderIndices[f] indexes into shaders[] for face f (-1 unassigned).
        // We split per shader, but only when more than one shader actually
        // contributes — preserving the original 1:1 vertex order in the
        // common single-material case keeps downstream consumers (and the
        // round-trip test's index-pairing) stable.
        MObjectArray shaders;
        MIntArray shaderIndices;
        MFnMesh(dag).getConnectedShaders(dag.instanceNumber(), shaders, shaderIndices);

        std::set<int> usedShaders;
        for (unsigned f = 0; f < shaderIndices.length(); ++f) usedShaders.insert(shaderIndices[f]);
        const bool multiMaterial = usedShaders.size() > 1;

        // First pass: populate per-master normals and UVs. (Source assets
        // pre-split vertices at hard edges and UV seams, so each master vertex
        // sees a single consistent corner value across its incident faces.)
        std::vector<int> faceShader;
        std::vector<std::vector<int>> faceTris;
        faceShader.reserve(shaderIndices.length());
        faceTris.reserve(shaderIndices.length());
        // Iterate the OUTPUT shape's polygons too — Orig may have different
        // topology when a deleteComponent (or any topology-changing deformer)
        // sits between Orig and the user-visible mesh. Reading polygon data
        // from `dag` keeps face/vert indices consistent with the positions
        // and skin weights we just collected.
        MItMeshPolygon poly(dag);
        int faceIndex = 0;
        for (; !poly.isDone(); poly.next(), ++faceIndex) {
            MIntArray faceVerts;
            poly.getVertices(faceVerts);
            for (unsigned c = 0; c < faceVerts.length(); ++c) {
                int vid = faceVerts[c];
                MVector n;
                poly.getNormal(c, n, MSpace::kWorld);
                masterVerts[vid].normal = {n.x, n.y, n.z};
                masterVerts[vid].hasNormal = true;
                if (poly.hasUVs()) {
                    float2 uv{0.0f, 0.0f};
                    poly.getUV(c, uv);
                    masterVerts[vid].uv = {uv[0], 1.0 - uv[1]};
                    masterVerts[vid].hasUv = true;
                }
            }
            MIntArray triVerts;
            MPointArray triPoints;
            poly.getTriangles(triPoints, triVerts);
            std::vector<int> tris;
            tris.reserve(triVerts.length());
            for (unsigned i = 0; i < triVerts.length(); ++i) tris.push_back(triVerts[i]);
            faceTris.push_back(std::move(tris));
            faceShader.push_back(faceIndex < int(shaderIndices.length()) ? shaderIndices[faceIndex]
                                                                          : -1);
        }

        if (!multiMaterial) {
            // Fast path preserves master vertex ordering 1:1.
            ir::Primitive prim;
            prim.vertices = std::move(masterVerts);
            for (const auto& tris : faceTris)
                for (int v : tris) prim.indices.push_back(v);
            prim.material = registerMaterial(shaders, doc);
            if (!prim.vertices.empty() && !prim.indices.empty()) outPrims.push_back(std::move(prim));
            return skinIndex;
        }

        // Multi-material split: one primitive per shader group, vertices
        // remapped per group so the primitives are self-contained.
        std::map<int, ir::Primitive> groups; // ordered: assigned first, -1 last
        std::map<int, std::unordered_map<int, int>> remap;
        for (size_t f = 0; f < faceTris.size(); ++f) {
            int s = faceShader[f];
            auto pit = groups.find(s);
            if (pit == groups.end()) {
                ir::Primitive empty;
                MObjectArray one;
                if (s >= 0 && s < int(shaders.length())) one.append(shaders[s]);
                empty.material = registerMaterial(one, doc);
                pit = groups.emplace(s, std::move(empty)).first;
                remap.emplace(s, std::unordered_map<int, int>{});
            }
            ir::Primitive& prim = pit->second;
            auto& rmap = remap[s];
            for (int masterVid : faceTris[f]) {
                auto rit = rmap.find(masterVid);
                int localVid;
                if (rit == rmap.end()) {
                    localVid = int(prim.vertices.size());
                    rmap.emplace(masterVid, localVid);
                    prim.vertices.push_back(masterVerts[masterVid]);
                } else {
                    localVid = rit->second;
                }
                prim.indices.push_back(localVid);
            }
        }

        // Emit in key order with -1 (unassigned) last.
        for (auto& [s, prim] : groups) {
            if (s < 0) continue;
            if (prim.vertices.empty() || prim.indices.empty()) continue;
            outPrims.push_back(std::move(prim));
        }
        auto unassigned = groups.find(-1);
        if (unassigned != groups.end() && !unassigned->second.vertices.empty() &&
            !unassigned->second.indices.empty()) {
            outPrims.push_back(std::move(unassigned->second));
        }
        return skinIndex;
    }

    void applyVertexWeights(MFnSkinCluster& skin, const MDagPath& dag,
                            const MDagPathArray& influences, const std::vector<int>& influenceSlot,
                            int vertex, ir::Vertex& vert) const {
        MFnSingleIndexedComponent compFn;
        MObject comp = compFn.create(MFn::kMeshVertComponent);
        compFn.addElement(vertex);
        MDoubleArray ws;
        unsigned count = 0;
        skin.getWeights(dag, comp, ws, count);

        std::vector<std::pair<int, double>> influencesByWeight;
        for (unsigned i = 0; i < ws.length() && i < influenceSlot.size(); ++i) {
            if (ws[i] <= 1e-6 || influenceSlot[i] < 0) continue;
            influencesByWeight.emplace_back(influenceSlot[i], ws[i]);
        }
        std::sort(influencesByWeight.begin(), influencesByWeight.end(),
                  [](const auto& a, const auto& b) { return a.second > b.second; });
        if (influencesByWeight.size() > 4) influencesByWeight.resize(4);

        double total = 0.0;
        for (const auto& [slot, w] : influencesByWeight) total += w;
        if (total <= 0.0) total = 1.0;
        for (size_t i = 0; i < influencesByWeight.size(); ++i) {
            vert.joints[i] = influencesByWeight[i].first;
            vert.weights[i] = influencesByWeight[i].second / total;
        }
        vert.hasSkin = true;
    }

    int registerMaterial(const MObjectArray& shaders, ir::Document& doc) const {
        if (shaders.length() == 0) return -1;
        // getConnectedShaders returns shading engines; the material name the
        // user sees is the surface shader wired into the group, not the group
        // node itself (whose name is often unrelated).
        std::string name = surfaceShaderName(shaders[0]);
        if (name.empty()) return -1;
        for (size_t i = 0; i < doc.materials.size(); ++i)
            if (doc.materials[i].name == name) return int(i);
        ir::Material mat;
        mat.name = name;
        int index = int(doc.materials.size());
        doc.materials.push_back(std::move(mat));
        return index;
    }

    static std::string surfaceShaderName(const MObject& shadingEngine) {
        MFnDependencyNode sg(shadingEngine);
        MStatus st;
        MPlug ss = sg.findPlug("surfaceShader", false, &st);
        if (st == MS::kSuccess) {
            MPlugArray srcs;
            ss.connectedTo(srcs, true, false);
            if (srcs.length() > 0) return shortName(MFnDependencyNode(srcs[0].node()).name());
        }
        return shortName(sg.name());
    }

    void sampleAnimation(ir::Document& doc, const std::vector<MDagPath>& joints) const {
        int start = int(MAnimControl::animationStartTime().as(MTime::uiUnit()));
        int end = int(MAnimControl::animationEndTime().as(MTime::uiUnit()));
        if (end < start) return;

        ir::Animation anim;
        anim.name = "animation";
        std::vector<int> frames;
        for (int f = start; f <= end; ++f) frames.push_back(f);

        struct Track { ir::AnimSampler t, r, s; };
        std::vector<Track> tracks(joints.size());

        for (int f : frames) {
            MAnimControl::setCurrentTime(MTime(double(f), MTime::uiUnit()));
            double seconds = MTime(double(f), MTime::uiUnit()).as(MTime::kSeconds);
            for (size_t j = 0; j < joints.size(); ++j) {
                ir::Transform xf = decompose(localMatrix(joints[j]));
                tracks[j].t.input.push_back(seconds);
                tracks[j].t.output.insert(tracks[j].t.output.end(),
                                          {xf.translation[0], xf.translation[1], xf.translation[2]});
                tracks[j].r.input.push_back(seconds);
                tracks[j].r.output.insert(
                    tracks[j].r.output.end(),
                    {xf.rotation.x, xf.rotation.y, xf.rotation.z, xf.rotation.w});
                tracks[j].s.input.push_back(seconds);
                tracks[j].s.output.insert(tracks[j].s.output.end(),
                                          {xf.scale[0], xf.scale[1], xf.scale[2]});
            }
        }

        for (size_t j = 0; j < joints.size(); ++j) {
            auto add = [&](ir::AnimSampler sampler, ir::AnimPath path) {
                int samplerIdx = int(anim.samplers.size());
                anim.samplers.push_back(std::move(sampler));
                ir::AnimChannel channel;
                channel.node = int(j);
                channel.path = path;
                channel.sampler = samplerIdx;
                anim.channels.push_back(channel);
            };
            add(std::move(tracks[j].t), ir::AnimPath::Translation);
            add(std::move(tracks[j].r), ir::AnimPath::Rotation);
            add(std::move(tracks[j].s), ir::AnimPath::Scale);
        }

        MAnimControl::setCurrentTime(MTime(double(start), MTime::uiUnit()));
        doc.animations.push_back(std::move(anim));
    }
};

} // namespace

MStatus MayaScene::importDocument(const ir::Document& doc, const SceneOptions& opts) {
    // Bake the user-requested global transform and mesh combine into the IR
    // before instantiating any Maya nodes, so the Importer stays unaware of
    // these options and the created scene carries no residual transforms.
    ir::Document working = doc;
    if (opts.combineMeshes) combineMeshesPass(working);
    applyGlobalTransform(working, opts);
    Importer importer(working);
    return importer.run();
}

MStatus MayaScene::exportDocument(ir::Document& doc, const SceneOptions& opts, bool exportSelected,
                                  std::string& error) const {
    Exporter exporter;
    return exporter.run(doc, opts, exportSelected, error);
}

} // namespace gltfmaya::scene
