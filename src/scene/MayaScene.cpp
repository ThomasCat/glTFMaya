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
#include <set>
#include <sstream>
#include <unordered_map>
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
            for (int face : faces) {
                MString cmd = "sets -e -forceElement ";
                cmd += sg;
                cmd += " \"";
                cmd += meshPath.c_str();
                cmd += ".f[";
                cmd += face;
                cmd += "]\"";
                MGlobal::executeCommand(cmd);
            }
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
    MStatus run(ir::Document& doc, bool exportAnimation, std::string& error) const {
        doc = ir::Document{};

        std::vector<MDagPath> joints;
        std::unordered_map<unsigned int, int> jointNodeIndex; // joint MObject hash -> doc node
        collectJoints(joints, jointNodeIndex, doc);

        int skinIndex = -1;
        if (!joints.empty()) {
            ir::Skin skin;
            skin.name = "skin";
            for (size_t i = 0; i < joints.size(); ++i) {
                skin.joints.push_back(int(i)); // joint doc-node indices are 0..N-1
                MMatrix inv = joints[i].inclusiveMatrixInverse();
                ir::Mat4 m{};
                for (int r = 0; r < 4; ++r)
                    for (int c = 0; c < 4; ++c) m[r * 4 + c] = inv(r, c);
                skin.inverseBind.push_back(m);
            }
            doc.skins.push_back(std::move(skin));
            skinIndex = 0;
        }

        collectMeshes(doc, joints, jointNodeIndex, skinIndex);

        if (exportAnimation && !joints.empty()) sampleAnimation(doc, joints);

        for (size_t i = 0; i < doc.nodes.size(); ++i)
            if (doc.nodes[i].parent < 0) doc.sceneRoots.push_back(int(i));

        if (doc.nodes.empty()) {
            error = "Nothing to export: no joints or meshes found.";
            return MS::kFailure;
        }
        return MS::kSuccess;
    }

private:
    void collectJoints(std::vector<MDagPath>& joints,
                       std::unordered_map<unsigned int, int>& jointNodeIndex,
                       ir::Document& doc) const {
        MItDag it(MItDag::kDepthFirst, MFn::kJoint);
        std::unordered_map<unsigned int, int> pathToIndex;
        for (; !it.isDone(); it.next()) {
            MDagPath path;
            if (it.getPath(path) != MS::kSuccess) continue;
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

    void collectMeshes(ir::Document& doc, const std::vector<MDagPath>& joints,
                       const std::unordered_map<unsigned int, int>& jointNodeIndex,
                       int skinIndex) const {
        MItDag it(MItDag::kDepthFirst, MFn::kMesh);
        std::set<unsigned int> seen;
        for (; !it.isDone(); it.next()) {
            MDagPath dag;
            if (it.getPath(dag) != MS::kSuccess) continue;
            MFnDagNode dagFn(dag);
            if (dagFn.isIntermediateObject()) continue;
            if (!seen.insert(MObjectHandle(dag.node()).hashCode()).second) continue;

            ir::Mesh mesh;
            mesh.name = shortName(MFnDagNode(dag).partialPathName());
            ir::Primitive prim;
            buildPrimitive(dag, joints, jointNodeIndex, prim, doc);
            if (prim.vertices.empty()) continue;
            mesh.primitives.push_back(std::move(prim));

            int meshIndex = int(doc.meshes.size());
            doc.meshes.push_back(std::move(mesh));

            ir::Node node;
            node.name = shortName(MFnDagNode(dag).partialPathName());
            node.mesh = meshIndex;
            if (skinIndex >= 0 && prim_hasSkin(doc.meshes[meshIndex])) node.skin = skinIndex;
            doc.nodes.push_back(std::move(node));
        }
    }

    static bool prim_hasSkin(const ir::Mesh& mesh) {
        return !mesh.primitives.empty() && !mesh.primitives[0].vertices.empty() &&
               mesh.primitives[0].vertices[0].hasSkin;
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

    void buildPrimitive(const MDagPath& dag, const std::vector<MDagPath>& joints,
                        const std::unordered_map<unsigned int, int>& jointNodeIndex,
                        ir::Primitive& prim, ir::Document& doc) const {
        MObject skinObj = findSkinCluster(dag);
        MFnSkinCluster skin;
        MDagPathArray influences;
        std::vector<int> influenceSlot; // influence index -> skin.joints slot
        if (!skinObj.isNull()) {
            skin.setObject(skinObj);
            skin.influenceObjects(influences);
            influenceSlot.resize(influences.length(), -1);
            for (unsigned i = 0; i < influences.length(); ++i) {
                auto it = jointNodeIndex.find(MObjectHandle(influences[i].node()).hashCode());
                if (it != jointNodeIndex.end()) influenceSlot[i] = it->second;
            }
        }

        MDagPath geomDag = skinObj.isNull() ? dag : geometrySource(dag);
        MFnMesh fnMesh(geomDag);
        const int numVertices = fnMesh.numVertices();

        // Material per face: capture connected shaders for primitive material 0.
        MObjectArray shaders;
        MIntArray shaderIndices;
        MFnMesh(dag).getConnectedShaders(dag.instanceNumber(), shaders, shaderIndices);
        prim.material = registerMaterial(shaders, doc);

        prim.vertices.resize(numVertices);
        MPointArray points;
        fnMesh.getPoints(points, MSpace::kObject);
        for (int v = 0; v < numVertices; ++v) {
            ir::Vertex& vert = prim.vertices[v];
            vert.position = {points[v].x, points[v].y, points[v].z};
            if (!skinObj.isNull()) applyVertexWeights(skin, dag, influences, influenceSlot, v, vert);
        }

        // Normals and UVs are stored per face-vertex in Maya. The source asset
        // pre-split vertices at hard edges and UV seams, so each vertex carries
        // a single consistent corner value; read it per corner rather than the
        // averaged per-vertex normal, which would blend across those splits.
        MItMeshPolygon poly(geomDag);
        for (; !poly.isDone(); poly.next()) {
            MIntArray faceVerts;
            poly.getVertices(faceVerts);
            for (unsigned c = 0; c < faceVerts.length(); ++c) {
                int vid = faceVerts[c];
                MVector n;
                poly.getNormal(c, n, MSpace::kObject);
                prim.vertices[vid].normal = {n.x, n.y, n.z};
                prim.vertices[vid].hasNormal = true;
                if (poly.hasUVs()) {
                    float2 uv{0.0f, 0.0f};
                    poly.getUV(c, uv);
                    prim.vertices[vid].uv = {uv[0], 1.0 - uv[1]};
                    prim.vertices[vid].hasUv = true;
                }
            }
            MIntArray triVerts;
            MPointArray triPoints;
            poly.getTriangles(triPoints, triVerts);
            for (unsigned i = 0; i < triVerts.length(); ++i) prim.indices.push_back(triVerts[i]);
        }
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
        MFnDependencyNode sg(shaders[0]);
        ir::Material mat;
        mat.name = shortName(sg.name());
        // Strip the "SG" suffix Maya appends to shading groups for a cleaner name.
        if (mat.name.size() > 2 && mat.name.compare(mat.name.size() - 2, 2, "SG") == 0)
            mat.name.resize(mat.name.size() - 2);
        int index = int(doc.materials.size());
        doc.materials.push_back(std::move(mat));
        return index;
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

MStatus MayaScene::importDocument(const ir::Document& doc) {
    Importer importer(doc);
    return importer.run();
}

MStatus MayaScene::exportDocument(ir::Document& doc, bool exportAnimation, std::string& error) const {
    Exporter exporter;
    return exporter.run(doc, exportAnimation, error);
}

} // namespace gltfmaya::scene
