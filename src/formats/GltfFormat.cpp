#include "formats/GltfFormat.h"

#include "io/Json.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace gltfmaya::formats {
namespace {

using io::Json;

constexpr std::uint32_t kGlbMagic = 0x46546C67; // "glTF"
constexpr std::uint32_t kChunkJson = 0x4E4F534A; // "JSON"
constexpr std::uint32_t kChunkBin = 0x004E4942;  // "BIN\0"

// glTF accessor.componentType enum values.
enum ComponentType {
    kByte = 5120,
    kUnsignedByte = 5121,
    kShort = 5122,
    kUnsignedShort = 5123,
    kUnsignedInt = 5125,
    kFloat = 5126,
};

int componentByteSize(int type) {
    switch (type) {
        case kByte: case kUnsignedByte: return 1;
        case kShort: case kUnsignedShort: return 2;
        case kUnsignedInt: case kFloat: return 4;
        default: return 0;
    }
}

int componentsOf(const std::string& type) {
    if (type == "SCALAR") return 1;
    if (type == "VEC2") return 2;
    if (type == "VEC3") return 3;
    if (type == "VEC4") return 4;
    if (type == "MAT2") return 4;
    if (type == "MAT3") return 9;
    if (type == "MAT4") return 16;
    return 0;
}

std::string directoryOf(const std::string& path) {
    size_t slash = path.find_last_of("/\\");
    return slash == std::string::npos ? std::string() : path.substr(0, slash + 1);
}

bool readWholeFile(const std::string& path, std::vector<std::uint8_t>& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    in.seekg(0, std::ios::end);
    std::streamoff size = in.tellg();
    in.seekg(0, std::ios::beg);
    out.resize(static_cast<size_t>(size));
    if (size > 0) in.read(reinterpret_cast<char*>(out.data()), size);
    return static_cast<bool>(in);
}

std::vector<std::uint8_t> decodeBase64(const std::string& in) {
    static const std::string table =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::vector<int> rev(256, -1);
    for (int i = 0; i < 64; ++i) rev[static_cast<unsigned char>(table[i])] = i;
    std::vector<std::uint8_t> out;
    int val = 0, bits = -8;
    for (unsigned char c : in) {
        if (c == '=' || rev[c] == -1) continue;
        val = (val << 6) | rev[c];
        bits += 6;
        if (bits >= 0) {
            out.push_back(static_cast<std::uint8_t>((val >> bits) & 0xFF));
            bits -= 8;
        }
    }
    return out;
}

// -- Reading -------------------------------------------------------------

class Reader {
public:
    Reader(const std::string& path) : dir_(directoryOf(path)) {}

    bool read(const std::vector<std::uint8_t>& file, ir::Document& doc, std::string& error) {
        std::string jsonText;
        if (!splitContainer(file, jsonText, error)) return false;

        json_ = Json::parse(jsonText, error);
        if (!error.empty()) return false;

        if (!loadBuffers(error)) return false;
        buildDocument(doc);
        return true;
    }

private:
    std::string dir_;
    Json json_;
    std::vector<std::uint8_t> glbBin_;
    bool hasGlbBin_ = false;
    std::vector<std::vector<std::uint8_t>> buffers_;

    bool splitContainer(const std::vector<std::uint8_t>& file, std::string& jsonText,
                        std::string& error) {
        if (file.size() >= 12) {
            std::uint32_t magic;
            std::memcpy(&magic, file.data(), 4);
            if (magic == kGlbMagic) return splitGlb(file, jsonText, error);
        }
        jsonText.assign(file.begin(), file.end());
        return true;
    }

    bool splitGlb(const std::vector<std::uint8_t>& file, std::string& jsonText,
                  std::string& error) {
        std::uint32_t length;
        std::memcpy(&length, file.data() + 8, 4);
        size_t total = std::min<size_t>(length, file.size());
        size_t off = 12;
        bool gotJson = false;
        while (off + 8 <= total) {
            std::uint32_t chunkLen, chunkType;
            std::memcpy(&chunkLen, file.data() + off, 4);
            std::memcpy(&chunkType, file.data() + off + 4, 4);
            off += 8;
            if (off + chunkLen > total) { error = "GLB chunk overruns file"; return false; }
            if (chunkType == kChunkJson) {
                jsonText.assign(file.begin() + off, file.begin() + off + chunkLen);
                gotJson = true;
            } else if (chunkType == kChunkBin) {
                glbBin_.assign(file.begin() + off, file.begin() + off + chunkLen);
                hasGlbBin_ = true;
            }
            off += chunkLen;
        }
        if (!gotJson) { error = "GLB missing JSON chunk"; return false; }
        return true;
    }

    bool loadBuffers(std::string& error) {
        const Json* buffers = json_.find("buffers");
        if (!buffers || !buffers->isArray()) return true;
        for (const auto& b : buffers->items()) {
            const Json* uri = b.find("uri");
            if (!uri) {
                buffers_.push_back(glbBin_);
                continue;
            }
            const std::string& u = uri->asString();
            if (u.rfind("data:", 0) == 0) {
                size_t comma = u.find(',');
                buffers_.push_back(comma == std::string::npos
                                       ? std::vector<std::uint8_t>()
                                       : decodeBase64(u.substr(comma + 1)));
            } else {
                std::vector<std::uint8_t> bytes;
                if (!readWholeFile(dir_ + u, bytes)) {
                    error = "Cannot open buffer file: " + u;
                    return false;
                }
                buffers_.push_back(std::move(bytes));
            }
        }
        return true;
    }

    double normalize(double raw, int componentType) const {
        switch (componentType) {
            case kByte: return std::max(-1.0, raw / 127.0);
            case kUnsignedByte: return raw / 255.0;
            case kShort: return std::max(-1.0, raw / 32767.0);
            case kUnsignedShort: return raw / 65535.0;
            default: return raw;
        }
    }

    double readComponent(const std::uint8_t* p, int componentType) const {
        switch (componentType) {
            case kByte: return static_cast<double>(*reinterpret_cast<const std::int8_t*>(p));
            case kUnsignedByte: return static_cast<double>(*p);
            case kShort: { std::int16_t v; std::memcpy(&v, p, 2); return v; }
            case kUnsignedShort: { std::uint16_t v; std::memcpy(&v, p, 2); return v; }
            case kUnsignedInt: { std::uint32_t v; std::memcpy(&v, p, 4); return v; }
            case kFloat: { float v; std::memcpy(&v, p, 4); return v; }
            default: return 0.0;
        }
    }

    // Decodes an accessor into a flat row-major array of doubles. `components`
    // receives the per-element component count.
    std::vector<double> decodeAccessor(int index, int& components) const {
        std::vector<double> out;
        const Json* accessors = json_.find("accessors");
        if (!accessors || index < 0 || index >= int(accessors->items().size())) return out;
        const Json& acc = accessors->items()[index];

        int componentType = acc.intAt("componentType");
        components = componentsOf(acc.stringAt("type"));
        int count = acc.intAt("count");
        bool normalized = acc.boolAt("normalized");
        int compSize = componentByteSize(componentType);
        if (components == 0 || compSize == 0) return out;

        out.assign(static_cast<size_t>(count) * components, 0.0);

        const Json* bvIndex = acc.find("bufferView");
        if (!bvIndex) return out; // sparse-only / zero-initialised

        const Json& bv = json_.find("bufferViews")->items()[bvIndex->asInt()];
        int bufferIdx = bv.intAt("buffer");
        if (bufferIdx < 0 || bufferIdx >= int(buffers_.size())) return out;
        const std::vector<std::uint8_t>& buffer = buffers_[bufferIdx];

        size_t bvOffset = static_cast<size_t>(bv.intAt("byteOffset"));
        size_t accOffset = static_cast<size_t>(acc.intAt("byteOffset"));
        int defaultStride = compSize * components;
        int stride = bv.intAt("byteStride", 0);
        if (stride == 0) stride = defaultStride;

        size_t base = bvOffset + accOffset;
        for (int e = 0; e < count; ++e) {
            size_t elem = base + static_cast<size_t>(e) * stride;
            for (int c = 0; c < components; ++c) {
                size_t at = elem + static_cast<size_t>(c) * compSize;
                if (at + compSize > buffer.size()) continue;
                double raw = readComponent(buffer.data() + at, componentType);
                out[static_cast<size_t>(e) * components + c] =
                    normalized ? normalize(raw, componentType) : raw;
            }
        }
        return out;
    }

    static ir::Transform transformFromNode(const Json& node) {
        ir::Transform xf;
        if (const Json* m = node.find("matrix")) {
            std::array<double, 16> col{};
            for (int i = 0; i < 16 && i < int(m->items().size()); ++i) {
                col[i] = m->items()[i].asNumber();
            }
            decomposeMatrix(col, xf);
            return xf;
        }
        if (const Json* t = node.find("translation")) {
            for (int i = 0; i < 3 && i < int(t->items().size()); ++i)
                xf.translation[i] = t->items()[i].asNumber();
        }
        if (const Json* r = node.find("rotation")) {
            const auto& v = r->items();
            if (v.size() >= 4) {
                xf.rotation = {v[0].asNumber(), v[1].asNumber(), v[2].asNumber(), v[3].asNumber()};
            }
        }
        if (const Json* s = node.find("scale")) {
            for (int i = 0; i < 3 && i < int(s->items().size()); ++i)
                xf.scale[i] = s->items()[i].asNumber();
        }
        return xf;
    }

    // glTF matrices are column-major. Extract TRS for the rare matrix node form.
    static void decomposeMatrix(const std::array<double, 16>& m, ir::Transform& xf) {
        xf.translation = {m[12], m[13], m[14]};
        ir::Vec3 col0{m[0], m[1], m[2]};
        ir::Vec3 col1{m[4], m[5], m[6]};
        ir::Vec3 col2{m[8], m[9], m[10]};
        auto length = [](const ir::Vec3& v) {
            return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        };
        double sx = length(col0), sy = length(col1), sz = length(col2);
        double det = m[0] * (m[5] * m[10] - m[6] * m[9]) -
                     m[4] * (m[1] * m[10] - m[2] * m[9]) +
                     m[8] * (m[1] * m[6] - m[2] * m[5]);
        if (det < 0) sx = -sx;
        xf.scale = {sx, sy, sz};
        double r[9] = {m[0] / sx, m[1] / sx, m[2] / sx,
                       m[4] / sy, m[5] / sy, m[6] / sy,
                       m[8] / sz, m[9] / sz, m[10] / sz};
        double trace = r[0] + r[4] + r[8];
        ir::Quat q;
        if (trace > 0.0) {
            double s = std::sqrt(trace + 1.0) * 2.0;
            q.w = 0.25 * s;
            q.x = (r[5] - r[7]) / s;
            q.y = (r[6] - r[2]) / s;
            q.z = (r[1] - r[3]) / s;
        } else if (r[0] > r[4] && r[0] > r[8]) {
            double s = std::sqrt(1.0 + r[0] - r[4] - r[8]) * 2.0;
            q.w = (r[5] - r[7]) / s;
            q.x = 0.25 * s;
            q.y = (r[3] + r[1]) / s;
            q.z = (r[6] + r[2]) / s;
        } else if (r[4] > r[8]) {
            double s = std::sqrt(1.0 + r[4] - r[0] - r[8]) * 2.0;
            q.w = (r[6] - r[2]) / s;
            q.x = (r[3] + r[1]) / s;
            q.y = 0.25 * s;
            q.z = (r[7] + r[5]) / s;
        } else {
            double s = std::sqrt(1.0 + r[8] - r[0] - r[4]) * 2.0;
            q.w = (r[1] - r[3]) / s;
            q.x = (r[6] + r[2]) / s;
            q.y = (r[7] + r[5]) / s;
            q.z = 0.25 * s;
        }
        xf.rotation = q;
    }

    void buildDocument(ir::Document& doc) {
        doc.generator = json_.find("asset")
                            ? json_.find("asset")->stringAt("generator", "GLTFMaya")
                            : "GLTFMaya";
        buildMaterials(doc);
        buildNodes(doc);
        buildSkins(doc);
        buildMeshes(doc);
        buildAnimations(doc);
        buildScene(doc);
    }

    void buildMaterials(ir::Document& doc) {
        const Json* materials = json_.find("materials");
        if (!materials) return;
        for (const auto& m : materials->items()) {
            ir::Material mat;
            mat.name = m.stringAt("name");
            mat.doubleSided = m.boolAt("doubleSided");
            if (const Json* pbr = m.find("pbrMetallicRoughness")) {
                mat.metallic = pbr->numberAt("metallicFactor", 1.0);
                mat.roughness = pbr->numberAt("roughnessFactor", 1.0);
                if (const Json* bc = pbr->find("baseColorFactor")) {
                    for (int i = 0; i < 4 && i < int(bc->items().size()); ++i)
                        mat.baseColor[i] = bc->items()[i].asNumber();
                }
            }
            doc.materials.push_back(std::move(mat));
        }
    }

    void buildNodes(ir::Document& doc) {
        const Json* nodes = json_.find("nodes");
        if (!nodes) return;
        doc.nodes.resize(nodes->items().size());
        for (size_t i = 0; i < nodes->items().size(); ++i) {
            const Json& n = nodes->items()[i];
            ir::Node& node = doc.nodes[i];
            node.name = n.stringAt("name");
            node.local = transformFromNode(n);
            node.mesh = n.intAt("mesh", -1);
            node.skin = n.intAt("skin", -1);
            if (const Json* ch = n.find("children")) {
                for (const auto& c : ch->items()) node.children.push_back(c.asInt());
            }
        }
        for (size_t i = 0; i < doc.nodes.size(); ++i) {
            for (int c : doc.nodes[i].children) {
                if (c >= 0 && c < int(doc.nodes.size())) doc.nodes[c].parent = int(i);
            }
        }
    }

    void buildSkins(ir::Document& doc) {
        const Json* skins = json_.find("skins");
        if (!skins) return;
        for (const auto& s : skins->items()) {
            ir::Skin skin;
            skin.name = s.stringAt("name");
            skin.skeleton = s.intAt("skeleton", -1);
            if (const Json* joints = s.find("joints")) {
                for (const auto& j : joints->items()) {
                    int idx = j.asInt();
                    skin.joints.push_back(idx);
                    if (idx >= 0 && idx < int(doc.nodes.size())) doc.nodes[idx].isJoint = true;
                }
            }
            if (const Json* ibm = s.find("inverseBindMatrices")) {
                int comps = 0;
                std::vector<double> data = decodeAccessor(ibm->asInt(), comps);
                if (comps == 16) {
                    size_t mats = data.size() / 16;
                    for (size_t m = 0; m < mats; ++m) {
                        ir::Mat4 mat{};
                        for (int k = 0; k < 16; ++k) mat[k] = data[m * 16 + k];
                        skin.inverseBind.push_back(mat);
                    }
                }
            }
            doc.skins.push_back(std::move(skin));
        }
    }

    void buildMeshes(ir::Document& doc) {
        const Json* meshes = json_.find("meshes");
        if (!meshes) return;
        for (const auto& m : meshes->items()) {
            ir::Mesh mesh;
            mesh.name = m.stringAt("name");
            const Json* prims = m.find("primitives");
            if (prims) {
                for (const auto& p : prims->items()) mesh.primitives.push_back(buildPrimitive(p));
            }
            doc.meshes.push_back(std::move(mesh));
        }
    }

    ir::Primitive buildPrimitive(const Json& p) {
        ir::Primitive prim;
        prim.material = p.intAt("material", -1);
        const Json* attrs = p.find("attributes");
        if (!attrs) return prim;

        int comps = 0;
        auto get = [&](const char* name) -> std::vector<double> {
            const Json* a = attrs->find(name);
            if (!a) return {};
            return decodeAccessor(a->asInt(), comps);
        };

        std::vector<double> pos = get("POSITION");
        size_t vertexCount = pos.size() / 3;
        std::vector<double> nrm = get("NORMAL");
        std::vector<double> uv = get("TEXCOORD_0");
        std::vector<double> joints = get("JOINTS_0");
        std::vector<double> weights = get("WEIGHTS_0");

        prim.vertices.resize(vertexCount);
        for (size_t v = 0; v < vertexCount; ++v) {
            ir::Vertex& vert = prim.vertices[v];
            vert.position = {pos[v * 3 + 0], pos[v * 3 + 1], pos[v * 3 + 2]};
            if (nrm.size() >= (v + 1) * 3) {
                vert.normal = {nrm[v * 3 + 0], nrm[v * 3 + 1], nrm[v * 3 + 2]};
                vert.hasNormal = true;
            }
            if (uv.size() >= (v + 1) * 2) {
                vert.uv = {uv[v * 2 + 0], uv[v * 2 + 1]};
                vert.hasUv = true;
            }
            if (joints.size() >= (v + 1) * 4 && weights.size() >= (v + 1) * 4) {
                for (int k = 0; k < 4; ++k) {
                    vert.joints[k] = static_cast<int>(joints[v * 4 + k]);
                    vert.weights[k] = weights[v * 4 + k];
                }
                vert.hasSkin = true;
            }
        }

        if (const Json* indices = p.find("indices")) {
            std::vector<double> idx = decodeAccessor(indices->asInt(), comps);
            prim.indices.reserve(idx.size());
            for (double d : idx) prim.indices.push_back(static_cast<int>(d));
        } else {
            prim.indices.reserve(vertexCount);
            for (size_t v = 0; v < vertexCount; ++v) prim.indices.push_back(int(v));
        }
        return prim;
    }

    void buildAnimations(ir::Document& doc) {
        const Json* anims = json_.find("animations");
        if (!anims) return;
        for (const auto& a : anims->items()) {
            ir::Animation anim;
            anim.name = a.stringAt("name");
            const Json* samplers = a.find("samplers");
            if (samplers) {
                for (const auto& s : samplers->items()) {
                    ir::AnimSampler sampler;
                    int comps = 0;
                    sampler.input = decodeAccessor(s.intAt("input", -1), comps);
                    sampler.output = decodeAccessor(s.intAt("output", -1), comps);
                    std::string interp = s.stringAt("interpolation", "LINEAR");
                    sampler.interpolation = interp == "STEP" ? ir::AnimInterp::Step
                                          : interp == "CUBICSPLINE" ? ir::AnimInterp::CubicSpline
                                          : ir::AnimInterp::Linear;
                    anim.samplers.push_back(std::move(sampler));
                }
            }
            const Json* channels = a.find("channels");
            if (channels) {
                for (const auto& c : channels->items()) {
                    ir::AnimChannel channel;
                    channel.sampler = c.intAt("sampler", -1);
                    if (const Json* target = c.find("target")) {
                        channel.node = target->intAt("node", -1);
                        std::string path = target->stringAt("path");
                        channel.path = path == "rotation" ? ir::AnimPath::Rotation
                                     : path == "scale" ? ir::AnimPath::Scale
                                     : ir::AnimPath::Translation;
                    }
                    anim.channels.push_back(channel);
                }
            }
            doc.animations.push_back(std::move(anim));
        }
    }

    void buildScene(ir::Document& doc) {
        int sceneIdx = json_.intAt("scene", 0);
        const Json* scenes = json_.find("scenes");
        if (scenes && sceneIdx >= 0 && sceneIdx < int(scenes->items().size())) {
            const Json& scene = scenes->items()[sceneIdx];
            if (const Json* roots = scene.find("nodes")) {
                for (const auto& r : roots->items()) doc.sceneRoots.push_back(r.asInt());
            }
        }
        if (doc.sceneRoots.empty()) {
            for (size_t i = 0; i < doc.nodes.size(); ++i)
                if (doc.nodes[i].parent < 0) doc.sceneRoots.push_back(int(i));
        }
    }
};

// -- Writing -------------------------------------------------------------

class BufferBuilder {
public:
    std::vector<std::uint8_t> bytes;

    void align(size_t boundary) {
        while (bytes.size() % boundary != 0) bytes.push_back(0);
    }
    template <typename T>
    void put(T v) {
        const auto* p = reinterpret_cast<const std::uint8_t*>(&v);
        bytes.insert(bytes.end(), p, p + sizeof(T));
    }
};

class Writer {
public:
    Writer() {
        json_ = Json::makeObject();
        bufferViews_ = Json::makeArray();
        accessors_ = Json::makeArray();
    }

    bool write(const std::string& path, const ir::Document& doc, Container container,
               std::string& error) {
        Json asset = Json::makeObject();
        asset.set("generator", Json(doc.generator));
        asset.set("version", Json("2.0"));
        json_.set("asset", std::move(asset));

        writeMaterials(doc);
        writeMeshes(doc);
        writeSkins(doc);
        writeNodes(doc);
        writeAnimations(doc);
        writeScene(doc);

        const std::string binName = baseName(path) + ".bin";

        Json buffers = Json::makeArray();
        Json buffer = Json::makeObject();
        buffer.set("byteLength", Json(int(blob_.bytes.size())));
        if (container == Container::Gltf) buffer.set("uri", Json(binName));
        buffers.push_back(std::move(buffer));
        json_.set("buffers", std::move(buffers));

        if (!bufferViews_.items().empty()) json_.set("bufferViews", std::move(bufferViews_));
        if (!accessors_.items().empty()) json_.set("accessors", std::move(accessors_));

        return container == Container::Glb ? writeGlb(path, error)
                                           : writeGltf(path, binName, error);
    }

private:
    Json json_;
    Json bufferViews_;
    Json accessors_;
    BufferBuilder blob_;

    static std::string baseName(const std::string& path) {
        size_t slash = path.find_last_of("/\\");
        std::string file = slash == std::string::npos ? path : path.substr(slash + 1);
        size_t dot = file.find_last_of('.');
        return dot == std::string::npos ? file : file.substr(0, dot);
    }

    int addBufferView(size_t byteOffset, size_t byteLength, int target) {
        Json bv = Json::makeObject();
        bv.set("buffer", Json(0));
        bv.set("byteOffset", Json(int(byteOffset)));
        bv.set("byteLength", Json(int(byteLength)));
        if (target != 0) bv.set("target", Json(target));
        int index = int(bufferViews_.items().size());
        bufferViews_.push_back(std::move(bv));
        return index;
    }

    int addFloatAccessor(const std::vector<double>& data, int components, const std::string& type,
                         int target, bool withBounds) {
        blob_.align(4);
        size_t offset = blob_.bytes.size();
        for (double d : data) blob_.put(static_cast<float>(d));
        int count = components ? int(data.size()) / components : 0;
        int bv = addBufferView(offset, blob_.bytes.size() - offset, target);

        Json acc = Json::makeObject();
        acc.set("bufferView", Json(bv));
        acc.set("componentType", Json(int(kFloat)));
        acc.set("count", Json(count));
        acc.set("type", Json(type));
        if (withBounds) addVecBounds(acc, data, components);
        int index = int(accessors_.items().size());
        accessors_.push_back(std::move(acc));
        return index;
    }

    static void addVecBounds(Json& acc, const std::vector<double>& data, int components) {
        if (components <= 0 || data.empty()) return;
        std::vector<double> lo(components, std::numeric_limits<double>::max());
        std::vector<double> hi(components, std::numeric_limits<double>::lowest());
        for (size_t i = 0; i + components <= data.size(); i += components) {
            for (int c = 0; c < components; ++c) {
                lo[c] = std::min(lo[c], data[i + c]);
                hi[c] = std::max(hi[c], data[i + c]);
            }
        }
        Json minJ = Json::makeArray(), maxJ = Json::makeArray();
        for (int c = 0; c < components; ++c) {
            minJ.push_back(Json(static_cast<float>(lo[c])));
            maxJ.push_back(Json(static_cast<float>(hi[c])));
        }
        acc.set("min", std::move(minJ));
        acc.set("max", std::move(maxJ));
    }

    int addJointAccessor(const std::vector<std::array<int, 4>>& joints) {
        blob_.align(4);
        size_t offset = blob_.bytes.size();
        for (const auto& j : joints)
            for (int k = 0; k < 4; ++k) blob_.put(static_cast<std::uint16_t>(j[k]));
        int bv = addBufferView(offset, blob_.bytes.size() - offset, 34962);
        Json acc = Json::makeObject();
        acc.set("bufferView", Json(bv));
        acc.set("componentType", Json(int(kUnsignedShort)));
        acc.set("count", Json(int(joints.size())));
        acc.set("type", Json("VEC4"));
        int index = int(accessors_.items().size());
        accessors_.push_back(std::move(acc));
        return index;
    }

    int addIndexAccessor(const std::vector<int>& indices) {
        blob_.align(4);
        size_t offset = blob_.bytes.size();
        bool wide = false;
        for (int i : indices) if (i > 65535) { wide = true; break; }
        if (wide) {
            for (int i : indices) blob_.put(static_cast<std::uint32_t>(i));
        } else {
            for (int i : indices) blob_.put(static_cast<std::uint16_t>(i));
        }
        int bv = addBufferView(offset, blob_.bytes.size() - offset, 34963);
        Json acc = Json::makeObject();
        acc.set("bufferView", Json(bv));
        acc.set("componentType", Json(wide ? int(kUnsignedInt) : int(kUnsignedShort)));
        acc.set("count", Json(int(indices.size())));
        acc.set("type", Json("SCALAR"));
        int index = int(accessors_.items().size());
        accessors_.push_back(std::move(acc));
        return index;
    }

    void writeMaterials(const ir::Document& doc) {
        if (doc.materials.empty()) return;
        Json arr = Json::makeArray();
        for (const auto& m : doc.materials) {
            Json mat = Json::makeObject();
            if (!m.name.empty()) mat.set("name", Json(m.name));
            Json pbr = Json::makeObject();
            Json bc = Json::makeArray();
            for (double c : m.baseColor) bc.push_back(Json(c));
            pbr.set("baseColorFactor", std::move(bc));
            pbr.set("metallicFactor", Json(m.metallic));
            pbr.set("roughnessFactor", Json(m.roughness));
            mat.set("pbrMetallicRoughness", std::move(pbr));
            if (m.doubleSided) mat.set("doubleSided", Json(true));
            arr.push_back(std::move(mat));
        }
        json_.set("materials", std::move(arr));
    }

    void writeMeshes(const ir::Document& doc) {
        if (doc.meshes.empty()) return;
        Json arr = Json::makeArray();
        for (const auto& mesh : doc.meshes) {
            Json m = Json::makeObject();
            if (!mesh.name.empty()) m.set("name", Json(mesh.name));
            Json prims = Json::makeArray();
            for (const auto& prim : mesh.primitives) prims.push_back(writePrimitive(prim));
            m.set("primitives", std::move(prims));
            arr.push_back(std::move(m));
        }
        json_.set("meshes", std::move(arr));
    }

    Json writePrimitive(const ir::Primitive& prim) {
        const size_t n = prim.vertices.size();
        std::vector<double> pos, nrm, uv, weights;
        std::vector<std::array<int, 4>> joints;
        pos.reserve(n * 3);
        bool hasNormal = n > 0 && prim.vertices[0].hasNormal;
        bool hasUv = n > 0 && prim.vertices[0].hasUv;
        bool hasSkin = n > 0 && prim.vertices[0].hasSkin;
        for (const auto& v : prim.vertices) {
            pos.insert(pos.end(), {v.position[0], v.position[1], v.position[2]});
            if (hasNormal) nrm.insert(nrm.end(), {v.normal[0], v.normal[1], v.normal[2]});
            if (hasUv) uv.insert(uv.end(), {v.uv[0], v.uv[1]});
            if (hasSkin) {
                joints.push_back(v.joints);
                weights.insert(weights.end(),
                               {v.weights[0], v.weights[1], v.weights[2], v.weights[3]});
            }
        }

        Json attrs = Json::makeObject();
        attrs.set("POSITION", Json(addFloatAccessor(pos, 3, "VEC3", 34962, true)));
        if (hasNormal) attrs.set("NORMAL", Json(addFloatAccessor(nrm, 3, "VEC3", 34962, false)));
        if (hasUv) attrs.set("TEXCOORD_0", Json(addFloatAccessor(uv, 2, "VEC2", 34962, false)));
        if (hasSkin) {
            attrs.set("JOINTS_0", Json(addJointAccessor(joints)));
            attrs.set("WEIGHTS_0", Json(addFloatAccessor(weights, 4, "VEC4", 34962, false)));
        }

        Json p = Json::makeObject();
        p.set("attributes", std::move(attrs));
        p.set("indices", Json(addIndexAccessor(prim.indices)));
        if (prim.material >= 0) p.set("material", Json(prim.material));
        return p;
    }

    void writeSkins(const ir::Document& doc) {
        if (doc.skins.empty()) return;
        Json arr = Json::makeArray();
        for (const auto& skin : doc.skins) {
            Json s = Json::makeObject();
            if (!skin.name.empty()) s.set("name", Json(skin.name));
            if (!skin.inverseBind.empty()) {
                std::vector<double> data;
                data.reserve(skin.inverseBind.size() * 16);
                for (const auto& m : skin.inverseBind)
                    for (double d : m) data.push_back(d);
                s.set("inverseBindMatrices", Json(addFloatAccessor(data, 16, "MAT4", 0, false)));
            }
            if (skin.skeleton >= 0) s.set("skeleton", Json(skin.skeleton));
            Json joints = Json::makeArray();
            for (int j : skin.joints) joints.push_back(Json(j));
            s.set("joints", std::move(joints));
            arr.push_back(std::move(s));
        }
        json_.set("skins", std::move(arr));
    }

    void writeNodes(const ir::Document& doc) {
        if (doc.nodes.empty()) return;
        Json arr = Json::makeArray();
        for (const auto& node : doc.nodes) {
            Json n = Json::makeObject();
            if (!node.name.empty()) n.set("name", Json(node.name));
            const ir::Transform& t = node.local;
            if (t.translation != ir::Vec3{0, 0, 0}) {
                Json tr = Json::makeArray();
                for (double d : t.translation) tr.push_back(Json(d));
                n.set("translation", std::move(tr));
            }
            if (t.rotation.x != 0 || t.rotation.y != 0 || t.rotation.z != 0 || t.rotation.w != 1) {
                Json rot = Json::makeArray();
                rot.push_back(Json(t.rotation.x));
                rot.push_back(Json(t.rotation.y));
                rot.push_back(Json(t.rotation.z));
                rot.push_back(Json(t.rotation.w));
                n.set("rotation", std::move(rot));
            }
            if (t.scale != ir::Vec3{1, 1, 1}) {
                Json sc = Json::makeArray();
                for (double d : t.scale) sc.push_back(Json(d));
                n.set("scale", std::move(sc));
            }
            if (node.mesh >= 0) n.set("mesh", Json(node.mesh));
            if (node.skin >= 0) n.set("skin", Json(node.skin));
            if (!node.children.empty()) {
                Json ch = Json::makeArray();
                for (int c : node.children) ch.push_back(Json(c));
                n.set("children", std::move(ch));
            }
            arr.push_back(std::move(n));
        }
        json_.set("nodes", std::move(arr));
    }

    void writeAnimations(const ir::Document& doc) {
        if (doc.animations.empty()) return;
        Json arr = Json::makeArray();
        for (const auto& anim : doc.animations) {
            Json a = Json::makeObject();
            if (!anim.name.empty()) a.set("name", Json(anim.name));
            Json samplers = Json::makeArray();
            for (const auto& s : anim.samplers) {
                Json sampler = Json::makeObject();
                sampler.set("input", Json(addFloatAccessor(s.input, 1, "SCALAR", 0, true)));
                int comps = s.input.empty() ? 0 : int(s.output.size() / s.input.size());
                std::string type = comps == 4 ? "VEC4" : comps == 3 ? "VEC3" : "SCALAR";
                sampler.set("output", Json(addFloatAccessor(s.output, comps, type, 0, false)));
                sampler.set("interpolation",
                            Json(s.interpolation == ir::AnimInterp::Step ? "STEP"
                                 : s.interpolation == ir::AnimInterp::CubicSpline ? "CUBICSPLINE"
                                 : "LINEAR"));
                samplers.push_back(std::move(sampler));
            }
            a.set("samplers", std::move(samplers));
            Json channels = Json::makeArray();
            for (const auto& c : anim.channels) {
                Json channel = Json::makeObject();
                channel.set("sampler", Json(c.sampler));
                Json target = Json::makeObject();
                target.set("node", Json(c.node));
                target.set("path", Json(c.path == ir::AnimPath::Rotation ? "rotation"
                                        : c.path == ir::AnimPath::Scale ? "scale"
                                        : "translation"));
                channel.set("target", std::move(target));
                channels.push_back(std::move(channel));
            }
            a.set("channels", std::move(channels));
            arr.push_back(std::move(a));
        }
        json_.set("animations", std::move(arr));
    }

    void writeScene(const ir::Document& doc) {
        json_.set("scene", Json(0));
        Json scenes = Json::makeArray();
        Json scene = Json::makeObject();
        Json roots = Json::makeArray();
        for (int r : doc.sceneRoots) roots.push_back(Json(r));
        scene.set("nodes", std::move(roots));
        scenes.push_back(std::move(scene));
        json_.set("scenes", std::move(scenes));
    }

    bool writeGltf(const std::string& path, const std::string& binName, std::string& error) {
        std::ofstream out(path, std::ios::binary);
        if (!out) { error = "Cannot open output file: " + path; return false; }
        std::string text = json_.dump(true);
        out.write(text.data(), text.size());
        if (!out) { error = "Failed writing glTF JSON"; return false; }

        std::string binPath = directoryOf(path) + binName;
        std::ofstream bin(binPath, std::ios::binary);
        if (!bin) { error = "Cannot open output buffer: " + binPath; return false; }
        if (!blob_.bytes.empty())
            bin.write(reinterpret_cast<const char*>(blob_.bytes.data()), blob_.bytes.size());
        return static_cast<bool>(bin);
    }

    bool writeGlb(const std::string& path, std::string& error) {
        std::string jsonText = json_.dump(false);
        std::vector<std::uint8_t> jsonChunk(jsonText.begin(), jsonText.end());
        while (jsonChunk.size() % 4 != 0) jsonChunk.push_back(' ');
        std::vector<std::uint8_t> binChunk = blob_.bytes;
        while (binChunk.size() % 4 != 0) binChunk.push_back(0);

        std::uint32_t total = 12 + 8 + std::uint32_t(jsonChunk.size()) + 8 +
                              std::uint32_t(binChunk.size());

        std::ofstream out(path, std::ios::binary);
        if (!out) { error = "Cannot open output file: " + path; return false; }
        auto put32 = [&](std::uint32_t v) { out.write(reinterpret_cast<const char*>(&v), 4); };
        put32(kGlbMagic);
        put32(2);
        put32(total);
        put32(std::uint32_t(jsonChunk.size()));
        put32(kChunkJson);
        out.write(reinterpret_cast<const char*>(jsonChunk.data()), jsonChunk.size());
        put32(std::uint32_t(binChunk.size()));
        put32(kChunkBin);
        out.write(reinterpret_cast<const char*>(binChunk.data()), binChunk.size());
        return static_cast<bool>(out);
    }
};

} // namespace

bool GltfFormat::readFile(const std::string& path, ir::Document& doc, std::string& error) const {
    std::vector<std::uint8_t> file;
    if (!readWholeFile(path, file)) {
        error = "Cannot open file: " + path;
        return false;
    }
    Reader reader(path);
    return reader.read(file, doc, error);
}

bool GltfFormat::writeFile(const std::string& path, const ir::Document& doc, Container container,
                           std::string& error) const {
    Writer writer;
    return writer.write(path, doc, container, error);
}

} // namespace gltfmaya::formats
