#include "translators/GltfFileTranslator.h"

#include "scene/MayaScene.h"

#include <maya/MFileObject.h>
#include <maya/MGlobal.h>

#include <algorithm>
#include <cstdlib>
#include <string>

namespace gltfmaya::translators {
namespace {

std::string lowerExt(const MString& name) {
    std::string s = name.asChar() ? name.asChar() : "";
    size_t dot = s.find_last_of('.');
    if (dot == std::string::npos) return {};
    std::string ext = s.substr(dot + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return ext;
}

// Maya concatenates the translator's default option string with the caller's,
// so honor the last occurrence of a key to let overrides win.
std::string findOptionValue(const std::string& s, const char* key) {
    std::string needle = std::string(key) + "=";
    size_t p = s.rfind(needle);
    if (p == std::string::npos) return {};
    p += needle.size();
    size_t end = s.find_first_of(";", p);
    if (end == std::string::npos) end = s.size();
    return s.substr(p, end - p);
}

bool optionBool(const MString& options, const char* key, bool fallback) {
    std::string s = options.asChar() ? options.asChar() : "";
    std::string v = findOptionValue(s, key);
    if (v.empty()) return fallback;
    return v[0] == '1';
}

double optionFloat(const MString& options, const char* key, double fallback) {
    std::string s = options.asChar() ? options.asChar() : "";
    std::string v = findOptionValue(s, key);
    if (v.empty()) return fallback;
    try { return std::stod(v); }
    catch (...) { return fallback; }
}

scene::SceneOptions buildOptions(const MString& options) {
    scene::SceneOptions opts;
    opts.exportAnimation = optionBool(options, "exportAnimation", false);
    opts.combineMeshes = optionBool(options, "combineMeshes", false);
    opts.globalScale[0] = optionFloat(options, "globalScaleX", 1.0);
    opts.globalScale[1] = optionFloat(options, "globalScaleY", 1.0);
    opts.globalScale[2] = optionFloat(options, "globalScaleZ", 1.0);
    opts.globalRotateDeg[0] = optionFloat(options, "globalRotateX", 0.0);
    opts.globalRotateDeg[1] = optionFloat(options, "globalRotateY", 0.0);
    opts.globalRotateDeg[2] = optionFloat(options, "globalRotateZ", 0.0);
    return opts;
}

} // namespace

MStatus GltfTranslatorBase::reader(const MFileObject& file, const MString& options,
                                   FileAccessMode) {
    formats::GltfFormat fmt;
    ir::Document doc;
    std::string error;
    if (!fmt.readFile(file.resolvedFullName().asChar(), doc, error)) {
        MGlobal::displayError(MString("[GLTFMaya] ") + error.c_str());
        return MS::kFailure;
    }
    scene::MayaScene maya;
    return maya.importDocument(doc, buildOptions(options));
}

MStatus GltfTranslatorBase::writer(const MFileObject& file, const MString& options,
                                   FileAccessMode mode) {
    const scene::SceneOptions opts = buildOptions(options);
    const bool exportSelected = (mode == kExportActiveAccessMode);

    scene::MayaScene maya;
    ir::Document doc;
    std::string error;
    if (maya.exportDocument(doc, opts, exportSelected, error) != MS::kSuccess) {
        MGlobal::displayError(MString("[GLTFMaya] ") + error.c_str());
        return MS::kFailure;
    }
    formats::GltfFormat fmt;
    if (!fmt.writeFile(file.resolvedFullName().asChar(), doc, container(), error)) {
        MGlobal::displayError(MString("[GLTFMaya] ") + error.c_str());
        return MS::kFailure;
    }
    MGlobal::displayInfo(MString("[GLTFMaya] Exported ") + file.resolvedFullName());
    return MS::kSuccess;
}

MPxFileTranslator::MFileKind GltfTranslatorBase::identifyFile(const MFileObject& file, const char*,
                                                              short) const {
    return lowerExt(file.resolvedName()) == extension() ? kIsMyFileType : kNotMyFileType;
}

} // namespace gltfmaya::translators
