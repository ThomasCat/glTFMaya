#include "translators/GltfFileTranslator.h"

#include "scene/MayaScene.h"

#include <maya/MFileObject.h>
#include <maya/MGlobal.h>

#include <algorithm>
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
bool optionBool(const MString& options, const char* key, bool fallback) {
    std::string s = options.asChar() ? options.asChar() : "";
    std::string needle = std::string(key) + "=";
    size_t p = s.rfind(needle);
    if (p == std::string::npos) return fallback;
    p += needle.size();
    return p < s.size() && s[p] == '1';
}

} // namespace

MStatus GltfTranslatorBase::reader(const MFileObject& file, const MString&, FileAccessMode) {
    formats::GltfFormat fmt;
    ir::Document doc;
    std::string error;
    if (!fmt.readFile(file.resolvedFullName().asChar(), doc, error)) {
        MGlobal::displayError(MString("[GLTFMaya] ") + error.c_str());
        return MS::kFailure;
    }
    scene::MayaScene maya;
    return maya.importDocument(doc);
}

MStatus GltfTranslatorBase::writer(const MFileObject& file, const MString& options,
                                   FileAccessMode mode) {
    const bool exportAnimation = optionBool(options, "exportAnimation", false);
    const bool exportSelected = (mode == kExportActiveAccessMode);
    (void)exportSelected;

    scene::MayaScene maya;
    ir::Document doc;
    std::string error;
    if (maya.exportDocument(doc, exportAnimation, error) != MS::kSuccess) {
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
