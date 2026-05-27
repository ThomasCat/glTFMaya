#include <maya/MFnPlugin.h>
#include <maya/MGlobal.h>

#include "translators/GltfFileTranslator.h"

namespace {

const char* kVendor = "Luke Loughlin";
const char* kVersion = "1.0";
const char* kGltfTranslator = "glTF";
const char* kGlbTranslator = "glTF Binary";

void registerExportOptions() {
    MString proc =
        "global proc int gltfMayaExportOptions(string $parent, string $action, "
        "string $initialSettings, string $resultCallback)\n"
        "{\n"
        "    int $exportAnimation = 0;\n"
        "    string $tokens[]; tokenize($initialSettings, \";\", $tokens);\n"
        "    for ($t in $tokens) {\n"
        "        string $kv[]; tokenize($t, \"=\", $kv);\n"
        "        if (size($kv) >= 2 && $kv[0] == \"exportAnimation\") $exportAnimation = ($kv[1] == \"1\");\n"
        "    }\n"
        "    if ($action == \"post\") {\n"
        "        setParent $parent;\n"
        "        columnLayout -adj true;\n"
        "        checkBoxGrp -ncb 1 -l1 \"Export Animation\" -v1 $exportAnimation "
        "            -ann \"Sample joint animation across the playback range.\" "
        "            gltfMayaOpt_exportAnimationCB;\n"
        "        return 1;\n"
        "    } else if ($action == \"query\") {\n"
        "        int $v = `checkBoxGrp -q -v1 gltfMayaOpt_exportAnimationCB`;\n"
        "        eval($resultCallback + \" \\\"exportAnimation=\" + $v + \";\\\"\");\n"
        "        return 1;\n"
        "    }\n"
        "    return 0;\n"
        "}";
    MGlobal::executeCommand(proc);
}

} // namespace

MStatus initializePlugin(MObject obj) {
    MFnPlugin plugin(obj, kVendor, kVersion, "Any");
    registerExportOptions();

    MStatus status = plugin.registerFileTranslator(
        kGltfTranslator, "", gltfmaya::translators::GltfFileTranslator::creator,
        "gltfMayaExportOptions", "exportAnimation=0;", true);
    if (!status) status.perror("[GLTFMaya] registerFileTranslator glTF");

    status = plugin.registerFileTranslator(
        kGlbTranslator, "", gltfmaya::translators::GlbFileTranslator::creator,
        "gltfMayaExportOptions", "exportAnimation=0;", true);
    if (!status) status.perror("[GLTFMaya] registerFileTranslator GLB");

    if (status) MGlobal::displayInfo("[GLTFMaya] loaded.");
    return status;
}

MStatus uninitializePlugin(MObject obj) {
    MFnPlugin plugin(obj);
    MStatus status = plugin.deregisterFileTranslator(kGlbTranslator);
    status = plugin.deregisterFileTranslator(kGltfTranslator);
    if (!status) status.perror("[GLTFMaya] deregisterFileTranslator");
    return status;
}
