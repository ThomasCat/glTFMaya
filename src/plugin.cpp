#include <maya/MFnPlugin.h>
#include <maya/MGlobal.h>

#include "translators/GltfFileTranslator.h"

namespace {

const char* kVendor = "Luke Loughlin";
const char* kVersion = "1.2";
const char* kGltfTranslator = "glTF";
const char* kGlbTranslator = "glTF Binary";

void registerExportOptions() {
    // Maya reuses one option script for both reader and writer dialogs, so the
    // panel exposes every option the translator understands. Defaults come
    // from the registered defaultOptionString and may be overridden by the
    // caller's `options=` string when invoking the translator from MEL/Python.
    const char* proc = R"MEL(
global proc int gltfMayaExportOptions(string $parent, string $action,
                                      string $initialSettings, string $resultCallback)
{
    int $exportAnimation = 0;
    int $combineMeshes = 0;
    float $gsx = 1.0; float $gsy = 1.0; float $gsz = 1.0;
    float $grx = 0.0; float $gry = 0.0; float $grz = 0.0;
    string $tokens[]; tokenize($initialSettings, ";", $tokens);
    for ($t in $tokens) {
        string $kv[]; tokenize($t, "=", $kv);
        if (size($kv) < 2) continue;
        if ($kv[0] == "exportAnimation") $exportAnimation = ($kv[1] == "1");
        else if ($kv[0] == "combineMeshes") $combineMeshes = ($kv[1] == "1");
        else if ($kv[0] == "globalScaleX") $gsx = ((float)$kv[1]);
        else if ($kv[0] == "globalScaleY") $gsy = ((float)$kv[1]);
        else if ($kv[0] == "globalScaleZ") $gsz = ((float)$kv[1]);
        else if ($kv[0] == "globalRotateX") $grx = ((float)$kv[1]);
        else if ($kv[0] == "globalRotateY") $gry = ((float)$kv[1]);
        else if ($kv[0] == "globalRotateZ") $grz = ((float)$kv[1]);
    }
    if ($action == "post") {
        setParent $parent;
        columnLayout -adj true;
        checkBoxGrp -ncb 1 -l1 "Export Animation" -v1 $exportAnimation
            -ann "Export: sample joint animation across the playback range. (Export only.)"
            gltfMayaOpt_exportAnimationCB;
        checkBoxGrp -ncb 1 -l1 "Combine Meshes" -v1 $combineMeshes
            -ann "Merge every mesh into a single mesh node. Per-face material assignments are preserved as separate primitives."
            gltfMayaOpt_combineMeshesCB;
        floatFieldGrp -nf 3 -l "Global Scale (XYZ)" -pre 3 -v1 $gsx -v2 $gsy -v3 $gsz
            -ann "Scale baked into vertex positions and joint translations. No node-level scale is left behind."
            gltfMayaOpt_globalScaleFG;
        floatFieldGrp -nf 3 -l "Global Rotate (XYZ, deg)" -pre 3 -v1 $grx -v2 $gry -v3 $grz
            -ann "Euler XYZ rotation (degrees) baked into vertices and joints. Useful for rebasing up-axis at the file boundary."
            gltfMayaOpt_globalRotateFG;
        return 1;
    } else if ($action == "query") {
        int $ea = `checkBoxGrp -q -v1 gltfMayaOpt_exportAnimationCB`;
        int $cm = `checkBoxGrp -q -v1 gltfMayaOpt_combineMeshesCB`;
        float $sx = `floatFieldGrp -q -v1 gltfMayaOpt_globalScaleFG`;
        float $sy = `floatFieldGrp -q -v2 gltfMayaOpt_globalScaleFG`;
        float $sz = `floatFieldGrp -q -v3 gltfMayaOpt_globalScaleFG`;
        float $rx = `floatFieldGrp -q -v1 gltfMayaOpt_globalRotateFG`;
        float $ry = `floatFieldGrp -q -v2 gltfMayaOpt_globalRotateFG`;
        float $rz = `floatFieldGrp -q -v3 gltfMayaOpt_globalRotateFG`;
        string $opt = "exportAnimation=" + $ea + ";"
                    + "combineMeshes=" + $cm + ";"
                    + "globalScaleX=" + $sx + ";"
                    + "globalScaleY=" + $sy + ";"
                    + "globalScaleZ=" + $sz + ";"
                    + "globalRotateX=" + $rx + ";"
                    + "globalRotateY=" + $ry + ";"
                    + "globalRotateZ=" + $rz + ";";
        eval($resultCallback + " \"" + $opt + "\"");
        return 1;
    }
    return 0;
}
)MEL";
    MGlobal::executeCommand(proc);
}

} // namespace

MStatus initializePlugin(MObject obj) {
    MFnPlugin plugin(obj, kVendor, kVersion, "Any");
    registerExportOptions();

    MStatus status = plugin.registerFileTranslator(
        kGltfTranslator, "", gltfmaya::translators::GltfFileTranslator::creator,
        "gltfMayaExportOptions",
        "exportAnimation=0;combineMeshes=0;globalScaleX=1;globalScaleY=1;globalScaleZ=1;"
        "globalRotateX=0;globalRotateY=0;globalRotateZ=0;",
        true);
    if (!status) status.perror("[GLTFMaya] registerFileTranslator glTF");

    status = plugin.registerFileTranslator(
        kGlbTranslator, "", gltfmaya::translators::GlbFileTranslator::creator,
        "gltfMayaExportOptions",
        "exportAnimation=0;combineMeshes=0;globalScaleX=1;globalScaleY=1;globalScaleZ=1;"
        "globalRotateX=0;globalRotateY=0;globalRotateZ=0;",
        true);
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
