#pragma once

#include "ir/GltfIr.h"

#include <maya/MStatus.h>

#include <string>

namespace gltfmaya::scene {

// File-dialog options threaded from the Maya translator down into the import
// and export passes. globalScale/globalRotateDeg are baked into geometry,
// joint transforms, inverse-bind matrices, and animation samplers so the
// resulting scene carries no residual node-level scale or rotation.
struct SceneOptions {
    double globalScale[3] = {1.0, 1.0, 1.0};
    double globalRotateDeg[3] = {0.0, 0.0, 0.0}; // Euler XYZ, degrees
    bool combineMeshes = false;
    bool exportAnimation = false; // honored on export only
};

// Bridges the engine-neutral IR Document and the live Maya scene. The format
// layer never touches Maya types, and this layer never touches files.
class MayaScene {
public:
    MStatus importDocument(const ir::Document& doc, const SceneOptions& opts = {});
    MStatus exportDocument(ir::Document& doc, const SceneOptions& opts, bool exportSelected,
                           std::string& error) const;
};

} // namespace gltfmaya::scene
