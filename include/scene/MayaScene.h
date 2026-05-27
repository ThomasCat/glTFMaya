#pragma once

#include "ir/GltfIr.h"

#include <maya/MStatus.h>

#include <string>

namespace gltfmaya::scene {

// Bridges the engine-neutral IR Document and the live Maya scene. The format
// layer never touches Maya types, and this layer never touches files.
class MayaScene {
public:
    MStatus importDocument(const ir::Document& doc);
    MStatus exportDocument(ir::Document& doc, bool exportAnimation, std::string& error) const;
};

} // namespace gltfmaya::scene
