#pragma once

#include "ir/GltfIr.h"

#include <string>

namespace gltfmaya::formats {

enum class Container { Gltf, Glb };

class GltfFormat {
public:
    // Reads either a .gltf (JSON + external .bin) or a .glb (binary). The
    // container is detected from the file contents, so the extension only
    // matters for locating a sidecar .bin.
    bool readFile(const std::string& path, ir::Document& doc, std::string& error) const;

    bool writeFile(const std::string& path, const ir::Document& doc, Container container,
                   std::string& error) const;
};

} // namespace gltfmaya::formats
