#pragma once

#include "formats/GltfFormat.h"

#include <maya/MPxFileTranslator.h>

namespace gltfmaya::translators {

// glTF and GLB share one data model and differ only by container, so both
// translators derive from a single base; subclasses pick the container and the
// file extension they advertise to Maya.
class GltfTranslatorBase : public MPxFileTranslator {
public:
    bool haveReadMethod() const override { return true; }
    bool haveWriteMethod() const override { return true; }
    bool canBeOpened() const override { return true; }

    MStatus reader(const MFileObject& file, const MString& options, FileAccessMode mode) override;
    MStatus writer(const MFileObject& file, const MString& options, FileAccessMode mode) override;
    MFileKind identifyFile(const MFileObject& file, const char* buffer, short size) const override;

protected:
    virtual formats::Container container() const = 0;
    virtual const char* extension() const = 0;
};

class GltfFileTranslator : public GltfTranslatorBase {
public:
    static void* creator() { return new GltfFileTranslator(); }
    MString defaultExtension() const override { return "gltf"; }
    MString filter() const override { return "*.gltf"; }

protected:
    formats::Container container() const override { return formats::Container::Gltf; }
    const char* extension() const override { return "gltf"; }
};

class GlbFileTranslator : public GltfTranslatorBase {
public:
    static void* creator() { return new GlbFileTranslator(); }
    MString defaultExtension() const override { return "glb"; }
    MString filter() const override { return "*.glb"; }

protected:
    formats::Container container() const override { return formats::Container::Glb; }
    const char* extension() const override { return "glb"; }
};

} // namespace gltfmaya::translators
