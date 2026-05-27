# GLTFMaya

A Maya plug-in for importing and exporting **glTF 2.0** (`.gltf`) and **glTF Binary** (`.glb`) models and animations.

Supports Autodesk Maya 2019 through 2027 on Windows (x64).

## Features

- Import glTF and GLB models and animations into Maya
- Export glTF and GLB models and animations from Maya

## Installation

1. Go to the [Releases page](https://github.com/ThomasCat/glTFMaya/releases) and download the zip that matches your Maya version (e.g. `GLTFMaya-Maya2024-Release.zip`).
2. Extract the zip. Inside you will find `GLTFMaya.mll`.
3. Copy `GLTFMaya.mll` into Maya's plug-in folder:
   - `C:\Program Files\Autodesk\Maya<version>\bin\plug-ins\`

   Replace `<version>` with your Maya version, e.g. `2024`.

## Loading the plug-in

1. Start Maya.
2. Open **Windows -> Settings/Preferences -> Plug-in Manager**.
3. Find **GLTFMaya.mll** in the list.
4. Tick **Loaded** to load it for this session. Tick **Auto load** if you want it to load every time Maya starts.

If the plug-in loaded correctly you will see `[GLTFMaya] loaded.` in Maya's script editor output.

## Importing a model or animation

1. Go to **File -> Import...**
2. In the **Files of type** dropdown, choose **glTF** or **glTF Binary**.
3. Select your `.gltf` or `.glb` file and click **Import**.

Or.. just drag it into your scene.

## Exporting a model or animation

1. In the viewport or Outliner, select the objects you want to export.
2. Go to **File -> Export Selection...**
3. In the **Files of type** dropdown, choose **glTF** or **glTF Binary**.
4. In the export options on the right side of the dialog:
   - Leave **Export Animation** unticked to export the selected model geometry.
   - Tick **Export Animation** to sample the joint animation across the playback range.
5. Choose a filename and click **Export Selection**.

## Supported Maya versions

Builds are published for Maya **2019, 2020, 2022, 2023, 2024, 2025, 2026, and 2027**. Make sure you download the zip that matches the version of Maya you are running. A plug-in built for one Maya version will not load in another.

## Troubleshooting

- **The plug-in does not appear in the Plug-in Manager.** Double-check that `GLTFMaya.mll` is in the plug-in folder listed above, then click **Refresh** in the Plug-in Manager.
- **Maya says the plug-in failed to load.** You are most likely using a build for a different Maya version. Download the zip that matches your Maya version.
- **My format is not in the Import/Export dropdown.** The plug-in is not loaded. Open the Plug-in Manager and tick **Loaded** next to `GLTFMaya.mll`.
- **My problem isn't listed here!** Create an issue on the [issues page](https://github.com/ThomasCat/glTFMaya/issues) and I will get around to it.

## Building from source

If you want to build the plug-in yourself instead of using a release:

1. Install Visual Studio 2022 with the **Desktop development with C++** workload.
2. Download the Autodesk Maya devkit for your Maya version and extract it somewhere on disk.
3. Open `GLTFMaya.sln` in Visual Studio.
4. Set the `MAYA_ROOT` MSBuild property to the path of the extracted devkit (the folder that contains `include\` and `lib\`).
5. Build the **Release / x64** configuration. The output is `bin\Release\GLTFMaya.mll`.

## License

See the [LICENSE](LICENSE) file for license information.
