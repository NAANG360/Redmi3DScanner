# Redmi 3D Scanner

> **Turn a normal Redmi into a pocket 3D scanner.**
>
> Camera motion + inertial sensing + open-source AR tooling → a locally reconstructed point cloud and, eventually, a real mesh.

![Status](https://img.shields.io/badge/status-early%20development-orange) ![Platform](https://img.shields.io/badge/platform-Android%209%2B-green) ![Target](https://img.shields.io/badge/target-Redmi%20Note%207S%20%2F%20lavender-blue) ![License](https://img.shields.io/badge/license-Apache--2.0-lightgrey)

## What this is

Redmi 3D Scanner is an experimental sensor-first Android 3D scanning app built around the hardware already inside the phone.

It combines camera tracking, inertial sensors, ARCore where available, local reconstruction, optional root diagnostics, and standard 3D export formats. It does **not** pretend the Redmi Note 7S has LiDAR.

## Hardware reality

The IR blaster is an IR transmitter for remote-control use, not a LiDAR/ToF depth transceiver. The scanner therefore does not misuse it as a fake depth sensor. Camera imagery and motion sensing are the primary inputs.

## Reconstruction pipeline (experimental)

The Android app bundles the ARCore SDK, OpenCV 4.10 ARM64 native libraries, and a MiDaS v2.1-small ONNX monocular-depth model. GitHub Actions downloads the MIT-licensed model weights and packages them into the APK.

- **ARCore automatic depth available:** camera tracking + metric depth samples are transformed into world space; adjacent depth-grid samples are triangulated into a colored OBJ surface.
- **ARCore depth unavailable:** OpenCV DNN runs the bundled MiDaS-small model periodically and generates an approximate relative-depth surface. Its depth-to-metre scale is heuristic and is not suitable for measurement.
- **Sparse feature reconstruction:** ORB descriptors are matched between camera frames and triangulated with tracked camera poses.
- **Exports:** PLY point cloud, OBJ surface/mesh, and keyframe CSV in `Download/Redmi3DScanner`.

This is a bundled, on-device **depth-grid surface reconstruction pipeline**, not a full COLMAP/OpenMVS-grade multi-view stereo implementation. The OBJ mesh can contain holes, overlap, and geometric errors; the monocular fallback is less reliable than ARCore's metric depth.

The ARCore Android SDK is packaged with the app, but Google Play Services for AR may still need to be installed or updated separately on compatible devices. Google does not provide its system runtime as a redistributable app dependency.

## Current status

- [x] ARCore SDK integration and camera tracking
- [x] OpenCV 4.10 ARM64 native runtime
- [x] ORB feature detection, descriptor matching, and sparse triangulation
- [x] ARCore automatic depth capture when supported
- [x] MiDaS-small ONNX model bundled for relative-depth fallback
- [x] Colored depth-grid mesh generation and OBJ export
- [x] PLY point cloud and keyframe CSV export
- [x] GitHub Actions APK build
- [ ] On-device performance and quality validation on the target Redmi
- [ ] Robust multi-view bundle adjustment and dense stereo fusion
- [ ] Texture atlas generation and watertight mesh cleanup
- [ ] GLB/STL export

## Open-source foundations

- Google ARCore Android SDK — https://github.com/google-ar/arcore-android-sdk
- Google Raw Depth Codelab — https://github.com/google-ar/codelab-raw-depth-api
- ARCore samples — https://github.com/google-ar/arcore-android-sdk/tree/main/samples
- DenseFrame — https://github.com/RandoTeam/denseframe

Upstream licenses remain with their respective projects.

## Build

GitHub Actions builds a debug APK on pushes to `main`, pull requests, and manual workflow runs. The artifact is named `Redmi3DScanner-debug`.

Local build:

```bash
gradle wrapper --gradle-version 8.9
./gradlew assembleDebug
```

## Root mode

Root is optional. When available, the app can inspect privileged device interfaces for diagnostics. Root does not create missing depth hardware and is never required for ordinary scanning.

## Status

**Experimental.** The current build is the capture foundation; the next major milestone is proper camera-intrinsics/world-space fusion followed by real-time point-cloud rendering and mesh reconstruction.

## License

Apache License 2.0.
