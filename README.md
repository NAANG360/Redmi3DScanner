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

## Pipeline

```text
Camera ──────┐
             ├──► tracking / reconstruction ──► point cloud ──► mesh ──► PLY/OBJ/GLB/STL
IMU ─────────┘
```

## Roadmap

- [x] Android project
- [x] Camera permission flow
- [x] ARCore integration
- [x] IMU capability detection
- [x] Optional root detection
- [x] Initial point-cloud capture
- [x] PLY export
- [x] GitHub Actions APK build
- [ ] Calibrated camera intrinsics
- [ ] Pose-correct world-space fusion
- [ ] Confidence filtering and voxel downsampling
- [ ] Live point-cloud renderer
- [ ] Scan coverage UI
- [ ] TSDF / marching-cubes mesh reconstruction
- [ ] OBJ / GLB / STL export

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
