# Third-party components

- **ARCore SDK for Android 1.49.0** — Google's ARCore SDK is linked as an application dependency. On supported Android devices, ARCore runtime services may still need to be installed/updated separately through Google Play Services for AR; this system runtime is not redistributed inside this APK.
- **OpenCV Android 4.10.0** — bundled through the Maven artifact `org.opencv:opencv:4.10.0`, including ARM64 native libraries. OpenCV is licensed under Apache License 2.0.
- **MiDaS v2.1 small ONNX model** — fetched at build time from the official Intel ISL MiDaS release (`https://github.com/isl-org/MiDaS/releases/tag/v2_1`) and packaged as `assets/midas_small.onnx`. The MiDaS project/model is distributed under its upstream license; consult the upstream repository and release materials for the exact terms before redistribution.
- **DenseMesh** — project code that triangulates neighboring samples from ARCore metric depth maps, rejecting large depth discontinuities. It is a lightweight depth-grid mesher, not a full COLMAP/OpenMVS-grade multi-view stereo pipeline.
- **MonocularDepthModel** — project integration for OpenCV DNN inference using MiDaS-small when ARCore depth is unavailable. Monocular output is relative depth and the fallback scale is heuristic, not measurement-grade.
