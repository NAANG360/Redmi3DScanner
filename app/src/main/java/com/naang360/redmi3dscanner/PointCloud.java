package com.naang360.redmi3dscanner;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;

/** Thread-safe colored point cloud with lightweight voxel thinning. */
public final class PointCloud {
    public static final class P {
        public final float x, y, z;
        public final int r, g, b;
        public final float nx, ny, nz;
        public final float confidence;

        P(float x, float y, float z, int r, int g, int b,
          float nx, float ny, float nz, float confidence) {
            this.x=x; this.y=y; this.z=z;
            this.r=r; this.g=g; this.b=b;
            this.nx=nx; this.ny=ny; this.nz=nz;
            this.confidence=confidence;
        }
    }

    private final List<P> points = new ArrayList<>();
    private final Set<Long> occupiedVoxels = new HashSet<>();
    private final int maxPoints;
    private volatile float voxelSizeMeters = 0.008f;

    public PointCloud(int maxPoints) { this.maxPoints = maxPoints; }

    public synchronized boolean add(float x, float y, float z, int r, int g, int b,
                                    float nx, float ny, float nz, float confidence) {
        if (points.size() >= maxPoints || !Float.isFinite(x) || !Float.isFinite(y) || !Float.isFinite(z)) return false;
        long key = voxelKey(x, y, z, voxelSizeMeters);
        if (!occupiedVoxels.add(key)) return false;
        points.add(new P(x, y, z, clamp(r), clamp(g), clamp(b), nx, ny, nz,
                Math.max(0f, Math.min(1f, confidence))));
        return true;
    }

    public boolean add(float x, float y, float z, int r, int g, int b, float confidence) {
        return add(x, y, z, r, g, b, 0f, 0f, 0f, confidence);
    }

    public boolean add(float x, float y, float z) {
        return add(x, y, z, 255, 255, 255, 0f, 0f, 0f, 1f);
    }

    public synchronized List<P> snapshot() { return new ArrayList<>(points); }
    public synchronized int size() { return points.size(); }
    public synchronized void clear() { points.clear(); occupiedVoxels.clear(); }
    public void setVoxelSizeMeters(float meters) { voxelSizeMeters = Math.max(0.001f, meters); }
    public float getVoxelSizeMeters() { return voxelSizeMeters; }

    public synchronized void writePly(File file) throws Exception {
        try (BufferedWriter w = new BufferedWriter(new FileWriter(file))) {
            w.write("ply\nformat ascii 1.0\ncomment Redmi3DScanner colored reconstruction\n");
            w.write("element vertex " + points.size() + "\n");
            w.write("property float x\nproperty float y\nproperty float z\n");
            w.write("property uchar red\nproperty uchar green\nproperty uchar blue\n");
            w.write("property float nx\nproperty float ny\nproperty float nz\n");
            w.write("property float confidence\nend_header\n");
            for (P p : points) {
                w.write(String.format(Locale.US,
                        "%.7f %.7f %.7f %d %d %d %.6f %.6f %.6f %.5f\n",
                        p.x,p.y,p.z,p.r,p.g,p.b,p.nx,p.ny,p.nz,p.confidence));
            }
        }
    }

    private static int clamp(int v) { return Math.max(0, Math.min(255, v)); }

    private static long voxelKey(float x, float y, float z, float size) {
        long ix = Math.round(x / size);
        long iy = Math.round(y / size);
        long iz = Math.round(z / size);
        long a = (ix & 0x1FFFFFL);
        long b = (iy & 0x1FFFFFL);
        long c = (iz & 0x1FFFFFL);
        return (a << 42) ^ (b << 21) ^ c;
    }
}