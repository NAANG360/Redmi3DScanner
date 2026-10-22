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

    /** Writes a colored OBJ point cloud with conservative local triangle guesses.
     * Faces are emitted only when nearby samples form a compact triangle; sparse
     * monocular scans may legitimately produce vertices without faces. */
    public synchronized void writeObj(File file) throws Exception {
        final int limit = 7000;
        ArrayList<P> mesh = new ArrayList<>();
        int stride = Math.max(1, (int)Math.ceil(points.size() / (double)limit));
        for (int i=0; i<points.size(); i+=stride) mesh.add(points.get(i));
        try (BufferedWriter w = new BufferedWriter(new FileWriter(file))) {
            w.write("# Redmi3DScanner reconstructed point cloud / conservative surface estimate\n");
            for (P p : mesh) w.write(String.format(Locale.US, "v %.7f %.7f %.7f %.4f %.4f %.4f\n",
                    p.x,p.y,p.z,p.r/255f,p.g/255f,p.b/255f));
            final float cell=0.05f;
            java.util.HashMap<String, ArrayList<Integer>> buckets = new java.util.HashMap<>();
            for(int i=0;i<mesh.size();i++) {
                P p=mesh.get(i); String k=bucket(p.x,p.y,p.z,cell);
                buckets.computeIfAbsent(k, key -> new ArrayList<>()).add(i);
            }
            java.util.HashSet<String> faces = new java.util.HashSet<>();
            for(int i=0;i<mesh.size();i++) {
                P a=mesh.get(i);
                ArrayList<Integer> near=new ArrayList<>();
                int bx=(int)Math.floor(a.x/cell), by=(int)Math.floor(a.y/cell), bz=(int)Math.floor(a.z/cell);
                for(int dx=-1;dx<=1;dx++) for(int dy=-1;dy<=1;dy++) for(int dz=-1;dz<=1;dz++) {
                    ArrayList<Integer> bucket=buckets.get((bx+dx)+":"+ (by+dy)+":"+ (bz+dz));
                    if(bucket!=null) for(int j:bucket) if(j!=i && distance2(a,mesh.get(j))<0.0025f) near.add(j);
                }
                near.sort((u,v)->Float.compare(distance2(a,mesh.get(u)),distance2(a,mesh.get(v))));
                if(near.size()>8) near.subList(8,near.size()).clear();
                for(int u=0;u<near.size();u++) for(int v=u+1;v<near.size();v++) {
                    int b=near.get(u), c=near.get(v);
                    if(distance2(mesh.get(b),mesh.get(c))>=0.0025f) continue;
                    int x=i+1,y=b+1,z=c+1;
                    int[] ids={x,y,z}; java.util.Arrays.sort(ids);
                    String key=ids[0]+":"+ids[1]+":"+ids[2];
                    if(faces.add(key)) w.write("f "+x+" "+y+" "+z+"\n");
                }
            }
            w.flush();
        }
    }

    private static String bucket(float x,float y,float z,float cell) {
        return ((int)Math.floor(x/cell))+":"+((int)Math.floor(y/cell))+":"+((int)Math.floor(z/cell));
    }
    private static float distance2(P a,P b) {
        float x=a.x-b.x,y=a.y-b.y,z=a.z-b.z;
        return x*x+y*y+z*z;
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