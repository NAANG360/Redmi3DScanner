package com.naang360.redmi3dscanner;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Locale;
import java.util.HashMap;
import java.util.HashSet;

/**
 * Depth-grid mesh fusion. Vertices are sampled from ARCore's metric depth map;
 * faces are created only between neighboring samples with small depth jumps.
 */
public final class DenseMesh {
    private static final int MAX_VERTICES = 180000;
    private static final int MAX_FACES = 320000;
    private static final class Vertex {
        final float x,y,z; final int r,g,b;
        Vertex(float x,float y,float z,int r,int g,int b) {
            this.x=x;this.y=y;this.z=z;this.r=r;this.g=g;this.b=b;
        }
    }
    private static final float VOXEL_METERS = 0.012f;
    private final ArrayList<Vertex> vertices = new ArrayList<>();
    private final ArrayList<int[]> faces = new ArrayList<>();
    private final HashMap<Long,Integer> voxelIndices = new HashMap<>();
    private final HashSet<Long> faceKeys = new HashSet<>();

    public synchronized void clear() { vertices.clear(); faces.clear(); voxelIndices.clear(); faceKeys.clear(); }
    public synchronized int vertexCount() { return vertices.size(); }
    public synchronized int faceCount() { return faces.size(); }

    public synchronized int addVertex(float x,float y,float z,int r,int g,int b) {
        if(!Float.isFinite(x) || !Float.isFinite(y) || !Float.isFinite(z)) return -1;
        long key=voxelKey(x,y,z);
        Integer existing=voxelIndices.get(key);
        if(existing!=null) return existing;
        if(vertices.size()>=MAX_VERTICES) return -1;
        int index=vertices.size();
        vertices.add(new Vertex(x,y,z,clamp(r),clamp(g),clamp(b)));
        voxelIndices.put(key,index);
        return index;
    }

    public synchronized void addTriangle(int a,int b,int c,float maxEdgeMeters) {
        if(faces.size()>=MAX_FACES || a<0 || b<0 || c<0 || a==b || b==c || a==c
                || a>=vertices.size() || b>=vertices.size() || c>=vertices.size()) return;
        Vertex va=vertices.get(a), vb=vertices.get(b), vc=vertices.get(c);
        float limit2=maxEdgeMeters*maxEdgeMeters;
        if(distance2(va,vb)>limit2 || distance2(vb,vc)>limit2 || distance2(vc,va)>limit2) return;
        int x=a+1,y=b+1,z=c+1; int[] ids={x,y,z}; java.util.Arrays.sort(ids);
        long faceKey=((long)ids[0]<<36)|((long)ids[1]<<18)|ids[2];
        if(faceKeys.add(faceKey)) faces.add(new int[]{x,y,z});
    }

    public synchronized void writeObj(File file) throws Exception {
        try(BufferedWriter w=new BufferedWriter(new FileWriter(file))) {
            w.write("# Redmi3DScanner dense depth-grid mesh\n");
            w.write("# Vertices="+vertices.size()+" faces="+faces.size()+"\n");
            for(Vertex v:vertices) w.write(String.format(Locale.US,"v %.6f %.6f %.6f %.4f %.4f %.4f\n",
                    v.x,v.y,v.z,v.r/255f,v.g/255f,v.b/255f));
            for(int[] f:faces) w.write("f "+f[0]+" "+f[1]+" "+f[2]+"\n");
        }
    }

    private static long voxelKey(float x,float y,float z) {
        long ix=Math.round(x/VOXEL_METERS), iy=Math.round(y/VOXEL_METERS), iz=Math.round(z/VOXEL_METERS);
        return ((ix&0x1FFFFFL)<<42)|((iy&0x1FFFFFL)<<21)|(iz&0x1FFFFFL);
    }
    private static float distance2(Vertex a,Vertex b) {
        float x=a.x-b.x,y=a.y-b.y,z=a.z-b.z; return x*x+y*y+z*z;
    }
    private static int clamp(int v) { return Math.max(0,Math.min(255,v)); }
}