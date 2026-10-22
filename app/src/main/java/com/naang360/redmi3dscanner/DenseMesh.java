package com.naang360.redmi3dscanner;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Locale;

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
    private final ArrayList<Vertex> vertices = new ArrayList<>();
    private final ArrayList<int[]> faces = new ArrayList<>();

    public synchronized void clear() { vertices.clear(); faces.clear(); }
    public synchronized int vertexCount() { return vertices.size(); }
    public synchronized int faceCount() { return faces.size(); }

    public synchronized int addVertex(float x,float y,float z,int r,int g,int b) {
        if(vertices.size()>=MAX_VERTICES || !Float.isFinite(x) || !Float.isFinite(y) || !Float.isFinite(z)) return -1;
        vertices.add(new Vertex(x,y,z,clamp(r),clamp(g),clamp(b)));
        return vertices.size()-1;
    }

    public synchronized void addTriangle(int a,int b,int c,float maxEdgeMeters) {
        if(faces.size()>=MAX_FACES || a<0 || b<0 || c<0 || a==b || b==c || a==c
                || a>=vertices.size() || b>=vertices.size() || c>=vertices.size()) return;
        Vertex va=vertices.get(a), vb=vertices.get(b), vc=vertices.get(c);
        float limit2=maxEdgeMeters*maxEdgeMeters;
        if(distance2(va,vb)>limit2 || distance2(vb,vc)>limit2 || distance2(vc,va)>limit2) return;
        faces.add(new int[]{a+1,b+1,c+1});
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

    private static float distance2(Vertex a,Vertex b) {
        float x=a.x-b.x,y=a.y-b.y,z=a.z-b.z; return x*x+y*y+z*z;
    }
    private static int clamp(int v) { return Math.max(0,Math.min(255,v)); }
}