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


    /** Remove degenerate triangles before export; vertices stay stable so face indices remain valid. */
    public synchronized int cleanup() {
        int before = faces.size();
        ArrayList<int[]> cleaned = new ArrayList<>(faces.size());
        HashSet<Long> seen = new HashSet<>();
        for (int[] f : faces) {
            int a=f[0]-1, b=f[1]-1, c=f[2]-1;
            if (a<0 || b<0 || c<0 || a>=vertices.size() || b>=vertices.size() || c>=vertices.size()
                    || a==b || b==c || a==c) continue;
            Vertex va=vertices.get(a), vb=vertices.get(b), vc=vertices.get(c);
            float abx=vb.x-va.x, aby=vb.y-va.y, abz=vb.z-va.z;
            float acx=vc.x-va.x, acy=vc.y-va.y, acz=vc.z-va.z;
            float nx=aby*acz-abz*acy, ny=abz*acx-abx*acz, nz=abx*acy-aby*acx;
            if (nx*nx+ny*ny+nz*nz < 1e-12f) continue;
            int[] ids={f[0],f[1],f[2]}; java.util.Arrays.sort(ids);
            long key=((long)ids[0]<<42)^((long)ids[1]<<21)^ids[2];
            if (seen.add(key)) cleaned.add(f);
        }
        faces.clear(); faces.addAll(cleaned);
        faceKeys.clear();
        for (int[] f:faces) {
            int[] ids={f[0],f[1],f[2]}; java.util.Arrays.sort(ids);
            faceKeys.add(((long)ids[0]<<36)|((long)ids[1]<<18)|ids[2]);
        }
        return before-faces.size();
    }

    /** Writes a binary STL mesh. STL stores geometry only, not vertex colors. */
    public synchronized void writeBinaryStl(File file) throws Exception {
        cleanup();
        try (java.io.DataOutputStream out = new java.io.DataOutputStream(
                new java.io.BufferedOutputStream(new java.io.FileOutputStream(file)))) {
            byte[] header = new byte[80];
            byte[] label = "Redmi3DScanner reconstructed mesh".getBytes(java.nio.charset.StandardCharsets.US_ASCII);
            System.arraycopy(label,0,header,0,Math.min(label.length,header.length));
            out.write(header);
            writeIntLE(out, faces.size());
            for (int[] f:faces) {
                Vertex a=vertices.get(f[0]-1), b=vertices.get(f[1]-1), c=vertices.get(f[2]-1);
                float abx=b.x-a.x, aby=b.y-a.y, abz=b.z-a.z;
                float acx=c.x-a.x, acy=c.y-a.y, acz=c.z-a.z;
                float nx=aby*acz-abz*acy, ny=abz*acx-abx*acz, nz=abx*acy-aby*acx;
                float len=(float)Math.sqrt(nx*nx+ny*ny+nz*nz);
                if(len>1e-9f){nx/=len;ny/=len;nz/=len;}
                writeFloatLE(out,nx);writeFloatLE(out,ny);writeFloatLE(out,nz);
                writeFloatLE(out,a.x);writeFloatLE(out,a.y);writeFloatLE(out,a.z);
                writeFloatLE(out,b.x);writeFloatLE(out,b.y);writeFloatLE(out,b.z);
                writeFloatLE(out,c.x);writeFloatLE(out,c.y);writeFloatLE(out,c.z);
                out.writeByte(0);out.writeByte(0);
            }
        }
    }

    /** Writes a self-contained glTF 2.0 binary (GLB) with per-vertex RGB colors. */
    public synchronized void writeGlb(File file) throws Exception {
        cleanup();
        if(vertices.isEmpty() || faces.isEmpty()) throw new IllegalStateException("Mesh has no triangles");
        int n=vertices.size(), indexCount=faces.size()*3;
        int posOffset=0, posLength=n*12;
        int colorOffset=align4(posOffset+posLength), colorLength=n*4;
        int indexOffset=align4(colorOffset+colorLength), indexLength=indexCount*4;
        int binLength=align4(indexOffset+indexLength);
        java.nio.ByteBuffer bin=java.nio.ByteBuffer.allocate(binLength).order(java.nio.ByteOrder.LITTLE_ENDIAN);
        float minX=Float.POSITIVE_INFINITY,minY=Float.POSITIVE_INFINITY,minZ=Float.POSITIVE_INFINITY;
        float maxX=Float.NEGATIVE_INFINITY,maxY=Float.NEGATIVE_INFINITY,maxZ=Float.NEGATIVE_INFINITY;
        for(Vertex v:vertices) {
            bin.putFloat(v.x).putFloat(v.y).putFloat(v.z);
            minX=Math.min(minX,v.x);minY=Math.min(minY,v.y);minZ=Math.min(minZ,v.z);
            maxX=Math.max(maxX,v.x);maxY=Math.max(maxY,v.y);maxZ=Math.max(maxZ,v.z);
        }
        bin.position(colorOffset);
        for(Vertex v:vertices) bin.put((byte)v.r).put((byte)v.g).put((byte)v.b).put((byte)255);
        bin.position(indexOffset);
        for(int[] f:faces) bin.putInt(f[0]-1).putInt(f[1]-1).putInt(f[2]-1);
        String json="{\"asset\":{\"version\":\"2.0\",\"generator\":\"Redmi3DScanner\"},"
            +"\"scene\":0,\"scenes\":[{\"nodes\":[0]}],\"nodes\":[{\"mesh\":0}],"
            +"\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":0,\"COLOR_0\":1},\"indices\":2,\"mode\":4}]}],"
            +"\"buffers\":[{\"byteLength\":"+binLength+"}],"
            +"\"bufferViews\":[{\"buffer\":0,\"byteOffset\":"+posOffset+",\"byteLength\":"+posLength+",\"target\":34962},"
            +"{\"buffer\":0,\"byteOffset\":"+colorOffset+",\"byteLength\":"+colorLength+",\"target\":34962},"
            +"{\"buffer\":0,\"byteOffset\":"+indexOffset+",\"byteLength\":"+indexLength+",\"target\":34963}],"
            +"\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":"+n+",\"type\":\"VEC3\",\"min\":["+minX+","+minY+","+minZ+"],\"max\":["+maxX+","+maxY+","+maxZ+"]},"
            +"{\"bufferView\":1,\"componentType\":5121,\"normalized\":true,\"count\":"+n+",\"type\":\"VEC4\"},"
            +"{\"bufferView\":2,\"componentType\":5125,\"count\":"+indexCount+",\"type\":\"SCALAR\"}]}";
        byte[] jb=json.getBytes(java.nio.charset.StandardCharsets.UTF_8);
        int jsonLength=align4(jb.length), total=12+8+jsonLength+8+binLength;
        try(java.io.DataOutputStream out=new java.io.DataOutputStream(new java.io.BufferedOutputStream(new java.io.FileOutputStream(file)))) {
            writeIntLE(out,0x46546C67);writeIntLE(out,2);writeIntLE(out,total);
            writeIntLE(out,jsonLength);writeIntLE(out,0x4E4F534A);out.write(jb);
            for(int i=jb.length;i<jsonLength;i++) out.writeByte(0x20);
            writeIntLE(out,binLength);writeIntLE(out,0x004E4942);out.write(bin.array());
        }
    }

    private static int align4(int n){return (n+3)&~3;}
    private static void writeIntLE(java.io.DataOutputStream out,int v) throws java.io.IOException {
        out.writeByte(v&255);out.writeByte((v>>>8)&255);out.writeByte((v>>>16)&255);out.writeByte((v>>>24)&255);
    }
    private static void writeFloatLE(java.io.DataOutputStream out,float v) throws java.io.IOException {
        writeIntLE(out,Float.floatToIntBits(v));
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