package com.naang360.redmi3dscanner;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.media.Image;

import com.google.ar.core.CameraIntrinsics;
import com.google.ar.core.Pose;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.util.Locale;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

/**
 * Stores a compact, calibrated image/pose dataset from a scan. This is the
 * reproducible input needed by an offline SfM/MVS stage; images are not uploaded.
 */
public final class ScanSessionArchive {
    private static final int OUT_W = 640;
    private static final int OUT_H = 360;
    private static final int MAX_FRAMES = 100;
    private static final long MIN_INTERVAL_MS = 500;
    private static final float MIN_TRANSLATION_M = 0.035f;
    private static final float MIN_ROTATION_DEG = 7.0f;

    private final File root;
    private BufferedWriter manifest;
    private int frames;
    private long lastTimestamp;
    private float[] lastPose;

    public ScanSessionArchive(File filesDir) {
        root = new File(filesDir, "scan-session");
        reset();
    }

    public synchronized void reset() {
        closeManifest();
        deleteRecursively(root);
        root.mkdirs();
        frames = 0;
        lastTimestamp = 0;
        lastPose = null;
        try {
            manifest = new BufferedWriter(new FileWriter(new File(root, "manifest.csv")));
            manifest.write("image,timestamp_ms,tx,ty,tz,qx,qy,qz,qw,width,height,fx,fy,cx,cy\n");
            manifest.flush();
        } catch (Exception ignored) {
            manifest = null;
        }
    }

    public synchronized int frameCount() { return frames; }

    /** Saves only useful, spatially distinct keyframes to limit Snapdragon 660 load and storage. */
    public synchronized boolean maybeCapture(Image image, Pose pose, CameraIntrinsics intr, long timestampMs) {
        if (image == null || image.getFormat() != android.graphics.ImageFormat.YUV_420_888
                || frames >= MAX_FRAMES || manifest == null) return false;
        float[] t = pose.getTranslation();
        float[] q = pose.getRotationQuaternion();
        float[] p = {t[0],t[1],t[2],q[0],q[1],q[2],q[3]};
        if (lastPose != null) {
            if (timestampMs-lastTimestamp < MIN_INTERVAL_MS) return false;
            float dx=p[0]-lastPose[0], dy=p[1]-lastPose[1], dz=p[2]-lastPose[2];
            float distance=(float)Math.sqrt(dx*dx+dy*dy+dz*dz);
            float dot=Math.abs(p[3]*lastPose[3]+p[4]*lastPose[4]+p[5]*lastPose[5]+p[6]*lastPose[6]);
            dot=Math.max(-1f,Math.min(1f,dot));
            float angle=(float)Math.toDegrees(2.0*Math.acos(dot));
            if (distance < MIN_TRANSLATION_M && angle < MIN_ROTATION_DEG) return false;
        }

        Bitmap bitmap = null;
        try {
            int width=image.getWidth(), height=image.getHeight();
            Image.Plane[] planes=image.getPlanes();
            Image.Plane yp=planes[0], up=planes[1], vp=planes[2];
            ByteBuffer yb=yp.getBuffer().duplicate(), ub=up.getBuffer().duplicate(), vb=vp.getBuffer().duplicate();
            int[] pixels=new int[OUT_W*OUT_H];
            for(int oy=0;oy<OUT_H;oy++) {
                int y=Math.min(height-1,oy*height/OUT_H);
                for(int ox=0;ox<OUT_W;ox++) {
                    int x=Math.min(width-1,ox*width/OUT_W);
                    int yi=y*yp.getRowStride()+x*yp.getPixelStride();
                    int ui=(y/2)*up.getRowStride()+(x/2)*up.getPixelStride();
                    int vi=(y/2)*vp.getRowStride()+(x/2)*vp.getPixelStride();
                    if(yi>=yb.limit()||ui>=ub.limit()||vi>=vb.limit()) continue;
                    int yy=yb.get(yi)&255, u=(ub.get(ui)&255)-128, v=(vb.get(vi)&255)-128;
                    int r=clamp(yy+(int)(1.402f*v));
                    int g=clamp(yy-(int)(0.344136f*u+0.714136f*v));
                    int b=clamp(yy+(int)(1.772f*u));
                    pixels[oy*OUT_W+ox]=Color.rgb(r,g,b);
                }
            }
            bitmap=Bitmap.createBitmap(pixels,OUT_W,OUT_H,Bitmap.Config.ARGB_8888);
            String name=String.format(Locale.US,"frame-%04d.jpg",frames);
            try(OutputStream out=new FileOutputStream(new File(root,name))) {
                if(!bitmap.compress(Bitmap.CompressFormat.JPEG,86,out)) return false;
            }
            float[] focal=intr.getFocalLength(), principal=intr.getPrincipalPoint();
            int[] dims=intr.getImageDimensions();
            float sx=OUT_W/(float)Math.max(1,dims[0]), sy=OUT_H/(float)Math.max(1,dims[1]);
            manifest.write(String.format(Locale.US,
                    "%s,%d,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%d,%d,%.5f,%.5f,%.5f,%.5f\n",
                    name,timestampMs,p[0],p[1],p[2],p[3],p[4],p[5],p[6],
                    OUT_W,OUT_H,focal[0]*sx,focal[1]*sy,principal[0]*sx,principal[1]*sy));
            manifest.flush();
            frames++;
            lastTimestamp=timestampMs;
            lastPose=p;
            return true;
        } catch(Exception ignored) {
            return false;
        } finally {
            if(bitmap!=null) bitmap.recycle();
        }
    }

    public synchronized void writeZip(File destination) throws Exception {
        if(frames<=0) throw new IllegalStateException("No archived camera keyframes");
        if(manifest!=null) manifest.flush();
        try(ZipOutputStream zip=new ZipOutputStream(new FileOutputStream(destination))) {
            File[] files=root.listFiles();
            if(files==null) throw new IllegalStateException("Scan archive is unavailable");
            byte[] buffer=new byte[32768];
            for(File file:files) {
                if(!file.isFile()) continue;
                zip.putNextEntry(new ZipEntry(file.getName()));
                try(java.io.FileInputStream in=new java.io.FileInputStream(file)) {
                    int n;
                    while((n=in.read(buffer))!=-1) zip.write(buffer,0,n);
                }
                zip.closeEntry();
            }
        }
    }

    public synchronized void close() { closeManifest(); }

    private void closeManifest() {
        if(manifest!=null) {
            try { manifest.flush(); manifest.close(); } catch(Exception ignored) {}
            manifest=null;
        }
    }

    private static void deleteRecursively(File f) {
        if(f==null || !f.exists()) return;
        if(f.isDirectory()) {
            File[] children=f.listFiles();
            if(children!=null) for(File child:children) deleteRecursively(child);
        }
        // Best-effort cleanup of this app's own temporary scan dataset.
        f.delete();
    }

    private static int clamp(int value) { return Math.max(0,Math.min(255,value)); }
}
