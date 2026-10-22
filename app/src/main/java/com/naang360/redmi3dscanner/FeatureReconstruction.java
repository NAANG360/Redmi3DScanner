package com.naang360.redmi3dscanner;

import android.media.Image;

import com.google.ar.core.Frame;
import com.google.ar.core.Pose;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Lightweight, dependency-free sparse photogrammetry for older phones.
 * Tracks FAST-like corners between consecutive camera images and triangulates
 * matched rays using ARCore's world-space camera poses and calibrated intrinsics.
 */
public final class FeatureReconstruction {
    private static final int MAX_FEATURES = 180;
    private static final int PATCH_RADIUS = 3;
    private static final int MIN_BASELINE_METERS = 0.025;
    private static final int MIN_PARALLAX_PIXELS = 2;

    private static final class Feature {
        final int x, y, score;
        Feature(int x, int y, int score) { this.x=x; this.y=y; this.score=score; }
    }

    private static final class View {
        final byte[] gray;
        final int width, height;
        final float fx, fy, cx, cy;
        final float[] origin;
        final Pose pose;
        final List<Feature> features;
        View(byte[] gray, int width, int height, float fx, float fy, float cx, float cy,
             Pose pose, List<Feature> features) {
            this.gray=gray; this.width=width; this.height=height;
            this.fx=fx; this.fy=fy; this.cx=cx; this.cy=cy; this.pose=pose;
            this.origin=pose.getTranslation();
            this.features=features;
        }
    }

    private View previous;
    private long lastProcessMs;
    private int lastMatchCount;

    public synchronized int getLastMatchCount() { return lastMatchCount; }

    public synchronized void reset() {
        previous = null;
        lastProcessMs = 0;
        lastMatchCount = 0;
    }

    /** Returns number of new 3D points accepted into the cloud. */
    public synchronized int process(Frame frame, Image image, PointCloud cloud, long nowMs) {
        if (nowMs - lastProcessMs < 180) return 0;
        lastProcessMs = nowMs;
        if (image == null || image.getFormat() != android.graphics.ImageFormat.YUV_420_888) return 0;

        try {
            int width = image.getWidth(), height = image.getHeight();
            Image.Plane plane = image.getPlanes()[0];
            ByteBuffer ybuf = plane.getBuffer().duplicate();
            int rowStride = plane.getRowStride(), pixelStride = plane.getPixelStride();
            byte[] gray = new byte[width * height];
            for (int y=0; y<height; y++) {
                int row = y * rowStride;
                int dst = y * width;
                for (int x=0; x<width; x++) {
                    int at = row + x * pixelStride;
                    if (at < ybuf.limit()) gray[dst+x] = ybuf.get(at);
                }
            }

            com.google.ar.core.CameraIntrinsics intr = frame.getCamera().getImageIntrinsics();
            float[] focal = intr.getFocalLength();
            float[] principal = intr.getPrincipalPoint();
            int[] dimensions = intr.getImageDimensions();
            float sx = width / (float)Math.max(1, dimensions[0]);
            float sy = height / (float)Math.max(1, dimensions[1]);
            float fx = focal[0] * sx, fy = focal[1] * sy;
            float cx = principal[0] * sx, cy = principal[1] * sy;
            if (fx <= 0 || fy <= 0) return 0;

            List<Feature> features = detectCorners(gray, width, height);
            View current = new View(gray, width, height, fx, fy, cx, cy,
                    frame.getCamera().getPose(), features);
            int added = 0;
            lastMatchCount = 0;

            if (previous != null && previous.width == width && previous.height == height) {
                float[] delta = new float[]{
                        current.origin[0]-previous.origin[0],
                        current.origin[1]-previous.origin[1],
                        current.origin[2]-previous.origin[2]
                };
                float baseline = (float)Math.sqrt(delta[0]*delta[0]+delta[1]*delta[1]+delta[2]*delta[2]);
                if (baseline >= MIN_BASELINE_METERS) {
                    boolean[] used = new boolean[previous.features.size()];
                    for (Feature f : current.features) {
                        int best = -1, bestSad = Integer.MAX_VALUE, secondSad = Integer.MAX_VALUE;
                        for (int j=0; j<previous.features.size(); j++) {
                            if (used[j]) continue;
                            Feature old = previous.features.get(j);
                            if (Math.abs(old.x-f.x) > width/5 || Math.abs(old.y-f.y) > height/5) continue;
                            int sad = patchSad(previous, old, current, f);
                            if (sad < bestSad) { secondSad=bestSad; bestSad=sad; best=j; }
                            else if (sad < secondSad) secondSad=sad;
                        }
                        if (best < 0 || bestSad > 1900 || (secondSad < Integer.MAX_VALUE && bestSad*100 > secondSad*92)) continue;
                        used[best] = true;
                        Feature old = previous.features.get(best);
                        if (Math.abs(old.x-f.x) < MIN_PARALLAX_PIXELS) continue;
                        float[] world = triangulate(previous, old, current, f);
                        if (world == null) continue;
                        int lum = gray[f.y*width+f.x] & 255;
                        if (cloud.add(world[0], world[1], world[2], lum, lum, lum, 0f,0f,0f,0.45f)) added++;
                        lastMatchCount++;
                    }
                }
            }
            previous = current;
            return added;
        } catch (Exception ignored) {
            return 0;
        }
    }

    private static List<Feature> detectCorners(byte[] gray, int w, int h) {
        ArrayList<Feature> found = new ArrayList<>();
        for (int y=8; y<h-8; y+=2) {
            for (int x=8; x<w-8; x+=2) {
                int c=gray[y*w+x]&255;
                int gx=Math.abs((gray[y*w+x+2]&255)-(gray[y*w+x-2]&255));
                int gy=Math.abs((gray[(y+2)*w+x]&255)-(gray[(y-2)*w+x]&255));
                int d1=Math.abs((gray[(y+2)*w+x+2]&255)-(gray[(y-2)*w+x-2]&255));
                int d2=Math.abs((gray[(y+2)*w+x-2]&255)-(gray[(y-2)*w+x+2]&255));
                int score=Math.min(gx,gy)+Math.min(d1,d2);
                if (score < 42) continue;
                if (Math.abs(c-(gray[y*w+x+3]&255))<7 && Math.abs(c-(gray[(y+3)*w+x]&255))<7) continue;
                found.add(new Feature(x,y,score));
            }
        }
        Collections.sort(found, (a,b)->Integer.compare(b.score,a.score));
        ArrayList<Feature> selected = new ArrayList<>();
        int cell = Math.max(18, Math.min(w,h)/18);
        boolean[] occupied = new boolean[((w+cell-1)/cell)*((h+cell-1)/cell)];
        int cols=(w+cell-1)/cell;
        for (Feature f:found) {
            int ix=f.x/cell, iy=f.y/cell, id=iy*cols+ix;
            if (occupied[id]) continue;
            occupied[id]=true;
            selected.add(f);
            if (selected.size()>=MAX_FEATURES) break;
        }
        return selected;
    }

    private static int patchSad(View a, Feature pa, View b, Feature pb) {
        int sad=0;
        for(int dy=-PATCH_RADIUS;dy<=PATCH_RADIUS;dy+=2) {
            for(int dx=-PATCH_RADIUS;dx<=PATCH_RADIUS;dx+=2) {
                int av=a.gray[(pa.y+dy)*a.width+pa.x+dx]&255;
                int bv=b.gray[(pb.y+dy)*b.width+pb.x+dx]&255;
                sad+=Math.abs(av-bv);
            }
        }
        return sad;
    }

    private static float[] triangulate(View a, Feature fa, View b, Feature fb) {
        float[] da=worldRay(a,fa), db=worldRay(b,fb);
        float[] oa=a.origin, ob=b.origin;
        float[] w0={oa[0]-ob[0],oa[1]-ob[1],oa[2]-ob[2]};
        float aa=dot(da,da), bb=dot(da,db), cc=dot(db,db);
        float dd=dot(da,w0), ee=dot(db,w0);
        float denom=aa*cc-bb*bb;
        if (Math.abs(denom)<1e-5f) return null;
        float s=(bb*ee-cc*dd)/denom;
        float t=(aa*ee-bb*dd)/denom;
        if (s<=0.05f || t<=0.05f || s>8f || t>8f) return null;
        float[] p={oa[0]+s*da[0],oa[1]+s*da[1],oa[2]+s*da[2]};
        float[] q={ob[0]+t*db[0],ob[1]+t*db[1],ob[2]+t*db[2]};
        float ex=p[0]-q[0], ey=p[1]-q[1], ez=p[2]-q[2];
        if (ex*ex+ey*ey+ez*ez > 0.04f) return null;
        return new float[]{(p[0]+q[0])*0.5f,(p[1]+q[1])*0.5f,(p[2]+q[2])*0.5f};
    }

    private static float[] worldRay(View v, Feature f) {
        float[] local={(f.x-v.cx)/v.fx, -(f.y-v.cy)/v.fy, -1f};
        float[] endpoint=new float[3];
        v.pose.transformPoint(local,0,endpoint,0);
        float[] ray={endpoint[0]-v.origin[0],endpoint[1]-v.origin[1],endpoint[2]-v.origin[2]};
        float len=(float)Math.sqrt(dot(ray,ray));
        if (len>0) { ray[0]/=len; ray[1]/=len; ray[2]/=len; }
        return ray;
    }

    private static float dot(float[] a,float[] b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
}
