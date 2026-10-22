package com.naang360.redmi3dscanner;

import android.media.Image;
import com.google.ar.core.Frame;
import com.google.ar.core.Pose;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.DMatch;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.calib3d.Calib3d;
import org.opencv.features2d.BFMatcher;
import org.opencv.features2d.ORB;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

/**
 * ORB feature-based sparse reconstruction. Features are matched by binary
 * descriptors, then triangulated using ARCore's tracked camera poses.
 * This is a sparse SfM component, not a dense MVS or bundle-adjustment engine.
 */
public final class FeatureReconstruction {
    private static final int MAX_FEATURES = 650;
    private static final float MIN_BASELINE_METERS = 0.025f;
    private static final float MAX_DESCRIPTOR_DISTANCE = 58f;
    private static final float RATIO_TEST = 0.78f;

    private static final class Feature {
        final int x, y, index;
        Feature(double x, double y, int index) {
            this.x = (int)Math.round(x); this.y = (int)Math.round(y); this.index = index;
        }
    }

    private static final class View {
        final int width, height;
        final float fx, fy, cx, cy;
        final float[] origin;
        final Pose pose;
        final List<Feature> features;
        final Mat descriptors;
        View(int width, int height, float fx, float fy, float cx, float cy,
             Pose pose, List<Feature> features, Mat descriptors) {
            this.width=width; this.height=height; this.fx=fx; this.fy=fy;
            this.cx=cx; this.cy=cy; this.pose=pose;
            this.origin=pose.getTranslation(); this.features=features;
            this.descriptors=descriptors;
        }
        void release() { if (descriptors != null) descriptors.release(); }
    }

    private static final class Candidate {
        final Feature oldFeature, newFeature;
        Candidate(Feature oldFeature, Feature newFeature) {
            this.oldFeature=oldFeature; this.newFeature=newFeature;
        }
    }

    private View previous;
    private long lastProcessMs;
    private int lastMatchCount;

    public synchronized int getLastMatchCount() { return lastMatchCount; }

    public synchronized void reset() {
        if (previous != null) previous.release();
        previous = null; lastProcessMs = 0; lastMatchCount = 0;
    }

    public synchronized int process(Frame frame, Image image, PointCloud cloud, long nowMs) {
        if (nowMs-lastProcessMs < 180) return 0;
        lastProcessMs=nowMs;
        if (image == null || image.getFormat()!=android.graphics.ImageFormat.YUV_420_888) return 0;

        Mat grayMat = new Mat();
        MatOfKeyPoint keypoints = new MatOfKeyPoint();
        Mat descriptors = new Mat();
        Mat mask = new Mat();
        try {
            int width=image.getWidth(), height=image.getHeight();
            Image.Plane plane=image.getPlanes()[0];
            ByteBuffer ybuf=plane.getBuffer().duplicate();
            int rowStride=plane.getRowStride(), pixelStride=plane.getPixelStride();
            byte[] gray=new byte[width*height];
            for(int y=0;y<height;y++) {
                int row=y*rowStride, dst=y*width;
                for(int x=0;x<width;x++) {
                    int at=row+x*pixelStride;
                    if(at<ybuf.limit()) gray[dst+x]=ybuf.get(at);
                }
            }
            grayMat.create(height,width,CvType.CV_8UC1);
            grayMat.put(0,0,gray);

            ORB orb=ORB.create(MAX_FEATURES);
            orb.detectAndCompute(grayMat,mask,keypoints,descriptors);
            KeyPoint[] points=keypoints.toArray();
            ArrayList<Feature> features=new ArrayList<>(points.length);
            for(int i=0;i<points.length;i++) {
                KeyPoint kp=points[i];
                if(kp.pt.x>=4 && kp.pt.y>=4 && kp.pt.x<width-4 && kp.pt.y<height-4)
                    features.add(new Feature(kp.pt.x,kp.pt.y,i));
            }

            com.google.ar.core.CameraIntrinsics intr=frame.getCamera().getImageIntrinsics();
            float[] focal=intr.getFocalLength(), principal=intr.getPrincipalPoint();
            int[] dimensions=intr.getImageDimensions();
            float sx=width/(float)Math.max(1,dimensions[0]);
            float sy=height/(float)Math.max(1,dimensions[1]);
            float fx=focal[0]*sx, fy=focal[1]*sy, cx=principal[0]*sx, cy=principal[1]*sy;
            if(fx<=0 || fy<=0) { descriptors.release(); return 0; }

            View current=new View(width,height,fx,fy,cx,cy,frame.getCamera().getPose(),features,descriptors);
            descriptors=new Mat(); // ownership transferred to current
            int added=0;
            lastMatchCount=0;

            if(previous!=null && previous.width==width && previous.height==height
                    && !previous.descriptors.empty() && !current.descriptors.empty()) {
                float dx=current.origin[0]-previous.origin[0];
                float dy=current.origin[1]-previous.origin[1];
                float dz=current.origin[2]-previous.origin[2];
                float baseline=(float)Math.sqrt(dx*dx+dy*dy+dz*dz);
                if(baseline>=MIN_BASELINE_METERS) {
                    BFMatcher matcher=BFMatcher.create(Core.NORM_HAMMING,false);
                    List<MatOfDMatch> pairs=new ArrayList<>();
                    try {
                        matcher.knnMatch(previous.descriptors,current.descriptors,pairs,2);
                        boolean[] usedCurrent=new boolean[points.length];
                        ArrayList<Candidate> candidates=new ArrayList<>();
                        for(MatOfDMatch pair:pairs) {
                            DMatch[] matches=pair.toArray();
                            if(matches.length<2) continue;
                            DMatch best=matches[0], second=matches[1];
                            if(best.distance>MAX_DESCRIPTOR_DISTANCE || best.distance>=RATIO_TEST*second.distance) continue;
                            int oldIndex=best.queryIdx, newIndex=best.trainIdx;
                            if(oldIndex<0 || newIndex<0 || newIndex>=usedCurrent.length || usedCurrent[newIndex]) continue;
                            Feature old=findFeature(previous.features,oldIndex);
                            Feature now=findFeature(current.features,newIndex);
                            if(old==null || now==null) continue;
                            usedCurrent[newIndex]=true;
                            if(Math.abs(old.x-now.x)<2 && Math.abs(old.y-now.y)<2) continue;
                            candidates.add(new Candidate(old,now));
                        }

                        // Geometric verification rejects descriptor matches that do not
                        // agree with a common two-view epipolar geometry.
                        if(candidates.size()>=8) {
                            ArrayList<Point> oldPoints=new ArrayList<>(candidates.size());
                            ArrayList<Point> newPoints=new ArrayList<>(candidates.size());
                            for(Candidate candidate:candidates) {
                                oldPoints.add(new Point(candidate.oldFeature.x,candidate.oldFeature.y));
                                newPoints.add(new Point(candidate.newFeature.x,candidate.newFeature.y));
                            }
                            MatOfPoint2f oldMat=new MatOfPoint2f(); oldMat.fromList(oldPoints);
                            MatOfPoint2f newMat=new MatOfPoint2f(); newMat.fromList(newPoints);
                            Mat inlierMask=new Mat();
                            Mat fundamental=null;
                            try {
                                fundamental=Calib3d.findFundamentalMat(oldMat,newMat,Calib3d.FM_RANSAC,2.5,0.99,inlierMask);
                                if(fundamental!=null && !fundamental.empty() && inlierMask.total()>=candidates.size()) {
                                    for(int i=0;i<candidates.size();i++) {
                                        double[] inlier=inlierMask.get(i,0);
                                        if(inlier==null || inlier.length==0 || inlier[0]==0) continue;
                                        Candidate candidate=candidates.get(i);
                                        float[] world=triangulate(previous,candidate.oldFeature,current,candidate.newFeature);
                                        if(world==null) continue;
                                        int[] color=sampleColor(image,gray,candidate.newFeature.x,candidate.newFeature.y);
                                        if(cloud.add(world[0],world[1],world[2],color[0],color[1],color[2],0f,0f,0f,0.65f)) added++;
                                        lastMatchCount++;
                                    }
                                }
                            } finally {
                                oldMat.release();newMat.release();inlierMask.release();
                                if(fundamental!=null) fundamental.release();
                            }
                        }
                    } finally {
                        for(MatOfDMatch pair:pairs) pair.release();
                    }
                }
            }
            if(previous!=null) previous.release();
            previous=current;
            return added;
        } catch(Exception ignored) {
            descriptors.release();
            return 0;
        } finally {
            grayMat.release(); keypoints.release(); mask.release();
        }
    }

    private static int[] sampleColor(Image image, byte[] gray, int x, int y) {
        Image.Plane[] planes=image.getPlanes();
        int yy=gray[y*image.getWidth()+x]&255;
        Image.Plane up=planes[1], vp=planes[2];
        ByteBuffer ub=up.getBuffer().duplicate(), vb=vp.getBuffer().duplicate();
        int ui=(y/2)*up.getRowStride()+(x/2)*up.getPixelStride();
        int vi=(y/2)*vp.getRowStride()+(x/2)*vp.getPixelStride();
        if(ui<0 || vi<0 || ui>=ub.limit() || vi>=vb.limit()) return new int[]{yy,yy,yy};
        int u=(ub.get(ui)&255)-128, v=(vb.get(vi)&255)-128;
        return new int[]{clamp(yy+(int)(1.402f*v)),clamp(yy-(int)(0.344136f*u+0.714136f*v)),clamp(yy+(int)(1.772f*u))};
    }

    private static int clamp(int value) { return Math.max(0,Math.min(255,value)); }

    private static Feature findFeature(List<Feature> features,int index) {
        for(Feature f:features) if(f.index==index) return f;
        return null;
    }

    private static float[] triangulate(View a,Feature fa,View b,Feature fb) {
        float[] da=worldRay(a,fa), db=worldRay(b,fb);
        float[] oa=a.origin, ob=b.origin;
        float[] w0={oa[0]-ob[0],oa[1]-ob[1],oa[2]-ob[2]};
        float aa=dot(da,da), bb=dot(da,db), cc=dot(db,db);
        float dd=dot(da,w0), ee=dot(db,w0);
        float denom=aa*cc-bb*bb;
        if(Math.abs(denom)<1e-5f) return null;
        float s=(bb*ee-cc*dd)/denom, t=(aa*ee-bb*dd)/denom;
        if(s<=0.05f || t<=0.05f || s>8f || t>8f) return null;
        float[] p={oa[0]+s*da[0],oa[1]+s*da[1],oa[2]+s*da[2]};
        float[] q={ob[0]+t*db[0],ob[1]+t*db[1],ob[2]+t*db[2]};
        float ex=p[0]-q[0],ey=p[1]-q[1],ez=p[2]-q[2];
        if(ex*ex+ey*ey+ez*ez>0.025f) return null;
        return new float[]{(p[0]+q[0])*0.5f,(p[1]+q[1])*0.5f,(p[2]+q[2])*0.5f};
    }

    private static float[] worldRay(View v,Feature f) {
        float[] local={(f.x-v.cx)/v.fx,-(f.y-v.cy)/v.fy,-1f};
        float[] endpoint=new float[3];
        v.pose.transformPoint(local,0,endpoint,0);
        float[] ray={endpoint[0]-v.origin[0],endpoint[1]-v.origin[1],endpoint[2]-v.origin[2]};
        float len=(float)Math.sqrt(dot(ray,ray));
        if(len>0) { ray[0]/=len;ray[1]/=len;ray[2]/=len; }
        return ray;
    }
    private static float dot(float[] a,float[] b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
}
