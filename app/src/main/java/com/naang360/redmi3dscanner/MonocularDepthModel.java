package com.naang360.redmi3dscanner;

import android.content.Context;
import android.media.Image;
import com.google.ar.core.Frame;
import com.google.ar.core.Pose;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.dnn.Dnn;
import org.opencv.dnn.Net;
import org.opencv.imgproc.Imgproc;
import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.util.Arrays;

/** Optional MiDaS-small monocular depth fallback. Depth is relative, not metric. */
public final class MonocularDepthModel {
    private static final String MODEL_ASSET = "midas_small.onnx";
    private static final int INPUT = 256;
    private Net net;
    private boolean attempted;

    public synchronized boolean initialize(Context context) {
        if (net != null) return true;
        if (attempted) return false;
        attempted = true;
        File model = new File(context.getFilesDir(), MODEL_ASSET);
        try {
            if (!model.exists() || model.length() < 1_000_000L) {
                try (InputStream in=context.getAssets().open(MODEL_ASSET);
                     FileOutputStream out=new FileOutputStream(model)) {
                    byte[] buffer=new byte[65536]; int n;
                    while((n=in.read(buffer))!=-1) out.write(buffer,0,n);
                }
            }
            net=Dnn.readNetFromONNX(model.getAbsolutePath());
            return net != null && !net.empty();
        } catch(Exception e) {
            net=null;
            return false;
        }
    }

    /**
     * Produces an approximate relative-depth surface when ARCore depth is unavailable.
     * The depth-to-metre mapping is deliberately heuristic and must not be treated as
     * measurement-grade geometry.
     */
    public synchronized int reconstruct(Context context, Frame frame, Image image, DenseMesh mesh) {
        if(net==null || image==null || image.getFormat()!=android.graphics.ImageFormat.YUV_420_888) return 0;
        Mat bgr=new Mat(), resized=new Mat(), blob=new Mat(), output=null;
        try {
            int w=image.getWidth(), h=image.getHeight();
            Image.Plane[] planes=image.getPlanes();
            Image.Plane yp=planes[0], up=planes[1], vp=planes[2];
            ByteBuffer yb=yp.getBuffer().duplicate(), ub=up.getBuffer().duplicate(), vb=vp.getBuffer().duplicate();
            byte[] pixels=new byte[w*h*3];
            for(int y=0;y<h;y++) for(int x=0;x<w;x++) {
                int yi=y*yp.getRowStride()+x*yp.getPixelStride();
                int ui=(y/2)*up.getRowStride()+(x/2)*up.getPixelStride();
                int vi=(y/2)*vp.getRowStride()+(x/2)*vp.getPixelStride();
                if(yi>=yb.limit()||ui>=ub.limit()||vi>=vb.limit()) continue;
                int yy=yb.get(yi)&255, u=(ub.get(ui)&255)-128, v=(vb.get(vi)&255)-128;
                int i=(y*w+x)*3;
                pixels[i]=(byte)clamp(yy+(int)(1.772f*u));       // B
                pixels[i+1]=(byte)clamp(yy-(int)(0.344136f*u+0.714136f*v)); // G
                pixels[i+2]=(byte)clamp(yy+(int)(1.402f*v));    // R
            }
            bgr.create(h,w,CvType.CV_8UC3); bgr.put(0,0,pixels);
            Imgproc.resize(bgr,resized,new Size(INPUT,INPUT));
            blob=Dnn.blobFromImage(resized,1.0/127.5,new Size(INPUT,INPUT),
                    new Scalar(1.0,1.0,1.0),true,false);
            net.setInput(blob);
            output=net.forward();
            int oh=(int)output.size(output.dims()-2), ow=(int)output.size(output.dims()-1);
            if(oh<8||ow<8) return 0;
            float[] raw=new float[oh*ow]; output.get(new int[output.dims()],raw);
            float min=Float.POSITIVE_INFINITY,max=Float.NEGATIVE_INFINITY;
            for(float v:raw) if(Float.isFinite(v)){min=Math.min(min,v);max=Math.max(max,v);}
            if(!Float.isFinite(min)||!Float.isFinite(max)||max-min<1e-6f) return 0;

            int step=4, rows=(oh+step-1)/step, cols=(ow+step-1)/step;
            int[][] grid=new int[rows][cols]; for(int[] row:grid) Arrays.fill(row,-1);
            com.google.ar.core.CameraIntrinsics intr=frame.getCamera().getImageIntrinsics();
            float[] focal=intr.getFocalLength(), principal=intr.getPrincipalPoint();
            int[] dims=intr.getImageDimensions();
            float sx=INPUT/(float)Math.max(1,dims[0]), sy=INPUT/(float)Math.max(1,dims[1]);
            float fx=focal[0]*sx, fy=focal[1]*sy, cx=principal[0]*sx, cy=principal[1]*sy;
            Pose pose=frame.getCamera().getPose();
            float[] local=new float[3], world=new float[3];
            int added=0;
            for(int y=1;y<oh-1;y+=step) for(int x=1;x<ow-1;x+=step) {
                float d=raw[y*ow+x]; if(!Float.isFinite(d)) continue;
                float normalized=(d-min)/(max-min);
                // MiDaS output is relative inverse depth: higher values generally mean closer.
                float z=2.5f-2.0f*normalized;
                local[0]=(x-cx)*z/fx; local[1]=-(y-cy)*z/fy; local[2]=-z;
                pose.transformPoint(local,0,world,0);
                int idx=mesh.addVertex(world[0],world[1],world[2],180,190,205);
                grid[y/step][x/step]=idx; if(idx>=0) added++;
            }
            for(int gy=0;gy<rows-1;gy++) for(int gx=0;gx<cols-1;gx++) {
                int a=grid[gy][gx], b=grid[gy][gx+1], c=grid[gy+1][gx], d=grid[gy+1][gx+1];
                if(a>=0&&b>=0&&c>=0) mesh.addTriangle(a,c,b,0.10f);
                if(b>=0&&c>=0&&d>=0) mesh.addTriangle(b,c,d,0.10f);
            }
            return added;
        } catch(Exception ignored) {
            return 0;
        } finally {
            bgr.release(); resized.release(); blob.release(); if(output!=null) output.release();
        }
    }
    private static int clamp(int x){return Math.max(0,Math.min(255,x));}
}