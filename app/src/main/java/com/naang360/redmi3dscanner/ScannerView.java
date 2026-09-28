package com.naang360.redmi3dscanner;

import android.content.Context;
import android.graphics.*;
import android.util.AttributeSet;
import android.view.View;
import android.media.Image;
import java.nio.ByteBuffer;

public final class ScannerView extends View {
    private final Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final PointCloud cloud = new PointCloud(250000);
    private Bitmap preview;
    private int trackedPoints;

    public ScannerView(Context context) { super(context); init(); }
    public ScannerView(Context context, AttributeSet attrs) { super(context, attrs); init(); }
    public ScannerView(Context context, AttributeSet attrs, int defStyleAttr) { super(context, attrs, defStyleAttr); init(); }

    private void init() {
        paint.setTextSize(18f);
        setBackgroundColor(Color.BLACK);
    }

    public PointCloud cloud() { return cloud; }

    public synchronized void setPreview(Image image) {
        try {
            Bitmap next = yuvToBitmap(image, 480, 360);
            Bitmap old = preview;
            preview = next;
            if (old != null && !old.isRecycled()) old.recycle();
        } catch (Exception ignored) {}
    }

    public void setTrackedPoints(int count) {
        trackedPoints = count;
    }

    @Override protected synchronized void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        if (preview != null && !preview.isRecycled()) {
            Rect src = new Rect(0, 0, preview.getWidth(), preview.getHeight());
            float scale = Math.max((float)getWidth()/preview.getWidth(), (float)getHeight()/preview.getHeight());
            int w = Math.round(preview.getWidth()*scale);
            int h = Math.round(preview.getHeight()*scale);
            int l = (getWidth()-w)/2, t = (getHeight()-h)/2;
            canvas.drawBitmap(preview, src, new Rect(l,t,l+w,t+h), paint);
        }

        paint.setStyle(Paint.Style.FILL);
        paint.setColor(Color.argb(210, 0, 0, 0));
        canvas.drawRect(0, 0, getWidth(), 58, paint);
        paint.setColor(Color.WHITE);
        canvas.drawText("REDMI 3D SCANNER", 16, 25, paint);
        paint.setTextSize(13f);
        canvas.drawText("Tracked feature points: " + trackedPoints + "  •  Cloud vertices: " + cloud.size(), 16, 46, paint);
        paint.setTextSize(18f);
    }

    private static Bitmap yuvToBitmap(Image image, int outW, int outH) {
        int iw = image.getWidth(), ih = image.getHeight();
        Bitmap b = Bitmap.createBitmap(outW, outH, Bitmap.Config.ARGB_8888);
        Image.Plane[] p = image.getPlanes();
        ByteBuffer yb=p[0].getBuffer().duplicate(), ub=p[1].getBuffer().duplicate(), vb=p[2].getBuffer().duplicate();
        int ys=p[0].getRowStride(), us=p[1].getRowStride(), vs=p[2].getRowStride();
        int yps=p[0].getPixelStride(), ups=p[1].getPixelStride(), vps=p[2].getPixelStride();
        int[] row = new int[outW];
        for (int oy=0; oy<outH; oy++) {
            int sy = oy*ih/outH;
            for (int ox=0; ox<outW; ox++) {
                int sx = ox*iw/outW;
                int yi=sy*ys+sx*yps;
                int ui=(sy/2)*us+(sx/2)*ups;
                int vi=(sy/2)*vs+(sx/2)*vps;
                int Y=(yi<yb.limit()?yb.get(yi)&255:0);
                int U=(ui<ub.limit()?ub.get(ui)&255:128)-128;
                int V=(vi<vb.limit()?vb.get(vi)&255:128)-128;
                int r=clamp(Y+(int)(1.402f*V));
                int g=clamp(Y-(int)(0.344136f*U+0.714136f*V));
                int bl=clamp(Y+(int)(1.772f*U));
                row[ox]=Color.rgb(r,g,bl);
            }
            b.setPixels(row,0,outW,0,oy,outW,1);
        }
        return b;
    }

    private static int clamp(int v) { return Math.max(0, Math.min(255, v)); }
}