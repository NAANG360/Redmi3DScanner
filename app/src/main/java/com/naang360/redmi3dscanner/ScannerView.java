package com.naang360.redmi3dscanner;

import android.content.Context;
import android.util.AttributeSet;
import android.graphics.*;
import android.view.*;

public final class ScannerView extends View {
    private final Paint p = new Paint(1);
    private final PointCloud c = new PointCloud(250000);

    public ScannerView(Context context) {
        super(context);
        init();
    }

    public ScannerView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    public ScannerView(Context context, AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        init();
    }

    private void init() {
        p.setColor(Color.WHITE);
        p.setTextSize(24);
    }

    public PointCloud cloud() {
        return c;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        canvas.drawText("3D Scanner", 20, getHeight() - 70, p);
        canvas.drawText("Points: " + c.size(), 20, getHeight() - 35, p);
    }
}
