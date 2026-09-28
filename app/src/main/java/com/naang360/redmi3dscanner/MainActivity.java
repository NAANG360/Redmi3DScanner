package com.naang360.redmi3dscanner;

import android.Manifest;
import android.media.Image;
import android.os.Bundle;
import android.os.Environment;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;

import com.google.ar.core.ArCoreApk;
import com.google.ar.core.Config;
import com.google.ar.core.Frame;
import com.google.ar.core.Pose;
import com.google.ar.core.Session;
import com.google.ar.core.TrackingState;
import com.google.ar.core.exceptions.UnavailableDeviceNotCompatibleException;

import java.io.File;
import java.util.Locale;

/**
 * Capture controller. Depth is opportunistic: if the target device has no ARCore
 * depth support, we keep the tracking/keyframe foundation alive instead of faking depth.
 */
public class MainActivity extends AppCompatActivity {
    private static final int CAMERA = 10;
    private Session session;
    private boolean scanning;
    private boolean installRequested;
    private SensorFusion fusion;
    private KeyframeRecorder keyframes;
    private TextView status, sensors, root;
    private ScannerView scannerView;
    private boolean depthSupported;

    @Override protected void onCreate(Bundle b) {
        super.onCreate(b);
        setContentView(R.layout.activity_main);
        status = findViewById(R.id.status);
        sensors = findViewById(R.id.sensors);
        root = findViewById(R.id.root);
        scannerView = findViewById(R.id.scannerView);
        fusion = new SensorFusion(this);
        keyframes = new KeyframeRecorder(120);

        sensors.setText(String.format(Locale.US,
                "Accel %s • Gyro %s • Mag %s",
                fusion.hasAccelerometer(), fusion.hasGyroscope(), fusion.hasMagnetometer()));
        root.setText("Root: " + (RootAccess.isRootAvailable() ? "AVAILABLE" : "not available (optional)"));

        Button scan = findViewById(R.id.scanButton);
        scan.setOnClickListener(v -> {
            scanning = !scanning;
            scan.setText(scanning ? "STOP SCAN" : "START SCAN");
            status.setText(scanning ? "Capturing tracking + RGB…" : "Paused");
        });
        findViewById(R.id.exportButton).setOnClickListener(v -> exportScan());

        if (checkSelfPermission(Manifest.permission.CAMERA) != android.content.pm.PackageManager.PERMISSION_GRANTED)
            requestPermissions(new String[]{Manifest.permission.CAMERA}, CAMERA);
        else startAr();
    }

    private void startAr() {
        try {
            if (ArCoreApk.getInstance().requestInstall(this, !installRequested)
                    == ArCoreApk.InstallStatus.INSTALL_REQUESTED) {
                installRequested = true;
                return;
            }
            session = new Session(this);
            Config c = session.getConfig();
            depthSupported = session.isDepthModeSupported(Config.DepthMode.RAW_DEPTH_ONLY);
            if (depthSupported) c.setDepthMode(Config.DepthMode.RAW_DEPTH_ONLY);
            session.configure(c);
            session.resume();
            status.setText(depthSupported
                    ? "RGB + ARCore raw depth ready"
                    : "Tracking ready • depth unavailable • RGB keyframes active");
            new Thread(this::captureLoop, "reconstruction-capture").start();
        } catch (UnavailableDeviceNotCompatibleException e) {
            status.setText("ARCore unavailable — camera-only reconstruction backend next");
        } catch (Exception e) {
            status.setText("AR startup failed: " + e.getClass().getSimpleName());
        }
    }

    private void captureLoop() {
        while (!isFinishing() && session != null) {
            if (!scanning) { sleep(40); continue; }
            try {
                Frame frame = session.update();
                if (frame.getCamera().getTrackingState() != TrackingState.TRACKING) continue;
                Pose pose = frame.getCamera().getPose();
                float[] t = pose.getTranslation();
                float[] q = pose.getRotationQuaternion();
                float[] pose7 = new float[]{t[0],t[1],t[2],q[0],q[1],q[2],q[3]};

                Image rgb = null;
                try {
                    rgb = frame.acquireCameraImage();
                    keyframes.add(System.currentTimeMillis(), pose7, rgb.getWidth(), rgb.getHeight(), 1.0f);
                    if (depthSupported) captureDepth(frame, rgb, pose);
                } finally {
                    if (rgb != null) rgb.close();
                }

                int points = scannerView.cloud().size();
                runOnUiThread(() -> {
                    status.setText(String.format(Locale.US, "TRACKING • %d pts • %d keyframes", points, keyframes.size()));
                    scannerView.invalidate();
                });
            } catch (Exception ignored) {
                // Camera/depth frames are asynchronous; transient NotYetAvailable is expected.
            }
        }
    }

    private void captureDepth(Frame frame, Image rgb, Pose pose) {
        try (Image depth = frame.acquireRawDepthImage16Bits()) {
            Image.Plane dp = depth.getPlanes()[0];
            java.nio.ByteBuffer db = dp.getBuffer().duplicate();
            int dw = depth.getWidth(), dh = depth.getHeight();
            int rw = rgb.getWidth(), rh = rgb.getHeight();
            Image.Plane[] rp = rgb.getPlanes();
            Image.Plane yp = rp[0], up = rp[1], vp = rp[2];
            java.nio.ByteBuffer yb = yp.getBuffer().duplicate();
            java.nio.ByteBuffer ub = up.getBuffer().duplicate();
            java.nio.ByteBuffer vb = vp.getBuffer().duplicate();
            float[] tr = pose.getTranslation();

            for (int y=2; y<dh-2; y+=4) {
                int drow = y * dp.getRowStride();
                for (int x=2; x<dw-2; x+=4) {
                    int off = drow + x * dp.getPixelStride();
                    if (off + 1 >= db.limit()) continue;
                    int mm = (db.get(off) & 255) | ((db.get(off+1) & 255) << 8);
                    if (mm < 80 || mm > 8000) continue;
                    float z = mm / 1000f;
                    float nx = (x - dw * 0.5f) / (float) dw;
                    float ny = (y - dh * 0.5f) / (float) dh;
                    int rx = Math.min(rw - 1, Math.max(0, x * rw / dw));
                    int ry = Math.min(rh - 1, Math.max(0, y * rh / dh));
                    int[] rgbColor = yuvAt(rx, ry, yp, up, vp, yb, ub, vb);
                    scannerView.cloud().add(tr[0] + nx*z, tr[1] - ny*z, tr[2] + z,
                            rgbColor[0], rgbColor[1], rgbColor[2],
                            0f, 0f, 0f, Math.min(1f, 1.0f - (z / 8f)));
                }
            }
        } catch (Exception ignored) { }
    }

    private static int[] yuvAt(int x, int y,
                               Image.Plane yp, Image.Plane up, Image.Plane vp,
                               java.nio.ByteBuffer yb, java.nio.ByteBuffer ub, java.nio.ByteBuffer vb) {
        int yi = y * yp.getRowStride() + x * yp.getPixelStride();
        int ux = x / 2, uy = y / 2;
        int ui = uy * up.getRowStride() + ux * up.getPixelStride();
        int vi = uy * vp.getRowStride() + ux * vp.getPixelStride();
        if (yi >= yb.limit() || ui >= ub.limit() || vi >= vb.limit()) return new int[]{255,255,255};
        int Y = yb.get(yi) & 255;
        int U = (ub.get(ui) & 255) - 128;
        int V = (vb.get(vi) & 255) - 128;
        int r = Y + (int)(1.402f * V);
        int g = Y - (int)(0.344136f * U + 0.714136f * V);
        int b = Y + (int)(1.772f * U);
        return new int[]{clamp(r), clamp(g), clamp(b)};
    }

    private void exportScan() {
        try {
            File dir = getExternalFilesDir(Environment.DIRECTORY_DOCUMENTS);
            if (dir == null) throw new Exception("No storage directory");
            String stamp = "scan-" + System.currentTimeMillis();
            File ply = new File(dir, stamp + ".ply");
            File csv = new File(dir, stamp + "-keyframes.csv");
            scannerView.cloud().writePly(ply);
            keyframes.writeCsv(csv);
            Toast.makeText(this, "Saved RGB PLY + keyframe data", Toast.LENGTH_LONG).show();
        } catch (Exception e) {
            Toast.makeText(this, "Export failed: " + e.getMessage(), Toast.LENGTH_LONG).show();
        }
    }

    private static int clamp(int x) { return Math.max(0, Math.min(255, x)); }
    private static void sleep(long ms) { try { Thread.sleep(ms); } catch (InterruptedException ignored) {} }

    @Override protected void onPause() { super.onPause(); if (session != null) session.pause(); }
    @Override protected void onDestroy() { if (fusion != null) fusion.close(); if (session != null) session.close(); super.onDestroy(); }
    @Override public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions,
                                                     @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == CAMERA && grantResults.length > 0 &&
                grantResults[0] == android.content.pm.PackageManager.PERMISSION_GRANTED) startAr();
    }
}