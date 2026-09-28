package com.naang360.redmi3dscanner;

import android.Manifest;
import android.content.ContentValues;
import android.content.pm.PackageManager;
import android.media.Image;
import android.net.Uri;
import android.os.Bundle;
import android.os.Environment;
import android.provider.MediaStore;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;

import com.google.ar.core.ArCoreApk;
import com.google.ar.core.Config;
import com.google.ar.core.Frame;
import com.google.ar.core.Pose;
import com.google.ar.core.PointCloud;
import com.google.ar.core.Session;
import com.google.ar.core.TrackingState;
import com.google.ar.core.exceptions.UnavailableDeviceNotCompatibleException;

import java.io.File;
import java.io.FileInputStream;
import java.io.OutputStream;
import java.nio.FloatBuffer;
import java.util.Locale;

public class MainActivity extends AppCompatActivity {
    private static final int CAMERA = 10;

    private Session session;
    private boolean scanning;
    private boolean installRequested;
    private boolean depthSupported;
    private boolean sessionReady;

    private SensorFusion fusion;
    private KeyframeRecorder keyframes;
    private ScannerView scannerView;
    private TextView status, sensors, root;
    private Thread captureThread;

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
            if (scanning) {
                scannerView.cloud().clear();
                keyframes.clear();
                status.setText("Scanning… move slowly around the object");
            } else {
                status.setText("Scan paused • " + scannerView.cloud().size() + " vertices captured");
            }
        });

        findViewById(R.id.exportButton).setOnClickListener(v -> exportScan());

        scannerView.setCameraTextureListener(textureId -> runOnUiThread(() -> {
            if (session == null) return;
            try {
                session.setCameraTextureName(textureId);
                if (!sessionReady) {
                    sessionReady = true;
                    session.resume();
                    status.setText(depthSupported
                            ? "CAMERA LIVE • depth + tracking"
                            : "CAMERA LIVE • visual tracking");
                    startCaptureThread();
                }
            } catch (Exception e) {
                status.setText("Camera texture error: " + e.getClass().getSimpleName());
            }
        }));

        if (checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            requestPermissions(new String[]{Manifest.permission.CAMERA}, CAMERA);
        } else {
            startAr();
        }
    }

    private void startAr() {
        try {
            if (ArCoreApk.getInstance().requestInstall(this, !installRequested)
                    == ArCoreApk.InstallStatus.INSTALL_REQUESTED) {
                installRequested = true;
                return;
            }

            session = new Session(this);
            Config config = session.getConfig();

            depthSupported = session.isDepthModeSupported(Config.DepthMode.RAW_DEPTH_ONLY);
            if (depthSupported) {
                config.setDepthMode(Config.DepthMode.RAW_DEPTH_ONLY);
            }

            session.configure(config);
            status.setText("Initializing ARCore camera…");

            // Do NOT resume until ScannerView has created the OpenGL camera texture.
            // ARCore's camera frames are delivered into that texture.
        } catch (UnavailableDeviceNotCompatibleException e) {
            status.setText("ARCore unavailable on this device");
        } catch (Exception e) {
            status.setText("AR startup failed: " + e.getClass().getSimpleName());
        }
    }

    private void startCaptureThread() {
        if (captureThread != null && captureThread.isAlive()) return;
        captureThread = new Thread(this::captureLoop, "reconstruction-capture");
        captureThread.start();
    }

    private void captureLoop() {
        while (!isFinishing() && session != null) {
            if (!sessionReady) {
                sleep(50);
                continue;
            }

            try {
                Frame frame = session.update();
                if (frame.getCamera().getTrackingState() != TrackingState.TRACKING) {
                    scannerView.renderFrame();
                    sleep(10);
                    continue;
                }

                int featureCount = 0;

                if (scanning) {
                    Pose pose = frame.getCamera().getPose();
                    float[] t = pose.getTranslation();
                    float[] q = pose.getRotationQuaternion();
                    float[] pose7 = new float[]{
                            t[0], t[1], t[2], q[0], q[1], q[2], q[3]
                    };

                    // The camera preview is already rendered by the OpenGL texture.
                    // The capture side only consumes tracking/depth data.
                    keyframes.add(System.currentTimeMillis(), pose7, 1920, 1080, 1.0f);

                    featureCount = captureTrackingPoints(frame);

                    if (depthSupported) {
                        Image rgb = null;
                        try {
                            rgb = frame.acquireCameraImage();
                            captureDepth(frame, rgb, pose);
                        } finally {
                            if (rgb != null) rgb.close();
                        }
                    }
                }

                scannerView.renderFrame();

                final int points = scannerView.cloud().size();
                final int features = featureCount;
                final int frames = keyframes.size();

                runOnUiThread(() -> {
                    if (scanning) {
                        status.setText(String.format(Locale.US,
                                "TRACKING • %d vertices • %d features • %d keyframes",
                                points, features, frames));
                    } else {
                        status.setText("CAMERA LIVE • press START SCAN");
                    }
                });

            } catch (Exception e) {
                // ARCore can briefly reject update() while lifecycle state changes.
                sleep(20);
            }
        }
    }

    private int captureTrackingPoints(Frame frame) {
        int accepted = 0;

        try (PointCloud pc = frame.acquirePointCloud()) {
            FloatBuffer points = pc.getPoints();

            // ARCore point cloud layout: X,Y,Z,confidence.
            // Voxel thinning in PointCloud prevents runaway growth.
            for (int i = 0; i + 3 < points.limit(); i += 4) {
                float x = points.get(i);
                float y = points.get(i + 1);
                float z = points.get(i + 2);
                float confidence = points.get(i + 3);

                if (!Float.isFinite(x) || !Float.isFinite(y) || !Float.isFinite(z)) continue;
                if (confidence < 0.12f || z < -20f || z > 100f) continue;

                if (scannerView.cloud().add(
                        x, y, z,
                        235, 245, 255,
                        confidence)) {
                    accepted++;
                }
            }
        } catch (Exception ignored) {
        }

        return accepted;
    }

    private void captureDepth(Frame frame, Image rgb, Pose pose) {
        try (Image depth = frame.acquireRawDepthImage16Bits()) {
            Image.Plane dp = depth.getPlanes()[0];
            java.nio.ByteBuffer db = dp.getBuffer().duplicate();

            int dw = depth.getWidth();
            int dh = depth.getHeight();
            int rw = rgb.getWidth();
            int rh = rgb.getHeight();

            Image.Plane[] rp = rgb.getPlanes();
            Image.Plane yp = rp[0];
            Image.Plane up = rp[1];
            Image.Plane vp = rp[2];

            java.nio.ByteBuffer yb = yp.getBuffer().duplicate();
            java.nio.ByteBuffer ub = up.getBuffer().duplicate();
            java.nio.ByteBuffer vb = vp.getBuffer().duplicate();

            float[] tr = pose.getTranslation();

            for (int y = 2; y < dh - 2; y += 4) {
                int drow = y * dp.getRowStride();

                for (int x = 2; x < dw - 2; x += 4) {
                    int off = drow + x * dp.getPixelStride();
                    if (off + 1 >= db.limit()) continue;

                    int mm = (db.get(off) & 255)
                            | ((db.get(off + 1) & 255) << 8);

                    if (mm < 80 || mm > 8000) continue;

                    float z = mm / 1000f;
                    float nx = (x - dw * 0.5f) / (float) dw;
                    float ny = (y - dh * 0.5f) / (float) dh;

                    int rx = Math.min(rw - 1, Math.max(0, x * rw / dw));
                    int ry = Math.min(rh - 1, Math.max(0, y * rh / dh));

                    int[] color = yuvAt(
                            rx, ry,
                            yp, up, vp,
                            yb, ub, vb);

                    scannerView.cloud().add(
                            tr[0] + nx * z,
                            tr[1] - ny * z,
                            tr[2] + z,
                            color[0], color[1], color[2],
                            0f, 0f, 0f,
                            Math.min(1f, 1f - z / 8f));
                }
            }
        } catch (Exception ignored) {
        }
    }

    private static int[] yuvAt(
            int x, int y,
            Image.Plane yp, Image.Plane up, Image.Plane vp,
            java.nio.ByteBuffer yb,
            java.nio.ByteBuffer ub,
            java.nio.ByteBuffer vb) {

        int yi = y * yp.getRowStride() + x * yp.getPixelStride();
        int ux = x / 2;
        int uy = y / 2;

        int ui = uy * up.getRowStride() + ux * up.getPixelStride();
        int vi = uy * vp.getRowStride() + ux * vp.getPixelStride();

        if (yi >= yb.limit() || ui >= ub.limit() || vi >= vb.limit()) {
            return new int[]{255, 255, 255};
        }

        int Y = yb.get(yi) & 255;
        int U = (ub.get(ui) & 255) - 128;
        int V = (vb.get(vi) & 255) - 128;

        return new int[]{
                clamp(Y + (int) (1.402f * V)),
                clamp(Y - (int) (0.344136f * U + 0.714136f * V)),
                clamp(Y + (int) (1.772f * U))
        };
    }

    private void exportScan() {
        int vertices = scannerView.cloud().size();

        if (vertices == 0) {
            Toast.makeText(this,
                    "Nothing to export — scan first and capture some vertices.",
                    Toast.LENGTH_LONG).show();
            return;
        }

        try {
            File dir = new File(getCacheDir(), "exports");
            if (!dir.exists() && !dir.mkdirs()) {
                throw new Exception("Cannot create export cache");
            }

            String stamp = "scan-" + System.currentTimeMillis();

            File ply = new File(dir, stamp + ".ply");
            File csv = new File(dir, stamp + "-keyframes.csv");

            scannerView.cloud().writePly(ply);
            keyframes.writeCsv(csv);

            Uri plyUri = publishDownload(
                    ply, ply.getName(), "application/octet-stream");
            Uri csvUri = publishDownload(
                    csv, csv.getName(), "text/csv");

            ply.delete();
            csv.delete();

            if (plyUri != null && csvUri != null) {
                Toast.makeText(this,
                        "Saved to Download/Redmi3DScanner",
                        Toast.LENGTH_LONG).show();
            } else {
                Toast.makeText(this,
                        "Export failed",
                        Toast.LENGTH_LONG).show();
            }

        } catch (Exception e) {
            Toast.makeText(this,
                    "Export failed: " + e.getMessage(),
                    Toast.LENGTH_LONG).show();
        }
    }

    private Uri publishDownload(File source, String name, String mime) throws Exception {
        if (android.os.Build.VERSION.SDK_INT >= 29) {
            ContentValues values = new ContentValues();
            values.put(MediaStore.Downloads.DISPLAY_NAME, name);
            values.put(MediaStore.Downloads.MIME_TYPE, mime);
            values.put(
                    MediaStore.Downloads.RELATIVE_PATH,
                    Environment.DIRECTORY_DOWNLOADS + "/Redmi3DScanner");

            Uri uri = getContentResolver().insert(
                    MediaStore.Downloads.EXTERNAL_CONTENT_URI,
                    values);

            if (uri == null) throw new Exception("MediaStore insert failed");

            try (FileInputStream in = new FileInputStream(source);
                 OutputStream out = getContentResolver().openOutputStream(uri)) {

                if (out == null) throw new Exception("Cannot open destination");

                byte[] buffer = new byte[8192];
                int n;

                while ((n = in.read(buffer)) != -1) {
                    out.write(buffer, 0, n);
                }
            }

            return uri;
        }

        File downloads = Environment.getExternalStoragePublicDirectory(
                Environment.DIRECTORY_DOWNLOADS);

        File targetDir = new File(downloads, "Redmi3DScanner");

        if (!targetDir.exists() && !targetDir.mkdirs()) {
            throw new Exception("Cannot create Downloads folder");
        }

        File target = new File(targetDir, name);

        try (FileInputStream in = new FileInputStream(source);
             OutputStream out = new java.io.FileOutputStream(target)) {

            byte[] buffer = new byte[8192];
            int n;

            while ((n = in.read(buffer)) != -1) {
                out.write(buffer, 0, n);
            }
        }

        return Uri.fromFile(target);
    }

    private static int clamp(int x) {
        return Math.max(0, Math.min(255, x));
    }

    private static void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ignored) {
        }
    }

    @Override protected void onResume() {
        super.onResume();

        if (session != null && sessionReady) {
            try {
                session.resume();
            } catch (Exception ignored) {
            }
        }
    }

    @Override protected void onPause() {
        if (session != null && sessionReady) {
            try {
                session.pause();
            } catch (Exception ignored) {
            }
        }

        super.onPause();
    }

    @Override protected void onDestroy() {
        if (fusion != null) fusion.close();

        if (session != null) {
            try {
                session.close();
            } catch (Exception ignored) {
            }
        }

        super.onDestroy();
    }

    @Override public void onRequestPermissionsResult(
            int requestCode,
            @NonNull String[] permissions,
            @NonNull int[] grantResults) {

        super.onRequestPermissionsResult(
                requestCode, permissions, grantResults);

        if (requestCode == CAMERA
                && grantResults.length > 0
                && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
            startAr();
        }
    }
}