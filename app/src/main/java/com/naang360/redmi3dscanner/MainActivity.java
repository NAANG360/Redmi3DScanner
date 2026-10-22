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
import org.opencv.android.OpenCVLoader;

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
    private boolean visionReady;

    private SensorFusion fusion;
    private KeyframeRecorder keyframes;
    private ScanSessionArchive scanArchive;
    private FeatureReconstruction reconstruction;
    private final DenseMesh denseMesh = new DenseMesh();
    private final MonocularDepthModel monocularDepth = new MonocularDepthModel();
    private volatile boolean aiDepthReady;
    private volatile boolean aiDepthInitStarted;
    private ScannerView scannerView;
    private TextView status, sensors, root;
    private Thread captureThread;
    private long lastImageCaptureMs;
    private long lastDepthCaptureMs;
    private long lastAiDepthCaptureMs;

    @Override protected void onCreate(Bundle b) {
        super.onCreate(b);
        setContentView(R.layout.activity_main);

        status = findViewById(R.id.status);
        sensors = findViewById(R.id.sensors);
        root = findViewById(R.id.root);
        scannerView = findViewById(R.id.scannerView);

        visionReady = OpenCVLoader.initLocal();
        fusion = new SensorFusion(this);
        keyframes = new KeyframeRecorder(120);
        scanArchive = new ScanSessionArchive(getFilesDir());
        reconstruction = new FeatureReconstruction();

        sensors.setText(String.format(Locale.US,
                "ORB %s • Accel %s • Gyro %s • Mag %s",
                visionReady ? "READY" : "FAILED", fusion.hasAccelerometer(), fusion.hasGyroscope(), fusion.hasMagnetometer()));
        root.setText("Root: " + (RootAccess.isRootAvailable() ? "AVAILABLE" : "not available (optional)"));

        Button scan = findViewById(R.id.scanButton);
        scan.setOnClickListener(v -> {
            scanning = !scanning;
            scan.setText(scanning ? "STOP SCAN" : "START SCAN");
            if (scanning) {
                scannerView.cloud().clear();
                denseMesh.clear();
                keyframes.clear();
                scanArchive.reset();
                reconstruction.reset();
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
                            ? "CAMERA LIVE • ARCore depth + tracking"
                            : "CAMERA LIVE • visual tracking + MiDaS fallback");
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

            depthSupported = session.isDepthModeSupported(Config.DepthMode.AUTOMATIC);
            if (depthSupported) config.setDepthMode(Config.DepthMode.AUTOMATIC);

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

                    // Throttle CPU-heavy image analysis on the Snapdragon 660.
                    long now = System.currentTimeMillis();
                    if (now - lastImageCaptureMs >= 220) {
                        lastImageCaptureMs = now;
                        Image rgb = null;
                        try {
                            rgb = frame.acquireCameraImage();
                            if (rgb != null) scanArchive.maybeCapture(rgb, pose,
                                    frame.getCamera().getImageIntrinsics(), now);
                            if (visionReady) {
                                reconstruction.process(frame, rgb, scannerView.cloud(), now);
                                featureCount = Math.max(featureCount, reconstruction.getLastMatchCount());
                            }
                            if (depthSupported && now - lastDepthCaptureMs >= 440) {
                                lastDepthCaptureMs = now;
                                captureDepth(frame, rgb, pose);
                            }
                            if (!depthSupported) {
                                if (!aiDepthInitStarted) {
                                    aiDepthInitStarted = true;
                                    new Thread(() -> aiDepthReady = monocularDepth.initialize(getApplicationContext()),
                                            "midas-model-init").start();
                                }
                                if (aiDepthReady && now - lastAiDepthCaptureMs >= 900) {
                                    lastAiDepthCaptureMs = now;
                                    monocularDepth.reconstruct(getApplicationContext(), frame, rgb, denseMesh);
                                }
                            }
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
                                "TRACKING • %d points • %d matches • %d frames • %d mesh faces",
                                points, features, frames, denseMesh.faceCount()));
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
        Pose pose = frame.getCamera().getPose();
        float[] cameraPoint = new float[3];
        float[] worldPoint = new float[3];

        try (com.google.ar.core.PointCloud pc = frame.acquirePointCloud()) {
            FloatBuffer points = pc.getPoints();
            // ARCore point-cloud coordinates are camera-local; transform to world space.
            for (int i = 0; i + 3 < points.limit(); i += 4) {
                cameraPoint[0] = points.get(i);
                cameraPoint[1] = points.get(i + 1);
                cameraPoint[2] = points.get(i + 2);
                float confidence = points.get(i + 3);
                if (!Float.isFinite(cameraPoint[0]) || !Float.isFinite(cameraPoint[1]) || !Float.isFinite(cameraPoint[2])) continue;
                if (confidence < 0.12f || cameraPoint[2] < -20f || cameraPoint[2] > 100f) continue;
                pose.transformPoint(cameraPoint, 0, worldPoint, 0);
                if (scannerView.cloud().add(worldPoint[0], worldPoint[1], worldPoint[2],
                        235, 245, 255, confidence)) accepted++;
            }
        } catch (Exception ignored) {
        }
        return accepted;
    }

    /**
     * Fuse ARCore raw depth into the world-space cloud using calibrated camera rays.
     * ARCore camera coordinates look down -Z; Pose transforms camera-local points to world.
     */
    private void captureDepth(Frame frame, Image rgb, Pose pose) {
        try (Image depth = frame.acquireDepthImage16Bits()) {
            Image.Plane dp = depth.getPlanes()[0];
            java.nio.ByteBuffer db = dp.getBuffer().duplicate();
            int dw = depth.getWidth(), dh = depth.getHeight();
            int rw = rgb.getWidth(), rh = rgb.getHeight();
            com.google.ar.core.CameraIntrinsics intr = frame.getCamera().getImageIntrinsics();
            float[] focal = intr.getFocalLength(), principal = intr.getPrincipalPoint();
            int[] imageSize = intr.getImageDimensions();
            float sx = dw / (float)Math.max(1, imageSize[0]);
            float sy = dh / (float)Math.max(1, imageSize[1]);
            float fx=focal[0]*sx, fy=focal[1]*sy, cx=principal[0]*sx, cy=principal[1]*sy;
            if (!(fx>0f && fy>0f)) return;

            Image.Plane[] rp=rgb.getPlanes();
            Image.Plane yp=rp[0], up=rp[1], vp=rp[2];
            java.nio.ByteBuffer yb=yp.getBuffer().duplicate(), ub=up.getBuffer().duplicate(), vb=vp.getBuffer().duplicate();
            int step=6, rows=(dh+step-1)/step, cols=(dw+step-1)/step;
            int[][] grid=new int[rows][cols];
            for(int[] row:grid) java.util.Arrays.fill(row,-1);
            float[] cameraPoint=new float[3], worldPoint=new float[3];
            int pixelStride=Math.max(1,dp.getPixelStride()), rowStride=dp.getRowStride();

            for(int y=2;y<dh-2;y+=step) {
                int drow=y*rowStride;
                for(int x=2;x<dw-2;x+=step) {
                    int off=drow+x*pixelStride;
                    if(off<0 || off+1>=db.limit()) continue;
                    int mm=(db.get(off)&255)|((db.get(off+1)&255)<<8);
                    if(mm<150 || mm>6000) continue;
                    float z=mm/1000f;
                    cameraPoint[0]=(x-cx)*z/fx;
                    cameraPoint[1]=-(y-cy)*z/fy;
                    cameraPoint[2]=-z;
                    pose.transformPoint(cameraPoint,0,worldPoint,0);
                    int rx=Math.min(rw-1,Math.max(0,x*rw/dw));
                    int ry=Math.min(rh-1,Math.max(0,y*rh/dh));
                    int[] color=yuvAt(rx,ry,yp,up,vp,yb,ub,vb);
                    float confidence=Math.max(0.05f,Math.min(1f,1f-z/7f));
                    scannerView.cloud().add(worldPoint[0],worldPoint[1],worldPoint[2],
                            color[0],color[1],color[2],0f,0f,0f,confidence);
                    grid[y/step][x/step]=denseMesh.addVertex(worldPoint[0],worldPoint[1],worldPoint[2],
                            color[0],color[1],color[2]);
                }
            }
            for(int gy=0;gy<rows-1;gy++) for(int gx=0;gx<cols-1;gx++) {
                int a=grid[gy][gx], b=grid[gy][gx+1], c=grid[gy+1][gx], d=grid[gy+1][gx+1];
                if(a>=0 && b>=0 && c>=0) denseMesh.addTriangle(a,c,b,0.10f);
                if(b>=0 && c>=0 && d>=0) denseMesh.addTriangle(b,c,d,0.10f);
            }
        } catch(Exception ignored) {
            // ARCore depth is optional and device/runtime-dependent.
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

        if (vertices == 0 && (scanArchive == null || scanArchive.frameCount() == 0)) {
            Toast.makeText(this,
                    "Nothing to export — scan first and capture camera keyframes.",
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
            File obj = new File(dir, stamp + ".obj");
            File csv = new File(dir, stamp + "-keyframes.csv");
            File glb = new File(dir, stamp + ".glb");
            File stl = new File(dir, stamp + ".stl");
            File dataset = new File(dir, stamp + "-photogrammetry-dataset.zip");

            scannerView.cloud().writePly(ply);
            if (denseMesh.faceCount() > 0) {
                denseMesh.cleanup();
                denseMesh.writeObj(obj);
                denseMesh.writeGlb(glb);
                denseMesh.writeBinaryStl(stl);
            } else {
                scannerView.cloud().writeObj(obj);
            }
            keyframes.writeCsv(csv);
            if (scanArchive.frameCount() > 0) scanArchive.writeZip(dataset);

            Uri plyUri = publishDownload(ply, ply.getName(), "application/octet-stream");
            Uri objUri = publishDownload(obj, obj.getName(), "text/plain");
            Uri csvUri = publishDownload(csv, csv.getName(), "text/csv");
            Uri glbUri = glb.exists() ? publishDownload(glb, glb.getName(), "model/gltf-binary") : null;
            Uri stlUri = stl.exists() ? publishDownload(stl, stl.getName(), "model/stl") : null;
            Uri datasetUri = dataset.exists() ? publishDownload(dataset, dataset.getName(), "application/zip") : null;

            ply.delete();
            obj.delete();
            csv.delete();
            glb.delete();
            stl.delete();
            dataset.delete();

            if (plyUri != null && objUri != null && csvUri != null) {
                String formats = glbUri != null && stlUri != null ? "PLY + OBJ + GLB + STL"
                        : "PLY + OBJ (no triangulated mesh available for GLB/STL)";
                if (datasetUri != null) formats += " + " + scanArchive.frameCount() + " calibrated source frames (ZIP)";
                formats += " + keyframes CSV";
                Toast.makeText(this,
                        "Saved " + formats + " • " + denseMesh.vertexCount() + " mesh vertices / "
                                + denseMesh.faceCount() + " faces in Download/Redmi3DScanner",
                        Toast.LENGTH_LONG).show();
            } else {
                Toast.makeText(this, "Export failed", Toast.LENGTH_LONG).show();
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
        if (scanArchive != null) scanArchive.close();

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