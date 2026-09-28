package com.naang360.redmi3dscanner;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/** Stores reconstruction observations so dense/mesh stages can reprocess a scan. */
public final class KeyframeRecorder {
    public static final class Keyframe {
        public final long timeMs;
        public final float[] pose;
        public final int width, height;
        public final float featureQuality;

        Keyframe(long timeMs, float[] pose, int width, int height, float featureQuality) {
            this.timeMs=timeMs; this.pose=pose; this.width=width; this.height=height;
            this.featureQuality=featureQuality;
        }
    }

    private final List<Keyframe> frames = new ArrayList<>();
    private long lastCaptureMs;
    private final long minIntervalMs;

    public KeyframeRecorder(long minIntervalMs) { this.minIntervalMs = Math.max(50, minIntervalMs); }

    public synchronized boolean add(long timeMs, float[] pose, int width, int height, float quality) {
        if (timeMs - lastCaptureMs < minIntervalMs) return false;
        lastCaptureMs = timeMs;
        frames.add(new Keyframe(timeMs, pose.clone(), width, height, quality));
        return true;
    }

    public synchronized int size() { return frames.size(); }
    public synchronized void clear() { frames.clear(); lastCaptureMs = 0; }

    public synchronized void writeCsv(File file) throws Exception {
        try (BufferedWriter w = new BufferedWriter(new FileWriter(file))) {
            w.write("timestamp_ms,tx,ty,tz,qx,qy,qz,qw,width,height,feature_quality\n");
            for (Keyframe k : frames) {
                w.write(String.format(Locale.US,
                        "%d,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%d,%d,%.5f\n",
                        k.timeMs, k.pose[0], k.pose[1], k.pose[2],
                        k.pose[3], k.pose[4], k.pose[5], k.pose[6],
                        k.width, k.height, k.featureQuality));
            }
        }
    }
}