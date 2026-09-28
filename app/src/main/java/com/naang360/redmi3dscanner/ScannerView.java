package com.naang360.redmi3dscanner;

import android.content.Context;
import android.graphics.SurfaceTexture;
import android.opengl.GLES11Ext;
import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.util.AttributeSet;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

public final class ScannerView extends GLSurfaceView {
    public interface CameraTextureListener { void onTextureReady(int textureId); }

    private final PointCloud cloud = new PointCloud(250000);
    private RendererImpl renderer;
    private CameraTextureListener listener;

    public ScannerView(Context context) { super(context); init(); }
    public ScannerView(Context context, AttributeSet attrs) { super(context, attrs); init(); }
    public ScannerView(Context context, AttributeSet attrs, int defStyleAttr) { super(context, attrs); init(); }

    private void init() {
        setEGLContextClientVersion(2);
        setPreserveEGLContextOnPause(true);
        renderer = new RendererImpl();
        setRenderer(renderer);
        setRenderMode(GLSurfaceView.RENDERMODE_WHEN_DIRTY);
    }

    public PointCloud cloud() { return cloud; }

    public void setCameraTextureListener(CameraTextureListener l) {
        listener = l;
        if (renderer.textureId != 0) l.onTextureReady(renderer.textureId);
    }

    public void renderFrame() { requestRender(); }

    @Override public void onPause() {
        super.onPause();
        if (renderer != null) renderer.releaseSurfaceTexture();
    }

    private final class RendererImpl implements GLSurfaceView.Renderer {
        int textureId;
        SurfaceTexture cameraTexture;
        int program;
        int positionHandle, texHandle, matrixHandle;
        final float[] stMatrix = new float[16];
        final FloatBuffer vertices = ByteBuffer.allocateDirect(16).order(ByteOrder.nativeOrder()).asFloatBuffer();
        final FloatBuffer texCoords = ByteBuffer.allocateDirect(16).order(ByteOrder.nativeOrder()).asFloatBuffer();

        RendererImpl() {
            vertices.put(new float[]{-1f,-1f, 1f,-1f, -1f,1f, 1f,1f}).position(0);
            texCoords.put(new float[]{0f,1f, 1f,1f, 0f,0f, 1f,0f}).position(0);
        }

        @Override public void onSurfaceCreated(GL10 gl, EGLConfig config) {
            String vs =
                    "attribute vec2 aPosition;" +
                    "attribute vec2 aTexCoord;" +
                    "uniform mat4 uTexMatrix;" +
                    "varying vec2 vTex;" +
                    "void main(){gl_Position=vec4(aPosition,0.0,1.0);vTex=(uTexMatrix*vec4(aTexCoord,0.0,1.0)).xy;}";
            String fs =
                    "#extension GL_OES_EGL_image_external : require\n" +
                    "precision mediump float;" +
                    "uniform samplerExternalOES uTexture;" +
                    "varying vec2 vTex;" +
                    "void main(){gl_FragColor=texture2D(uTexture,vTex);}";
            program = buildProgram(vs, fs);
            positionHandle = GLES20.glGetAttribLocation(program, "aPosition");
            texHandle = GLES20.glGetAttribLocation(program, "aTexCoord");
            matrixHandle = GLES20.glGetUniformLocation(program, "uTexMatrix");

            int[] ids = new int[1];
            GLES20.glGenTextures(1, ids, 0);
            textureId = ids[0];
            GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, textureId);
            GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_LINEAR);
            GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR);
            GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_WRAP_S, GLES20.GL_CLAMP_TO_EDGE);
            GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_WRAP_T, GLES20.GL_CLAMP_TO_EDGE);

            cameraTexture = new SurfaceTexture(textureId);
            cameraTexture.setDefaultBufferSize(1920, 1080);
            if (listener != null) listener.onTextureReady(textureId);
        }

        @Override public void onSurfaceChanged(GL10 gl, int width, int height) {
            GLES20.glViewport(0, 0, width, height);
        }

        @Override public void onDrawFrame(GL10 gl) {
            GLES20.glClearColor(0f,0f,0f,1f);
            GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT);
            if (cameraTexture == null) return;
            try {
                cameraTexture.updateTexImage();
                cameraTexture.getTransformMatrix(stMatrix);
            } catch (RuntimeException ignored) { return; }

            GLES20.glUseProgram(program);
            GLES20.glUniformMatrix4fv(matrixHandle, 1, false, stMatrix, 0);
            vertices.position(0);
            GLES20.glEnableVertexAttribArray(positionHandle);
            GLES20.glVertexAttribPointer(positionHandle, 2, GLES20.GL_FLOAT, false, 0, vertices);
            texCoords.position(0);
            GLES20.glEnableVertexAttribArray(texHandle);
            GLES20.glVertexAttribPointer(texHandle, 2, GLES20.GL_FLOAT, false, 0, texCoords);
            GLES20.glDrawArrays(GLES20.GL_TRIANGLE_STRIP, 0, 4);
            GLES20.glDisableVertexAttribArray(positionHandle);
            GLES20.glDisableVertexAttribArray(texHandle);
        }

        void releaseSurfaceTexture() {
            if (cameraTexture != null) {
                try { cameraTexture.release(); } catch (Exception ignored) {}
                cameraTexture = null;
            }
        }

        private int buildProgram(String vs, String fs) {
            int v = compile(GLES20.GL_VERTEX_SHADER, vs);
            int f = compile(GLES20.GL_FRAGMENT_SHADER, fs);
            int p = GLES20.glCreateProgram();
            GLES20.glAttachShader(p, v);
            GLES20.glAttachShader(p, f);
            GLES20.glLinkProgram(p);
            int[] ok = new int[1];
            GLES20.glGetProgramiv(p, GLES20.GL_LINK_STATUS, ok, 0);
            if (ok[0] == 0) throw new RuntimeException(GLES20.glGetProgramInfoLog(p));
            return p;
        }

        private int compile(int type, String source) {
            int s = GLES20.glCreateShader(type);
            GLES20.glShaderSource(s, source);
            GLES20.glCompileShader(s);
            int[] ok = new int[1];
            GLES20.glGetShaderiv(s, GLES20.GL_COMPILE_STATUS, ok, 0);
            if (ok[0] == 0) throw new RuntimeException(GLES20.glGetShaderInfoLog(s));
            return s;
        }
    }
}