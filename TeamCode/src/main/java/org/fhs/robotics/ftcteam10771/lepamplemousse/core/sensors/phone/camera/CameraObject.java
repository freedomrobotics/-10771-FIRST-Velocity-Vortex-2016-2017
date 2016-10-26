package org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera;

import android.app.Activity;
import android.content.Context;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.os.Handler;
import android.os.HandlerThread;
import android.util.Log;
import android.view.Display;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.widget.FrameLayout;

import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.ReturnValues;

import java.io.ByteArrayOutputStream;
import java.io.IOException;

/**
 * Camera Object. Used to create camera and pull a preview and image from it.
 *
 * <p>Must be constructed with reference to the main Activity (eg FtcRobotControllerActivity which is
 * accessible from (Activity) hardwareMap.appContext).</p>
 *
 * <p>Should only be constructed once since more than one CameraObject will provide unexpected results
 * without throwing an error other than {@link ReturnValues ReturnValues.CAMERA_ALREADY_OPEN}.
 * (Maybe I should have it throw an exception)</p>
 *
 * <p>After construction, {@link #initCamera} is called to start and initialize the camera. Once the
 * camera is started, it's must be stopped with {@link #stopCamera}.</p>
 *
 * <p>To retrieve camera data, pull values from the subclass {@link CameraData}.</p>
 * <p>&nbsp;<br>Created by Adam Li on 12/27/2015.<br>&nbsp;</p>
 *
 * <p>Created with the help of below resources:
 * <ul>
 * <li><a href=https://github.com/cheer4ftc/OpModeCamera>https://github.com/cheer4ftc/OpModeCamera</a>
 * <li><a href=http://developer.android.com/guide/topics/media/camera.html>
 * http://developer.android.com/guide/topics/media/camera.html</a>
 * <li><a href=http://stackoverflow.com/questions/3841122/android-camera-preview-is-sideways>
 * http://stackoverflow.com/questions/3841122/android-camera-preview-is-sideways</a>
 * <li><a href=http://stackoverflow.com/questions/18149964/best-use-of-handlerthread-over-other-similar-classes/19154438#19154438>
 * http://stackoverflow.com/questions/18149964/best-use-of-handlerthread-over-other-similar-classes/19154438#19154438</a>
 * </ul>
 * </p>
 */
public final class CameraObject {

    //Debug Tag that is never really used in the end
    private static final String TAG = "CameraDebug";
    //The Camera object
    private static android.hardware.Camera camera = null;
    //The {@link CameraPreview} object
    private static CameraPreview cameraPreview = null;
    /**
     * The data from the camera preview. Contains frame width, height, Camera orientation, and YUV and RGB image types.
     * <p/>
     * <p>See more at {@link CameraData}</p>
     */
    public CameraData cameraData = new CameraData();
    //The only config for this class you'll ever need.
    private int frameLayoutId = com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId;
    //The context of the app
    private Context context;
    //The downsampling ratio
    private int downSample;
    private int downSampleBitmap;
    //The root view of the activity
    private View rootView;
    //The main activity
    private Activity activity;
    //The data from the camera
    //The FrameLayout object of camera preview
    private FrameLayout cameraPreviewLayout;
    //A debug variable
    //public int debug = 0;
    //The preview callback from the camera to for image data of the live preview
    private android.hardware.Camera.PreviewCallback previewCallback = new android.hardware.Camera.PreviewCallback() {
        @Override
        public void onPreviewFrame(byte[] data, android.hardware.Camera camera) {
            try {
                android.hardware.Camera.Parameters parameters = camera.getParameters();
                cameraData.setWidth(parameters.getPreviewSize().width);
                cameraData.setHeight(parameters.getPreviewSize().height);
                cameraData.setYuvImage(new YuvImage(data, ImageFormat.NV21, cameraData.getWidth(), cameraData.getHeight(), null));
                //debug++;
            } catch (Exception e) {

            }
        }
    };
    //The thread that both opens the camera and runs the previewCallback. "reduces" lag on the main thread.
    private CameraHandlerThread cameraHandlerThread = null;

    /**
     * Constructor to create the Camera object given the Activity with the preview frame and the downsampling ratio.
     * <p/>
     * <p><b>NOTE:</b> The activity must have the FrameLayout defined in the frameLayoutId variable!</p>
     * <p><b>NOTE:</b> This is defined by hardwareMap.appContext in the FTC SDK, but it must be casted to (Activity)</p>
     *
     * @param activity   The main android activity
     * @param downSample The downsampling ratio from {@link Downsample}
     */
    public CameraObject(Activity activity, Downsample downSample) {
        this(activity, activity, downSample);
    }

    /**
     * Constructor to create the Camera object given the context of the Activity with the preview frame and the downsampling ratio.
     * <p/>
     * <p><b>NOTE:</b> The activity must have the FrameLayout defined in the frameLayoutId variable!</p>
     * <p><b>NOTE:</b> This is defined by hardwareMap.appContext in the FTC SDK</p>
     *
     * @param context    The context of the main android activity
     * @param downSample The downsampling ratio from {@link Downsample}
     */
    public CameraObject(Context context, Downsample downSample) {
        this((Activity) context, context, downSample);
    }

    /**
     * Constructor to create the Camera object given the Activity with the preview frame, the Context of the application, and the downsampling ratio.
     * <p/>
     * <p><b>NOTE:</b> The activity must have the FrameLayout defined in the frameLayoutId variable!</p>
     *
     * @param activity   The main android activity
     * @param context    The context of the application
     * @param downSample The downsampling ratio from {@link Downsample}
     */
    public CameraObject(Activity activity, Context context, Downsample downSample) {
        this.context = context;
        cameraData.setDownSample(downSample);
        this.downSample = downSample.getPreview();
        this.downSampleBitmap = downSample.getRGB();
        rootView = activity.getWindow().getDecorView().findViewById(android.R.id.content);
        this.activity = activity;
    }

    /**
     * From the android tutorial, apparently a safe way to get the camera object
     *
     * @return android.hardware.Camera
     */
    private static android.hardware.Camera getCameraInstance() {
        android.hardware.Camera c = null;
        try {
            c = android.hardware.Camera.open(); // attempt to get a Camera instance
        } catch (Exception e) {
            // Camera is not available (in use or does not exist)
        }
        return c; // returns null if camera is unavailable
    }

    /**
     * Method to initialize and open the camera. {@link #stopCamera} must be called after finished with this!
     *
     * @return A {@link ReturnValues} value for the camera status.
     * <li><b>ReturnValues.SUCCESS:</b> The camera was successfully initialized and opened.
     * <li><b>ReturnValues.CAMERA_ALREADY_OPEN:</b> The camera was already opened for whatever reason.
     * If this is returned, an existing object already has the camera.
     * <li><b>ReturnValues.CAMERA_FAILED_TO_OPEN:</b> The camera failed to open for whatever reason.
     * <li><b>ReturnValues.CAMERA_DOESNT_EXIST:</b> This device does not have a camera.
     */
    public ReturnValues initCamera() {
        //Check of the device has a camera.
        if (context.getPackageManager().hasSystemFeature(PackageManager.FEATURE_CAMERA)) {
            //Camera exists on device and is not defined yet
            if (camera == null) {
                cameraHandlerThread = new CameraHandlerThread();

                synchronized (cameraHandlerThread) {
                    cameraHandlerThread.openCamera();
                }
                //Camera exists on device, but for whatever reason, it could not open
                if (camera == null) {
                    return ReturnValues.CAMERA_FAILED_TO_OPEN;
                }
                //Camera exists on device and was opened, so create preview
                createPreview();
                return ReturnValues.SUCCESS;
            }
            //Camera exists on device and was defined already, so create preview (maybe)
            createPreview();
            return ReturnValues.CAMERA_ALREADY_OPEN;
        }
        //Camera does not exist
        return ReturnValues.CAMERA_DOESNT_EXIST;
    }

    //reate the on-screen preview of the camera.
    private void createPreview() {
        if (cameraPreview == null) {
            activity.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    cameraPreview = new CameraPreview(camera, previewCallback);
                    cameraPreviewLayout = (FrameLayout) rootView.findViewById(frameLayoutId);
                    cameraPreviewLayout.addView(cameraPreview);

                }
            });
        }
    }

    /**
     * Method to stop camera. MUST BE CALLED OR ELSE (that is, if you called {@link #initCamera} at all).
     */
    public void stopCamera() {
        if (camera != null) {
            // TODO: 12/29/2015 don't let it remove the preview until the last instance of camera has be removed. Use static counter.
            if (cameraPreview != null) {
                activity.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        cameraPreviewLayout.removeAllViews();

                    }
                });
                cameraPreview = null;
            }
            camera.stopPreview();
            camera.setPreviewCallback(null);
            if (camera != null) {
                camera.release();
            }
            camera = null;
        }
    }

    /**
     * <p>The enumeration for the orientations of the camera. (Goes counterClockwise)</p>
     * <p/>
     * Inside:
     * <ul>
     * <li>PORTRAIT
     * <li>LANDSCAPE
     * <li>PORTRAIT_FLIPPED
     * <li>LANDSCAPE_FLIPPED
     * </ul>
     */
    public enum Orientation {
        PORTRAIT,
        LANDSCAPE,
        PORTRAIT_FLIPPED,
        LANDSCAPE_FLIPPED
    }

    /**
     * <p>An enumeration of downsampling ratios for the camera.</p>
     * <p/>
     * Inside:
     * <ul>
     * <li>FULL
     * <li>HALF
     * <li>FOURTH
     * <li>EIGHTH
     * <li>SIXTEENTH
     * <li>BY32
     * <li>BY64<br>&nbsp;
     * <li>{@link #getPreview()} Returns the numerical(int) ratio associated with the value for the camera preview.
     * <li>{@link #getRGB()} Returns the numerical(int) ratio associated with the value for the RGB bitmap.
     * <li>{@link #getValue()} Returns the numerical(int) ratio associated with the value.
     * </ul>
     */
    public enum Downsample {
        FULL(1, 1),
        HALF(2, 1),
        FOURTH(4, 1),
        EIGHTH(8, 1),
        SIXTEENTH(8, 2),
        BY32(8, 4),
        BY64(8, 8),
        BY128(8, 16);

        private final int preview;
        private final int RGB;

        Downsample(final int preview, final int RGB) {
            this.preview = preview;
            this.RGB = RGB;
        }

        /**
         * Returns the numerical(int) ratio associated with the value for the camera preview.
         */
        public int getPreview() {
            return preview;
        }

        /**
         * Returns the numerical(int) ratio associated with the value for the RGB bitmap.
         */
        public int getRGB() {
            return RGB;
        }

        /**
         * Returns the numerical(int) ratio associated with the value.
         */
        public int getValue() {
            return preview*RGB;
        }
    }

    /**
     * Based on the code from the android camera tutorial.
     * Creates the preview for the camera.
     */
    public class CameraPreview extends SurfaceView implements SurfaceHolder.Callback {
        private SurfaceHolder holder;
        private android.hardware.Camera camera;
        private android.hardware.Camera.PreviewCallback previewCallback = null;

        public CameraPreview(android.hardware.Camera camera, android.hardware.Camera.PreviewCallback previewCallback) {
            super(context);
            this.camera = camera;

            // Install a SurfaceHolder.Callback so we get notified when the
            // underlying surface is created and destroyed.
            holder = getHolder();
            holder.addCallback(this);
            // deprecated setting, but required on Android versions prior to 3.0
            holder.setType(SurfaceHolder.SURFACE_TYPE_PUSH_BUFFERS);

            this.previewCallback = previewCallback;
        }

        /**
         * This is called immediately after the surface is first created.
         * Implementations of this should start up whatever rendering code
         * they desire.  Note that only one thread can ever draw into
         * a {@link Surface}, so you should not draw into the Surface here
         * if your normal rendering will be in another thread.
         *
         * @param holder The SurfaceHolder whose surface is being created.
         */
        @Override
        public void surfaceCreated(SurfaceHolder holder) {
            // The Surface has been created, now tell the camera where to draw the preview.

            //Check because if user spams, the thing crashes since surface might be removed before this is called.
            //Doesn't stop the crash, just makes it less likely.
            if (this.holder.getSurface() == null) {
                // preview surface does not exist
                return;
            }

            android.hardware.Camera.Parameters parameters = camera.getParameters();
            Display display = activity.getWindowManager().getDefaultDisplay();

            if (display.getRotation() == Surface.ROTATION_0) {
                cameraData.setOrientation(Orientation.PORTRAIT);
            }

            if (display.getRotation() == Surface.ROTATION_90) {
                cameraData.setOrientation(Orientation.LANDSCAPE);
            }

            if (display.getRotation() == Surface.ROTATION_180) {
                cameraData.setOrientation(Orientation.PORTRAIT_FLIPPED);
            }

            if (display.getRotation() == Surface.ROTATION_270) {
                cameraData.setOrientation(Orientation.LANDSCAPE_FLIPPED);
            }

            // start preview with new settings
            camera.setParameters(parameters);
            camera.setPreviewCallback(previewCallback);
            try {
                camera.setPreviewDisplay(holder);
                camera.startPreview();
            } catch (IOException e) {
                Log.d(TAG, "Error setting camera preview: " + e.getMessage());
            }
        }

        /**
         * This is called immediately after any structural changes (format or
         * size) have been made to the surface.  You should at this point update
         * the imagery in the surface.  This method is always called at least
         * once, after {@link #surfaceCreated}.
         *
         * @param holder The SurfaceHolder whose surface has changed.
         * @param format The new PixelFormat of the surface.
         * @param w      The new width of the surface.
         * @param h      The new height of the surface.
         */
        // Adapted from http://stackoverflow.com/questions/3841122/android-camera-preview-is-sideways
        @Override
        public void surfaceChanged(SurfaceHolder holder, int format, int w, int h) {
            // If your preview can change or rotate, take care of those events here.
            // Make sure to stop the preview before resizing or reformatting it.

            if (this.holder.getSurface() == null) {
                // preview surface does not exist
                return;
            }

            // stop preview before making changes
            try {
                camera.stopPreview();
            } catch (Exception e) {
                // ignore: tried to stop a non-existent preview
            }

            // set preview size and make any resize, rotate or
            // reformatting changes here

            android.hardware.Camera.Parameters parameters = camera.getParameters();
            // FIXME: 12/28/2015 Doesn't reorient the camera, but doesn't matter for now
            Display display = activity.getWindowManager().getDefaultDisplay();

            if (display.getRotation() == Surface.ROTATION_0) {
                cameraData.setOrientation(Orientation.PORTRAIT);
            }

            if (display.getRotation() == Surface.ROTATION_90) {
                cameraData.setOrientation(Orientation.LANDSCAPE);
            }

            if (display.getRotation() == Surface.ROTATION_180) {
                cameraData.setOrientation(Orientation.PORTRAIT_FLIPPED);
            }

            if (display.getRotation() == Surface.ROTATION_270) {
                cameraData.setOrientation(Orientation.LANDSCAPE_FLIPPED);
            }

            // start preview with new settings
            camera.setParameters(parameters);
            camera.setPreviewCallback(previewCallback);
            try {
                camera.setPreviewDisplay(this.holder);
                camera.startPreview();

            } catch (Exception e) {
                Log.d(TAG, "Error starting camera preview: " + e.getMessage());
            }
        }

        /**
         * This is called immediately before a surface is being destroyed. After
         * returning from this call, you should no longer try to access this
         * surface.  If you have a rendering thread that directly accesses
         * the surface, you must ensure that thread is no longer touching the
         * Surface before returning from this function.
         *
         * @param holder The SurfaceHolder whose surface is being destroyed.
         */
        @Override
        public void surfaceDestroyed(SurfaceHolder holder) {
            // empty. Take care of releasing the Camera preview in your activity.
        }

        //public android.hardware.Camera.PreviewCallback getPreview(){
        //    return previewCallback;
        //}
    }

    //Adapted from http://stackoverflow.com/questions/18149964/best-use-of-handlerthread-over-other-similar-classes/19154438#19154438
    //Creats a seperate thread for the camera read and stuff
    private class CameraHandlerThread extends HandlerThread {
        Handler mHandler = null;

        CameraHandlerThread() {
            super("CameraHandlerThread");
            start();
            mHandler = new Handler(getLooper());
        }

        synchronized void notifyCameraOpened() {
            notify();
        }

        void openCamera() {
            mHandler.post(new Runnable() {
                @Override
                public void run() {
                    camera = getCameraInstance();

                    camera.setPreviewCallback(previewCallback);

                    android.hardware.Camera.Parameters parameters = camera.getParameters();

                    cameraData.setWidth(parameters.getPreviewSize().width / downSample);
                    cameraData.setHeight(parameters.getPreviewSize().height / downSample);
                    parameters.setPreviewSize(cameraData.getWidth(), cameraData.getHeight());

                    camera.setParameters(parameters);

                    notifyCameraOpened();
                }
            });
            try {
                wait();
            } catch (InterruptedException e) {
                Log.w(TAG, "Camera open wait was interrupted");
            }
        }
    }

    /**
     * <p><b>Camera Data</b></p>
     * <p/>
     * <p>This class structures the data from the camera preview. Contains frame width, height, Camera orientation, and YUV and RGB image types.<br>&nbsp;</p>
     * <p/>
     * Inside:
     * <ul>
     * <li>{@link #getDownSample()}
     * <li>{@link #getWidth()}
     * <li>{@link #getHeight()}
     * <li>{@link #getOrientation()}
     * <li>{@link #setOrientation setOrientation(}{@link Orientation}{@link #setOrientation )}
     * <li>{@link #getYuvImage()}
     * <li>{@link #getRgbImage()}
     * </ul>
     */
    public class CameraData {
        private int w, h;
        private Downsample d = null;
        private YuvImage yuvImage;
        private Orientation o = Orientation.PORTRAIT;

        /**
         * @return The downsampling ratio (see {@link Downsample})
         */
        public Downsample getDownSample() {
            return d;
        }

        protected void setDownSample(Downsample d) {
            this.d = d;
        }

        /**
         * @return The width of the camera preview data as an int
         */
        public int getWidth() {
            return w;
        }

        protected void setWidth(int w) {
            this.w = w;
        }

        /**
         * @return The height of the camera preview data as an int
         */
        public int getHeight() {
            return h;
        }

        protected void setHeight(int h) {
            this.h = h;
        }

        /**
         * @return The YUV image data as a {@link YuvImage}
         */
        public YuvImage getYuvImage() {
            return yuvImage;
        }

        protected void setYuvImage(YuvImage data) {
            this.yuvImage = data;
        }

        /**
         * @return The RGB image data as a {@link Bitmap}
         */
        //From cheer4ftc
        public Bitmap getRgbImage() {
            if (yuvImage == null) return null;
            Bitmap rgbImage;
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            yuvImage.compressToJpeg(new Rect(0, 0, w, h), 0, out);
            byte[] imageBytes = out.toByteArray();

            BitmapFactory.Options opt;
            opt = new BitmapFactory.Options();
            opt.inSampleSize = downSampleBitmap;

            rgbImage = BitmapFactory.decodeByteArray(imageBytes, 0, imageBytes.length, opt);
            return rgbImage;
        }

        /**
         * @return The orientation of the camera preview as a {@link Orientation}
         */
        public Orientation getOrientation() {
            return o;
        }

        /**
         * Changes the orientation of the camera preview.
         *
         * @param o {@link Orientation}
         */
        public void setOrientation(Orientation o) {
            if (o == Orientation.PORTRAIT) {
                camera.setDisplayOrientation(90);
            }

            if (o == Orientation.LANDSCAPE) {
                camera.setDisplayOrientation(0);
            }

            if (o == Orientation.PORTRAIT_FLIPPED) {
                camera.setDisplayOrientation(270);
            }

            if (o == Orientation.LANDSCAPE_FLIPPED) {
                camera.setDisplayOrientation(180);
            }
            this.o = o;
        }
    }
}
