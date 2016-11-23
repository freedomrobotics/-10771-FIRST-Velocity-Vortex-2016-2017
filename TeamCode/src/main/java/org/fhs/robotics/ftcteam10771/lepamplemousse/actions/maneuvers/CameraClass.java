package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.graphics.Color;
import android.util.DisplayMetrics;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.FrameLayout;

import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers.CameraVision;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;
import com.vuforia.samples.ImageTargets.ImageTargets;
import com.vuforia.samples.ImageTargets.ImageTargetsRenderer;
import com.vuforia.samples.ImageTargets.Texture;
import com.vuforia.samples.ImageTargets.VuforiaSampleGLView;

import java.util.Vector;

/**
 * Created by joelv on 11/19/2016.
 */

@Autonomous(name = "CameraC", group = "10771")
public class CameraClass extends LinearOpMode {

    // Our OpenGL view:
    private VuforiaSampleGLView mGlView;

    //The root view of the activity
    private View rootView;

    // Our renderer:
    private ImageTargetsRenderer mRenderer;

    // Display size of the device:
    DisplayMetrics metrics = new DisplayMetrics();
    private int mScreenWidth = 0;
    private int mScreenHeight = 0;
    int mLastScreenRotation = -1;

    private FrameLayout mUILayout;

    // Application status constants:
    private static final int APPSTATUS_UNINITED = -1;
    private static final int APPSTATUS_INIT_APP = 0;
    private static final int APPSTATUS_INIT_VUFORIA = 1;
    private static final int APPSTATUS_INIT_TRACKER = 2;
    private static final int APPSTATUS_INIT_APP_AR = 3;
    private static final int APPSTATUS_LOAD_TRACKER = 4;
    private static final int APPSTATUS_INITED = 5;
    private static final int APPSTATUS_CAMERA_STOPPED = 6;
    private static final int APPSTATUS_CAMERA_RUNNING = 7;

    public static final String library = "ImageTargetsNative";
    static{
        System.loadLibrary("Vuforia");
        System.loadLibrary(library);
    }

    // The current application status:
    private int mAppStatus = APPSTATUS_UNINITED;

    public native void startCamera(int camera);
    public native void stopCamera();
    public native void updateRendering(int width, int height);
    public native void initRendering();
    public native void initTracker();
    public native void deinitTracker();
    public native void loadTrackerData();
    public native void onVuforiaInitializedNative();
    public native void renderFrame();
    public native void initApplicationNative(int width, int height);
    public native void deinitApplicationNative();

    // The textures we will use for rendering:
    private Vector<Texture> mTextures;
    private void loadTextures()
    {
        mTextures.add(Texture.loadTextureFromApk("TextureTeapotBrass.png",
                FtcRobotControllerActivity.getGlobalAssets()));
        mTextures.add(Texture.loadTextureFromApk("TextureTeapotBlue.png",
                FtcRobotControllerActivity.getGlobalAssets()));
        mTextures.add(Texture.loadTextureFromApk("TextureTeapotRed.png",
                FtcRobotControllerActivity.getGlobalAssets()));
        mTextures
                .add(Texture.loadTextureFromApk("Buildings.jpeg", FtcRobotControllerActivity.getGlobalAssets()));
    }

    /** Returns the number of registered textures. */
    public int getTextureCount()
    {
        return mTextures.size();
    }

    /** Returns the texture object at the specified index. */
    public Texture getTexture(int i)
    {
        return mTextures.elementAt(i);
    }

    /** Initializes AR application components. */
    @SuppressLint("WrongViewCast")
    private void initApplicationAR()
    {
        // Do application initialization in native code (e.g. registering
        // callbacks, etc.):
        initApplicationNative(mScreenWidth, mScreenHeight);

        // Create OpenGL ES view:
        int depthSize = 16;
        int stencilSize = 0;
        boolean translucent = Vuforia.requiresAlpha();

        mGlView = new VuforiaSampleGLView(hardwareMap.appContext);
        mGlView.init(translucent, depthSize, stencilSize);

        mRenderer = new ImageTargetsRenderer();
        mRenderer.mActivity = this;
        mGlView.setRenderer(mRenderer);

        mUILayout = (FrameLayout) rootView.findViewById(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);

        mUILayout.setVisibility(View.VISIBLE);
        mUILayout.setBackgroundColor(Color.BLACK);

    }
    /**
     * Updates projection matrix and viewport after a screen rotation change was
     * detected.
     */
    public void updateRenderView()
    {
        int currentScreenRotation = ((Activity) hardwareMap.appContext).getWindowManager().getDefaultDisplay()
                .getRotation();
        if (currentScreenRotation != mLastScreenRotation)
        {
            // Set projection matrix if there is already a valid one:
            if (Vuforia.isInitialized()
                    && (mAppStatus == APPSTATUS_CAMERA_RUNNING))
            {
                // Query display dimensions:
                storeScreenDimensions();

                // Update viewport via renderer:
                mRenderer.updateRendering(mScreenWidth, mScreenHeight);

                // Cache last rotation used for setting projection matrix:
                mLastScreenRotation = currentScreenRotation;
            }
        }
    }

    /** Stores screen dimensions */
    private void storeScreenDimensions()
    {
        // Query display dimensions:
        mScreenWidth = mUILayout.getWidth();
        mScreenHeight = mUILayout.getHeight();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        rootView = ((Activity) hardwareMap.appContext).getWindow().getDecorView().findViewById(android.R.id.content);

        mTextures = new Vector<Texture>();
        loadTextures();

        initApplicationAR();

        storeScreenDimensions();

        Vuforia.setInitParameters((Activity) hardwareMap.appContext, 0, "AVj2kiX/////AAAAGV0J5W5oOkUTvP1+IKxrWdIpD63oQV8zSY/+qSNDxkt5zj8tW0N9AK7/3yUJRBlnJx80gStuZcHF7JoiKUNj4JmO6gcyIQn2LWZ/0hL9gFM+PmwM6lvzJu9U/gmvf++GngzR74ft0gjlNPle9qDHEaAgMHYcbEDpc4msHDVn6ZjCcxDem2tQyW4gEY334fwAU9E0ySkw1KwC/Mo6gaE7bW1Mh9xLbXYTe2+sRclEA6YbrKeH8LHmJBDXQxTdcL4HyS26oPYAGRXfFLoi7QkBdkPDYKiPQUsCoHhNz1uhPh5duEdwOD9Sm6YUPZYet7Mo9QP3sxaDlaqY5l2pHYn/tH31Xu9eqLKe2RmNRzgMNaJ9\n");

        int mProgressValue = 0;
        do
        {
            // Vuforia.init() blocks until an initialization step is
            // complete, then it proceeds to the next step and reports
            // progress in percents (0 ... 100%).
            // If Vuforia.init() returns -1, it indicates an error.
            // Initialization is done when progress has reached 100%.
            mProgressValue = Vuforia.init();

            // We check whether the task has been canceled in the
            // meantime (by calling AsyncTask.cancel(true)).
            // and bail out if it has, thus stopping this thread.
            // This is necessary as the AsyncTask will run to completion
            // regardless of the status of the component that
            // started is.
        } while (/*!isCancelled() &&*/ mProgressValue >= 0
                && mProgressValue < 100);

        if (mProgressValue > 0) return;

        boolean test = Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_OBJECT_TARGETS, 4);
        boolean test2 = Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        mScreenWidth = metrics.widthPixels;
        mScreenHeight = metrics.heightPixels;

        initApplicationNative(mScreenWidth, mScreenHeight);

        // Native post initialization:
        onVuforiaInitializedNative();

        // Now add the GL surface view. It is important
        // that the OpenGL ES surface view gets added
        // BEFORE the camera is started and video
        // background is configured.
        ((Activity)hardwareMap.appContext).addContentView(mGlView, new ViewGroup.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT));

        startCamera(0);
        initRendering();

        waitForStart();

        while (opModeIsActive()){
            mScreenWidth = metrics.widthPixels;
            mScreenHeight = metrics.heightPixels;
            telemetry.addData("width", mScreenWidth);
            telemetry.addData("height", mScreenHeight);
            updateRendering(mScreenWidth, mScreenHeight);
            telemetry.update();
        }
        stopCamera();
        deinitApplicationNative();
        // Unload texture:
        mTextures.clear();
        mTextures = null;
    }
}

