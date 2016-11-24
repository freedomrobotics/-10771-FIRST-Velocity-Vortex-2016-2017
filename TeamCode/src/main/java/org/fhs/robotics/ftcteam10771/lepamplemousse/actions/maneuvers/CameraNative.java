package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers;

import android.app.Activity;
import android.content.pm.ActivityInfo;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.DisplayMetrics;
import android.view.GestureDetector;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.widget.FrameLayout;
import android.widget.LinearLayout;
import android.widget.RelativeLayout;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Vuforia;
import com.vuforia.samples.ImageTargets.DebugLog;
import com.vuforia.samples.ImageTargets.ImageTargets;
import com.vuforia.samples.ImageTargets.ImageTargetsRenderer;
import com.vuforia.samples.ImageTargets.Texture;
import com.vuforia.samples.ImageTargets.VuforiaSampleGLView;
import com.vuforia.samples.ImageTargets.ui.SampleAppMenu.SampleAppMenu;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.util.Vector;

/**
 * Created by joelv on 11/23/2016.
 */


@Autonomous (name = "CameraNative", group = "10771")
public class CameraNative extends LinearOpMode{

    //Added own code
    //private FtcRobotControllerActivity FTC = new FtcRobotControllerActivity();
    // Focus mode constants:
    private static final int FOCUS_MODE_NORMAL = 0;
    private static final int FOCUS_MODE_CONTINUOUS_AUTO = 1;
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
    final static int CMD_BACK = -1;
    final static int CMD_EXTENDED_TRACKING = 1;
    final static int CMD_AUTOFOCUS = 2;
    final static int CMD_FLASH = 3;
    final static int CMD_CAMERA_FRONT = 4;
    final static int CMD_CAMERA_REAR = 5;
    final static int CMD_DATASET_STONES_AND_CHIPS_DATASET = 6;
    final static int CMD_DATASET_TARMAC_DATASET = 7;
    final static int STONES_AND_CHIPS_DATASET_ID = 0;
    final static int TARMAC_DATASET_ID = 1;
    final static int CAMERA_DIRECTION_DEFAULT = 0;
    final static int CAMERA_DIRECTION_BACK = 1;
    final static int CAMERA_DIRECTION_FRONT = 2;
    private native void switchDatasetAsap(int datasetId);
    private native boolean autofocus();
    private native boolean setFocusMode(int mode);
    private native boolean activateFlash(boolean flash);
    public native void initApplicationNative(int width, int height);
    private native void setActivityPortraitMode(boolean isPortrait);
    public native void deinitApplicationNative();
    public native int initTracker();
    public native void deinitTracker();
    public native int loadTrackerData();
    public native void destroyTrackerData();
    public native void onVuforiaInitializedNative();
    public native void startCamera(int camera);
    public native void stopCamera();
    private native boolean startExtendedTracking();
    private native boolean stopExtendedTracking();
    //Name for library sample
    public static final String library = "ImageTargetsNative";
    // Constants for Hiding/Showing Loading dialog
    static final int HIDE_LOADING_DIALOG = 0;
    static final int SHOW_LOADING_DIALOG = 1;
    //private View mLoadingDialogContainer;
    // Our OpenGL view:
    private VuforiaSampleGLView mGlView;
    // Our renderer:
    //private ImageTargetsRenderer mRenderer;
    //The root view of the activity
    private View rootView;
    // Display size of the device
    private int mScreenWidth = 0;
    private int mScreenHeight = 0;
    // Constant representing invalid screen orientation to trigger a query:
    private static final int INVALID_SCREEN_ROTATION = -1;
    // Last detected screen rotation:
    private int mLastScreenRotation = INVALID_SCREEN_ROTATION;
    // Keeps track of the current camera
    //int mCurrentCamera = CAMERA_DIRECTION_DEFAULT;
    int mCurrentCamera = 0;
    // The current application status:
    private int mAppStatus = APPSTATUS_UNINITED;
    // See CameraClass.java for full comment
    //private Object mShutdownLock = new Object();
    // Vuforia initialization flags:
    private int mVuforiaFlags = 0;
    // The textures we will use for rendering:
    private Vector<Texture> mTextures;
    // Detects the double tap gesture for launching the Camera menu
    private GestureDetector mGestureDetector;
    private SampleAppMenu mSampleAppMenu;
    // Contextual Menu Options for Camera Flash - Autofocus
    private boolean mFlash = false;
    private boolean mContAutofocus = false;
    private boolean mExtendedTracking = false;
    //private View mFlashOptionView;
    //Source of ExceptionNullPointer
    //private RelativeLayout mUILayout = (RelativeLayout) rootView.findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);
    boolean mIsDroidDevice = false;
    int depthSize = 16;
    int stencilSize = 0;
    boolean translucent = Vuforia.requiresAlpha();

    static
    {
        System.loadLibrary("Vuforia");
        System.loadLibrary(library);
    }
    @Override
    public void runOpMode() throws InterruptedException{

        /*mGlView.init(translucent, depthSize, stencilSize);
        ((Activity)hardwareMap.appContext).addContentView(mGlView, new ViewGroup.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT));
        */
        telemetry.addData("do", "it");
        mTextures = new Vector<Texture>();
        waitForStart();
        Handler h = new Handler(Looper.getMainLooper());
        h.post(new Runnable() {
            @Override
            public void run() {
                mGlView = new VuforiaSampleGLView(hardwareMap.appContext);
            }
        });
        loadTextures();
        Vuforia.setInitParameters(FTC, mVuforiaFlags, "AVj2kiX/////AAAAGV0J5W5oOkUTvP1+IKxrWdIpD63oQV8zSY/+qSNDxkt5zj8tW0N9AK7/3yUJRBlnJx80gStuZcHF7JoiKUNj4JmO6gcyIQn2LWZ/0hL9gFM+PmwM6lvzJu9U/gmvf++GngzR74ft0gjlNPle9qDHEaAgMHYcbEDpc4msHDVn6ZjCcxDem2tQyW4gEY334fwAU9E0ySkw1KwC/Mo6gaE7bW1Mh9xLbXYTe2+sRclEA6YbrKeH8LHmJBDXQxTdcL4HyS26oPYAGRXfFLoi7QkBdkPDYKiPQUsCoHhNz1uhPh5duEdwOD9Sm6YUPZYet7Mo9QP3sxaDlaqY5l2pHYn/tH31Xu9eqLKe2RmNRzgMNaJ9\n");
        Vuforia.init();
        initApplicationNative(100, 200);
        loadTrackerData();
        startCamera(1);
        while (opModeIsActive()){
            telemetry.addData("status", "yeet");
        }
        stopCamera();
    }

    private class GestureListener extends
            GestureDetector.SimpleOnGestureListener
    {
        public boolean onDown(MotionEvent e)
        {
            return true;
        }


        public boolean onSingleTapUp(MotionEvent e)
        {
            // Calls the Autofocus Native Method
            autofocus();

            // Triggering manual auto focus disables continuous
            // autofocus
            mContAutofocus = false;

            return true;
        }

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

    /** Stores screen dimensions *//*
    private void storeScreenDimensions()
    {
        // Query display dimensions:
        //DisplayMetrics metrics = new DisplayMetrics();
        //FTC.getWindowManager().getDefaultDisplay().getMetrics(metrics);
        mScreenWidth = mUILayout.getWidth();
        mScreenHeight = mUILayout.getHeight();
    }


    /**
     * Called when the activity first starts or the user navigates back to an
     * activity.
     *//*
    public void onCreate(Bundle savedInstanceState)
    {
        DebugLog.LOGD("onCreate");

        // Load any sample specific textures:
        mTextures = new Vector<Texture>();
        loadTextures();

        // Configure Vuforia to use OpenGL ES 2.0
        mVuforiaFlags = Vuforia.GL_20;

        // Creates the GestureDetector listener for processing double tap
        //mGestureDetector = new GestureDetector(FTC, new GestureListener());

        mIsDroidDevice = android.os.Build.MODEL.toLowerCase().startsWith(
                "droid");

    }

    protected void onResume()
    {

        // This is needed for some Droid devices to force portrait
        if (mIsDroidDevice)
        {
            //FTC.setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
            //FTC.setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);
        }

        // Vuforia-specific resume operation
        Vuforia.onResume();

        // We may start the camera only if the Vuforia SDK has already been
        // initialized

        // Resume the GL view:
        if (mGlView != null)
        {
            mGlView.setVisibility(View.VISIBLE);
            mGlView.onResume();
        }

    }
    */
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
}

