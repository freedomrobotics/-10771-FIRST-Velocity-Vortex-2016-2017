package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.DialogInterface;
import android.content.pm.ActivityInfo;
import android.graphics.Color;
import android.os.AsyncTask;
import android.view.View;
import android.view.ViewGroup;
import android.view.WindowManager;
import android.widget.LinearLayout;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Vuforia;
import com.vuforia.samples.ImageTargets.DebugLog;
import com.vuforia.samples.ImageTargets.ImageTargetsRenderer;
import com.vuforia.samples.ImageTargets.Texture;
import com.vuforia.samples.ImageTargets.VuforiaSampleGLView;

import java.util.Vector;

/**
 * Created by joelv on 11/19/2016.
 */
@Autonomous (name = "CameraClass_mod", group = "10771")
public class CameraClass extends LinearOpMode {

    //Added own code
    private Activity FTC;

    //region Focus mode constants:
    private static final int FOCUS_MODE_NORMAL = 0;
    private static final int FOCUS_MODE_CONTINUOUS_AUTO = 1;
    //endregion

    //region Application status constants:
    private static final int APPSTATUS_UNINITED = -1;
    private static final int APPSTATUS_INIT_APP = 0;
    private static final int APPSTATUS_INIT_VUFORIA = 1;
    private static final int APPSTATUS_INIT_TRACKER = 2;
    private static final int APPSTATUS_INIT_APP_AR = 3;
    private static final int APPSTATUS_LOAD_TRACKER = 4;
    private static final int APPSTATUS_INITED = 5;
    private static final int APPSTATUS_CAMERA_STOPPED = 6;
    private static final int APPSTATUS_CAMERA_RUNNING = 7;
    //endregion

    //Name for library sample
    public static final String library = "ImageTargetsNative";

    // Constant representing invalid screen orientation to trigger a query:
    private static final int INVALID_SCREEN_ROTATION = -1;

    //region UI stuff
    // Our OpenGL view:
    private VuforiaSampleGLView mGlView;
    
    // Our renderer:
    private ImageTargetsRenderer mRenderer;

    //The root view of the activity
    private View rootView;

    // Display size of the device
    private int mScreenWidth = 0;
    private int mScreenHeight = 0;

    // Last detected screen rotation:
    private int mLastScreenRotation = INVALID_SCREEN_ROTATION;

    private LinearLayout container;

    boolean mIsDroidDevice = false;
    //endregion

    // Keeps track of the current camera
    //int mCurrentCamera = CAMERA_DIRECTION_DEFAULT;
    int mCurrentCamera = 0;

    // The current application status:
    private int mAppStatus = APPSTATUS_UNINITED;

    // The async tasks to initialize the Vuforia SDK:
    private InitVuforiaTask mInitVuforiaTask;
    private LoadTrackerTask mLoadTrackerTask;

    // See CameraClass.java for full comment
    private Object mShutdownLock = new Object();

    // Vuforia initialization flags:
    private int mVuforiaFlags = 0;

    // The textures we will use for rendering:
    private Vector<Texture> mTextures;

    // Contextual Menu Options for Camera Flash - Autofocus
    private boolean mContAutofocus = false;
    private boolean mExtendedTracking = false;
    
    static{
        System.loadLibrary("Vuforia");
        System.loadLibrary(library);
    }

    //region native functions
    public native int initTracker();
    public native void deinitTracker();
    public native int loadTrackerData();
    public native void destroyTrackerData();
    /** Native sample initialization. */
    public native void onVuforiaInitializedNative();
    public native void startCamera(int camera);
    public native void stopCamera();
    /** Native method for starting / stopping off target tracking */
    private native boolean startExtendedTracking();
    private native boolean stopExtendedTracking();

    public native void deinitApplicationNative();

    /** Tells native code whether we are in portait or landscape mode */
    private native void setActivityPortraitMode(boolean isPortrait);

    /** Tells native code to switch dataset as soon as possible */
    private native void switchDatasetAsap(int datasetId);
    private native boolean autofocus();
    private native boolean setFocusMode(int mode);

    public native void initApplicationNative(int width, int height);
    //endregion

    /** An async task to initialize Vuforia asynchronously. */
    private class InitVuforiaTask extends AsyncTask<Void, Integer, Boolean>
    {
        // Initialize with invalid value:
        private int mProgressValue = -1;


        protected Boolean doInBackground(Void... params)
        {
            // Prevent the cameraStop() method to overlap with initialization:
            synchronized (mShutdownLock)
            {
                //IMPORTANT
                Vuforia.setInitParameters(FTC, mVuforiaFlags, "AVj2kiX/////AAAAGV0J5W5oOkUTvP1+IKxrWdIpD63oQV8zSY/+qSNDxkt5zj8tW0N9AK7/3yUJRBlnJx80gStuZcHF7JoiKUNj4JmO6gcyIQn2LWZ/0hL9gFM+PmwM6lvzJu9U/gmvf++GngzR74ft0gjlNPle9qDHEaAgMHYcbEDpc4msHDVn6ZjCcxDem2tQyW4gEY334fwAU9E0ySkw1KwC/Mo6gaE7bW1Mh9xLbXYTe2+sRclEA6YbrKeH8LHmJBDXQxTdcL4HyS26oPYAGRXfFLoi7QkBdkPDYKiPQUsCoHhNz1uhPh5duEdwOD9Sm6YUPZYet7Mo9QP3sxaDlaqY5l2pHYn/tH31Xu9eqLKe2RmNRzgMNaJ9\n");
                //Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_OBJECT_TARGETS, 4);
                //Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

                do
                {
                    // See ImageTargets.java for details
                    mProgressValue = Vuforia.init();

                    // Publish the progress value:
                    publishProgress(mProgressValue);

                    // We check whether the task has been canceled in the
                    // meantime (by calling AsyncTask.cancel(true)).
                    // and bail out if it has, thus stopping this thread.
                    // This is necessary as the AsyncTask will run to completion
                    // regardless of the status of the component that
                    // started is.
                } while (!isCancelled() && mProgressValue >= 0
                        && mProgressValue < 100);

                return (mProgressValue > 0);
            }
        }

        //UNUSED
        protected void onProgressUpdate(Integer... values)
        {
            // Do something with the progress value "values[0]", e.g. update
            // splash screen, progress bar, etc.
        }


        protected void onPostExecute(Boolean result)
        {
            // Done initializing Vuforia, proceed to next application
            // initialization status:
            if (result)
            {
                DebugLog.LOGD("InitVuforiaTask::onPostExecute: Vuforia "
                        + "initialization successful");

                updateApplicationStatus(APPSTATUS_INIT_TRACKER);
            } else
            {
                // Create dialog box for display error:
                AlertDialog dialogError = new AlertDialog.Builder(
                        FTC).create();

                dialogError.setButton(DialogInterface.BUTTON_POSITIVE, "OK",
                        new DialogInterface.OnClickListener()
                        {
                            public void onClick(DialogInterface dialog, int which)
                            {
                                // Exiting application:
                                System.exit(1);
                            }
                        });

                String logMessage;

                // See ImageTargets.java for more
                dialogError.show();
            }
        }
    }

    //ImageTargets.java line 265 optional code

    /** An async task to load the tracker data asynchronously. */
    private class LoadTrackerTask extends AsyncTask<Void, Integer, Boolean>
    {
        protected Boolean doInBackground(Void... params)
        {
            // Prevent the cameraStop() method to overlap:
            synchronized (mShutdownLock)
            {
                // Load the tracker data set:
                return (loadTrackerData() > 0);
            }
        }


        protected void onPostExecute(Boolean result)
        {
            DebugLog.LOGD("LoadTrackerTask::onPostExecute: execution "
                    + (result ? "successful" : "failed"));

            if (result)
            {
                // Done loading the tracker, update application status:
                updateApplicationStatus(APPSTATUS_INITED);
            } else
            {
                // Create dialog box for display error:
                AlertDialog dialogError = new AlertDialog.Builder(
                        FTC).create();

                dialogError.setButton(DialogInterface.BUTTON_POSITIVE, "Close",
                        new DialogInterface.OnClickListener()
                        {
                            public void onClick(DialogInterface dialog, int which)
                            {
                                // Exiting application:
                                System.exit(1);
                            }
                        });

                // Show dialog box with error message:
                dialogError.setMessage("Failed to load tracker data.");
                dialogError.show();
            }
        }
    }

    /** Stores screen dimensions */
    private void storeScreenDimensions()
    {
        if (container != null) {
            mScreenWidth = container.getWidth();
            mScreenHeight = container.getHeight();
        } else {
            mScreenHeight = mScreenWidth = 0;
        }
    }

//region init, start, pause, stop
    /**
     * Called when the activity first starts or the user navigates back to an
     * activity.
     */
    public void initCamera()
    {
        DebugLog.LOGD("initCamera");

        // Load any sample specific textures:
        mTextures = new Vector<Texture>();
        loadTextures();

        // Configure Vuforia to use OpenGL ES 2.0
        mVuforiaFlags = Vuforia.GL_20;

        // Update the application status to start initializing application:
        updateApplicationStatus(APPSTATUS_INIT_APP);

        mIsDroidDevice = android.os.Build.MODEL.toLowerCase().startsWith(
                "droid");

    }

    /** Called when the activity will start interacting with the user. */
    protected void startCamera()
    {
        DebugLog.LOGD("startCamera");

        // This is needed for some Droid devices to force portrait
        if (mIsDroidDevice)
        {
            FTC.setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
            FTC.setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);
        }

        // Vuforia-specific resume operation
        Vuforia.onResume();

        // We may start the camera only if the Vuforia SDK has already been
        // initialized
        if (mAppStatus == APPSTATUS_CAMERA_STOPPED)
        {
            updateApplicationStatus(APPSTATUS_CAMERA_RUNNING);
        }

        // Resume the GL view:
        if (mGlView != null)
        {
            mGlView.setVisibility(View.VISIBLE);
            mGlView.onResume();
        }

    }

    /** Called when the system is about to start resuming a previous activity. */
    protected void pauseCamera() {
        DebugLog.LOGD("pauseCamera");

        if (mGlView != null) {
            mGlView.setVisibility(View.INVISIBLE);
            mGlView.onPause();
        }

        if (mAppStatus == APPSTATUS_CAMERA_RUNNING) {
            updateApplicationStatus(APPSTATUS_CAMERA_STOPPED);
        }

        // Vuforia-specific pause operation
        Vuforia.onPause();
    }

    /** stop vuforia */
    protected void cameraStop()
    {
        DebugLog.LOGD("cameraStop");

        // Cancel potentially running tasks
        if (mInitVuforiaTask != null
                && mInitVuforiaTask.getStatus() != CameraClass.InitVuforiaTask.Status.FINISHED)
        {
            mInitVuforiaTask.cancel(true);
            mInitVuforiaTask = null;
        }

        if (mLoadTrackerTask != null
                && mLoadTrackerTask.getStatus() != CameraClass.LoadTrackerTask.Status.FINISHED)
        {
            mLoadTrackerTask.cancel(true);
            mLoadTrackerTask = null;
        }

        // Ensure that all asynchronous operations to initialize Vuforia
        // and loading the tracker datasets do not overlap:
        synchronized (mShutdownLock)
        {

            // Do application deinitialization in native code:
            deinitApplicationNative();

            // Unload texture:
            mTextures.clear();
            mTextures = null;

            // Destroy the tracking data set:
            destroyTrackerData();

            // Deinit the tracker:
            deinitTracker();

            // Deinitialize Vuforia SDK:
            Vuforia.deinit();
        }

        System.gc();
    }
//endregion

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

        /**
         * Updates projection matrix and viewport after a screen rotation change was
         * detected.
         */
    public void updateRenderView()
    {
        int currentScreenRotation = FTC.getWindowManager().getDefaultDisplay()
                .getRotation();
        if (currentScreenRotation != mLastScreenRotation)
        {
            // Set projection matrix if there is already a valid one:
            if (Vuforia.isInitialized()
                    && (mAppStatus == APPSTATUS_CAMERA_RUNNING))
            {
                DebugLog.LOGD("updateRenderView");

                // Query display dimensions:
                storeScreenDimensions();

                // Update viewport via renderer:
                mRenderer.updateRendering(mScreenWidth, mScreenHeight);

                // Cache last rotation used for setting projection matrix:
                mLastScreenRotation = currentScreenRotation;
            }
        }
    }

    /**
     * NOTE: this method is synchronized because of a potential concurrent
     * access by onResume() and InitVuforiaTask.onPostExecute().
     */
    private synchronized void updateApplicationStatus(int appStatus)
    {
        // Exit if there is no change in status:
        if (mAppStatus == appStatus)
            return;

        // Store new status value:
        mAppStatus = appStatus;

        // Execute application state-specific actions:
        switch (mAppStatus)
        {
            case APPSTATUS_INIT_APP:
                // Initialize application elements that do not rely on Vuforia
                // initialization:
                initApplication();

                // Proceed to next application initialization status:
                updateApplicationStatus(APPSTATUS_INIT_VUFORIA);
                break;

            case APPSTATUS_INIT_VUFORIA:
                // Initialize Vuforia SDK asynchronously to avoid blocking the
                // main (UI) thread.
                //
                // NOTE: This task instance must be created and invoked on the
                // UI thread and it can be executed only once!
                try
                {
                    mInitVuforiaTask = new CameraClass.InitVuforiaTask();
                    mInitVuforiaTask.execute();
                } catch (Exception e)
                {
                    DebugLog.LOGE("Initializing Vuforia SDK failed");
                }
                break;

            case APPSTATUS_INIT_TRACKER:
                // Initialize the ObjectTracker:
                if (initTracker() > 0)
                {
                    // Proceed to next application initialization status:
                    updateApplicationStatus(APPSTATUS_INIT_APP_AR);
                }
                break;

            case APPSTATUS_INIT_APP_AR:
                // Initialize Augmented Reality-specific application elements
                // that may rely on the fact that the Vuforia SDK has been
                // already initialized:
                initApplicationAR();

                // Proceed to next application initialization status:
                updateApplicationStatus(APPSTATUS_LOAD_TRACKER);
                break;

            case APPSTATUS_LOAD_TRACKER:
                // Load the tracking data set:
                //
                // NOTE: This task instance must be created and invoked on the
                // UI thread and it can be executed only once!
                try
                {
                    mLoadTrackerTask = new CameraClass.LoadTrackerTask();
                    mLoadTrackerTask.execute();
                } catch (Exception e)
                {
                    DebugLog.LOGE("Loading tracking data set failed");
                }
                break;

            case APPSTATUS_INITED:
                // Hint to the virtual machine that it would be a good time to
                // run the garbage collector:
                //
                // NOTE: This is only a hint. There is no guarantee that the
                // garbage collector will actually be run.
                System.gc();

                // Native post initialization:
                onVuforiaInitializedNative();

                // Activate the renderer:
                mRenderer.mIsActive = true;

                // Now add the GL surface view. It is important
                // that the OpenGL ES surface view gets added
                // BEFORE the camera is started and video
                // background is configured.
                container.addView(mGlView, new ViewGroup.LayoutParams(
                        ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT));

                // Sets the UILayout to be drawn in front of the camera
                container.bringToFront();

                // Start the camera:
                updateApplicationStatus(APPSTATUS_CAMERA_RUNNING);

                break;

            case APPSTATUS_CAMERA_STOPPED:
                // Call the native function to stop the camera:
                stopCamera();
                break;

            case APPSTATUS_CAMERA_RUNNING:
                // Call the native function to start the camera:
                startCamera(mCurrentCamera);

                // Sets the layout background to transparent
                container.setBackgroundColor(Color.TRANSPARENT);

                // Set continuous auto-focus if supported by the device,
                // otherwise default back to regular auto-focus mode.
                // This will be activated by a tap to the screen in this
                // application.
                boolean result = setFocusMode(FOCUS_MODE_CONTINUOUS_AUTO);
                if (!result)
                {
                    DebugLog.LOGE("Unable to enable continuous autofocus");
                    mContAutofocus = false;
                    setFocusMode(FOCUS_MODE_NORMAL);
                } else
                {
                    mContAutofocus = true;
                }

                break;

            default:
                throw new RuntimeException("Invalid application state");
        }
    }

    /** Initialize application GUI elements that are not related to AR. */
    private void initApplication()
    {
        setActivityPortraitMode(true);

        // Query display dimensions:
        storeScreenDimensions();
    }

    /** Initializes AR application components. */
    private void initApplicationAR()
    {
        // Do application initialization in native code (e.g. registering
        // callbacks, etc.):
        initApplicationNative(mScreenWidth, mScreenHeight);

        // Create OpenGL ES view:
        int depthSize = 16;
        int stencilSize = 0;
        boolean translucent = Vuforia.requiresAlpha();

        mGlView = new VuforiaSampleGLView(FTC);
        mGlView.init(translucent, depthSize, stencilSize);

        mRenderer = new ImageTargetsRenderer();
        mRenderer.mActivity = this;
        mGlView.setRenderer(mRenderer);

        container = (LinearLayout) rootView.findViewById(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);

        container.setVisibility(View.VISIBLE);
        container.setBackgroundColor(Color.BLACK);
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

    @Override
    public void runOpMode() throws InterruptedException{
        //properly assign the activity
        FTC = ((Activity)hardwareMap.appContext);
        rootView = FTC.getWindow().getDecorView().findViewById(android.R.id.content);
        mTextures = new Vector<>();
        loadTextures();
        mVuforiaFlags = Vuforia.GL_20;

        // Update the application status to start initializing application:
        initApplication();
        try
        {
            mInitVuforiaTask = new CameraClass.InitVuforiaTask();
            mInitVuforiaTask.execute();
        } catch (Exception e)
        {
            DebugLog.LOGE("Initializing Vuforia SDK failed");
        }
        mIsDroidDevice = android.os.Build.MODEL.toLowerCase().startsWith(
                "droid");
        waitForStart();
        DebugLog.LOGD("startCamera");

        // This is needed for some Droid devices to force portrait
        if (mIsDroidDevice)
        {
            FTC.setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
            FTC.setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);
        }

        // Vuforia-specific resume operation
        Vuforia.onResume();

        // We may start the camera only if the Vuforia SDK has already been
        // initialized
        startCamera(mCurrentCamera);

        // Sets the layout background to transparent
        container.setBackgroundColor(Color.TRANSPARENT);

        // Set continuous auto-focus if supported by the device,
        // otherwise default back to regular auto-focus mode.
        // This will be activated by a tap to the screen in this
        // application.
        boolean result = setFocusMode(FOCUS_MODE_CONTINUOUS_AUTO);
        if (!result)
        {
            DebugLog.LOGE("Unable to enable continuous autofocus");
            mContAutofocus = false;
            setFocusMode(FOCUS_MODE_NORMAL);
        } else
        {
            mContAutofocus = true;
        }

        // Resume the GL view:
        if (mGlView != null)
        {
            mGlView.setVisibility(View.VISIBLE);
            mGlView.onResume();
        }
        while(opModeIsActive()){

        }
        DebugLog.LOGD("pauseCamera");

        if (mGlView != null) {
            mGlView.setVisibility(View.INVISIBLE);
            mGlView.onPause();
        }
        stopCamera();
        // Vuforia-specific pause operation
        Vuforia.onPause();
        if (mInitVuforiaTask != null
                && mInitVuforiaTask.getStatus() != CameraClass.InitVuforiaTask.Status.FINISHED)
        {
            mInitVuforiaTask.cancel(true);
            mInitVuforiaTask = null;
        }
        if (mLoadTrackerTask != null
                && mLoadTrackerTask.getStatus() != CameraClass.LoadTrackerTask.Status.FINISHED)
        {
            mLoadTrackerTask.cancel(true);
            mLoadTrackerTask = null;
        }
        mTextures.clear();
        mTextures = null;
        // Destroy the tracking data set:
        destroyTrackerData();
        // Deinit the tracker:
        deinitTracker();
        // Deinitialize Vuforia SDK:
        Vuforia.deinit();
        System.gc();
    }
}

