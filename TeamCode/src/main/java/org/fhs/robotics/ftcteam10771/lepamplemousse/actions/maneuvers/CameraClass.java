package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.app.AlertDialog;
import android.content.DialogInterface;
import android.content.pm.ActivityInfo;
import android.content.res.Configuration;
import android.graphics.Color;
import android.os.AsyncTask;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.Message;
import android.util.DisplayMetrics;
import android.view.GestureDetector;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.view.WindowManager;
import android.widget.CheckBox;
import android.widget.FrameLayout;
import android.widget.LinearLayout;
import android.widget.RelativeLayout;
import android.widget.Switch;
import android.widget.Toast;

import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers.CameraVision;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;
import com.vuforia.samples.ImageTargets.DebugLog;
import com.vuforia.samples.ImageTargets.ImageTargets;
import com.vuforia.samples.ImageTargets.ImageTargetsRenderer;
import com.vuforia.samples.ImageTargets.Texture;
import com.vuforia.samples.ImageTargets.VuforiaSampleGLView;
import com.vuforia.samples.ImageTargets.ui.SampleAppMenu.SampleAppMenu;
import com.vuforia.samples.ImageTargets.DebugLog;
import com.vuforia.samples.ImageTargets.ImageTargets;
import com.vuforia.samples.ImageTargets.ui.SampleAppMenu.SampleAppMenuInterface;

import java.lang.ref.WeakReference;
import java.util.Vector;

/**
 * Created by joelv on 11/19/2016.
 */
@Autonomous (name = "Lala", group = "10771")
public class CameraClass extends LinearOpMode implements SampleAppMenuInterface {

    //Added own code
    private Activity FTC = ((Activity)hardwareMap.appContext);

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

    //Name for library sample
    public static final String library = "ImageTargetsNative";
    
    // Constants for Hiding/Showing Loading dialog
    static final int HIDE_LOADING_DIALOG = 0;
    static final int SHOW_LOADING_DIALOG = 1;

    private View mLoadingDialogContainer;
    
    // Our OpenGL view:
    private VuforiaSampleGLView mGlView;
    
    // Our renderer:
    private ImageTargetsRenderer mRenderer;
    
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

    // The async tasks to initialize the Vuforia SDK:
    private InitVuforiaTask mInitVuforiaTask;
    private LoadTrackerTask mLoadTrackerTask;

    // See CameraClass.java for full comment
    private Object mShutdownLock = new Object();

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

    private View mFlashOptionView;

    private RelativeLayout mUILayout;

    boolean mIsDroidDevice = false;
    
    static{
        System.loadLibrary("Vuforia");
        System.loadLibrary(library);
    }


    /**
     * Creates a handler to update the status of the Loading Dialog from an UI
     * Thread
     */
    static class LoadingDialogHandler extends Handler
    {
        private final WeakReference<CameraClass> mImageTargets;


        LoadingDialogHandler(CameraClass imageTargets)
        {
            mImageTargets = new WeakReference<CameraClass>(imageTargets);
        }


        public void handleMessage(Message msg)
        {
            CameraClass imageTargets = mImageTargets.get();
            if (imageTargets == null)
            {
                return;
            }

            if (msg.what == SHOW_LOADING_DIALOG)
            {
                imageTargets.mLoadingDialogContainer
                        .setVisibility(View.VISIBLE);

            } else if (msg.what == HIDE_LOADING_DIALOG)
            {
                imageTargets.mLoadingDialogContainer.setVisibility(View.GONE);
            }
        }
    }

    private Handler loadingDialogHandler = new CameraClass.LoadingDialogHandler(this);
    /** An async task to initialize Vuforia asynchronously. */

    private class InitVuforiaTask extends AsyncTask<Void, Integer, Boolean>
    {
        // Initialize with invalid value:
        private int mProgressValue = -1;


        protected Boolean doInBackground(Void... params)
        {
            // Prevent the onDestroy() method to overlap with initialization:
            synchronized (mShutdownLock)
            {
                //IMPORTANT
                Vuforia.setInitParameters(FTC, mVuforiaFlags, "AVj2kiX/////AAAAGV0J5W5oOkUTvP1+IKxrWdIpD63oQV8zSY/+qSNDxkt5zj8tW0N9AK7/3yUJRBlnJx80gStuZcHF7JoiKUNj4JmO6gcyIQn2LWZ/0hL9gFM+PmwM6lvzJu9U/gmvf++GngzR74ft0gjlNPle9qDHEaAgMHYcbEDpc4msHDVn6ZjCcxDem2tQyW4gEY334fwAU9E0ySkw1KwC/Mo6gaE7bW1Mh9xLbXYTe2+sRclEA6YbrKeH8LHmJBDXQxTdcL4HyS26oPYAGRXfFLoi7QkBdkPDYKiPQUsCoHhNz1uhPh5duEdwOD9Sm6YUPZYet7Mo9QP3sxaDlaqY5l2pHYn/tH31Xu9eqLKe2RmNRzgMNaJ9\n");

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
            // Prevent the onDestroy() method to overlap:
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
        // Query display dimensions:
        DisplayMetrics metrics = new DisplayMetrics();
        FTC.getWindowManager().getDefaultDisplay().getMetrics(metrics);
        mScreenWidth = mUILayout.getWidth();
        mScreenHeight = mUILayout.getHeight();
    }

    /**
     * Called when the activity first starts or the user navigates back to an
     * activity.
     */
    public void onCreate(Bundle savedInstanceState)
    {
        DebugLog.LOGD("onCreate");

        // Load any sample specific textures:
        mTextures = new Vector<Texture>();
        loadTextures();

        // Configure Vuforia to use OpenGL ES 2.0
        mVuforiaFlags = Vuforia.GL_20;

        // Creates the GestureDetector listener for processing double tap
        mGestureDetector = new GestureDetector(FTC, new GestureListener());

        // Update the application status to start initializing application:
        updateApplicationStatus(APPSTATUS_INIT_APP);

        mIsDroidDevice = android.os.Build.MODEL.toLowerCase().startsWith(
                "droid");

    }

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

    /** Called when the activity will start interacting with the user. */
    protected void onResume()
    {
        DebugLog.LOGD("onResume");

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

    /** Callback for configuration changes the activity handles itself */
    public void onConfigurationChanged(Configuration config)
    {
        DebugLog.LOGD("onConfigurationChanged");

        storeScreenDimensions();

        // Invalidate screen rotation to trigger query upon next render call:
        mLastScreenRotation = INVALID_SCREEN_ROTATION;
    }

    /** Called when the system is about to start resuming a previous activity. */
    protected void onPause()
    {
        DebugLog.LOGD("onPause");

        if (mGlView != null)
        {
            mGlView.setVisibility(View.INVISIBLE);
            mGlView.onPause();
        }

        // Turn off the flash
        if (mFlashOptionView != null && mFlash)
        {
            // OnCheckedChangeListener is called upon changing the checked state
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.JELLY_BEAN_MR1)
            {
                ((Switch) mFlashOptionView).setChecked(false);
            } else
            {
                ((CheckBox) mFlashOptionView).setChecked(false);
            }
        }

        if (mAppStatus == APPSTATUS_CAMERA_RUNNING)
        {
            updateApplicationStatus(APPSTATUS_CAMERA_STOPPED);
        }

        // Vuforia-specific pause operation
        Vuforia.onPause();
    }

    public native void deinitApplicationNative();

    /** The final call you receive before your activity is destroyed. */
    protected void onDestroy()
    {
        DebugLog.LOGD("onDestroy");

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
                FTC.addContentView(mGlView, new ViewGroup.LayoutParams(
                        ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT));

                // Sets the UILayout to be drawn in front of the camera
                mUILayout.bringToFront();

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

                // Hides the Loading Dialog
                loadingDialogHandler.sendEmptyMessage(HIDE_LOADING_DIALOG);

                // Sets the layout background to transparent
                mUILayout.setBackgroundColor(Color.TRANSPARENT);

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

                if( mSampleAppMenu == null)
                {
                    mSampleAppMenu = new SampleAppMenu(this, FTC, "Image Targets",
                            mGlView, mUILayout, null);
                    setSampleAppMenuSettings();
                }

                break;

            default:
                throw new RuntimeException("Invalid application state");
        }
    }

    /** Tells native code whether we are in portait or landscape mode */
    private native void setActivityPortraitMode(boolean isPortrait);

    /** Initialize application GUI elements that are not related to AR. */
    private void initApplication()
    {
        setActivityPortraitMode(true);

        // Query display dimensions:
        storeScreenDimensions();

        // As long as this window is visible to the user, keep the device's
        // screen turned on and bright:
        FTC.getWindow().setFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON,
                WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
    }

    public native void initApplicationNative(int width, int height);

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

        LayoutInflater inflater = LayoutInflater.from(FTC);
        /* mUILayout = (RelativeLayout) inflater.inflate(R.layout.camera_overlay,
            null, false); */
        mUILayout = (RelativeLayout) rootView.findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

        mUILayout.setVisibility(View.VISIBLE);
        mUILayout.setBackgroundColor(Color.BLACK);

        // Shows the loading indicator at start
        loadingDialogHandler.sendEmptyMessage(SHOW_LOADING_DIALOG);

        // Adds the inflated layout to the view
        FTC.addContentView(mUILayout, new ViewGroup.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.MATCH_PARENT));

    }

    /** Tells native code to switch dataset as soon as possible */
    private native void switchDatasetAsap(int datasetId);
    private native boolean autofocus();
    private native boolean setFocusMode(int mode);
    /** Activates the Flash */
    private native boolean activateFlash(boolean flash);


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

    public boolean onTouchEvent(MotionEvent event)
    {
        // Process the Gestures
        if (mSampleAppMenu != null && mSampleAppMenu.processEvent(event))
            return true;

        return mGestureDetector.onTouchEvent(event);
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

    // This method sets the menu's settings
    private void setSampleAppMenuSettings()
    {
        /* See ImageTargets.java for full code*/
    }

    public boolean menuProcess(int command)
    {

        boolean result = true;

        switch (command)
        {
            case CMD_BACK:
                FTC.finish();
                break;

            case CMD_FLASH:
                result = activateFlash(!mFlash);

                if (result)
                {
                    mFlash = !mFlash;
                } else
                {
                    /* showToast(getString(mFlash ? R.string.menu_flash_error_off
                        : R.string.menu_flash_error_on));
                    DebugLog
                        .LOGE(getString(mFlash ? R.string.menu_flash_error_off
                            : R.string.menu_flash_error_on)); */
                }
                break;

            case CMD_AUTOFOCUS:

                if (mContAutofocus)
                {
                    result = setFocusMode(FOCUS_MODE_NORMAL);

                    if (result)
                    {
                        mContAutofocus = false;
                    } else
                    {
                        /* showToast(getString(R.string.menu_contAutofocus_error_off));
                        DebugLog
                            .LOGE(getString(R.string.menu_contAutofocus_error_off)); */
                    }
                } else
                {
                    result = setFocusMode(FOCUS_MODE_CONTINUOUS_AUTO);

                    if (result)
                    {
                        mContAutofocus = true;
                    } else
                    {
                        /* showToast(getString(R.string.menu_contAutofocus_error_on));
                        DebugLog
                            .LOGE(getString(R.string.menu_contAutofocus_error_on));*/
                    }
                }

                break;

            case CMD_CAMERA_FRONT:
            case CMD_CAMERA_REAR:

                // Turn off the flash
                if (mFlashOptionView != null && mFlash)
                {
                    // OnCheckedChangeListener is called upon changing the checked state
                    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.JELLY_BEAN_MR1)
                    {
                        ((Switch) mFlashOptionView).setChecked(false);
                    } else
                    {
                        ((CheckBox) mFlashOptionView).setChecked(false);
                    }
                }

                if (command == CMD_CAMERA_FRONT)
                    mCurrentCamera = CAMERA_DIRECTION_FRONT;
                else
                    mCurrentCamera = CAMERA_DIRECTION_BACK;

                stopCamera();
                startCamera(mCurrentCamera);

                // Update rendering primitives
                mRenderer.updateRenderingPrimitives();

                break;

            case CMD_EXTENDED_TRACKING:
                if (mExtendedTracking)
                {
                    result = stopExtendedTracking();
                    if (!result)
                    {
                        showToast("Failed to stop extended tracking target");
                        DebugLog
                                .LOGE("Failed to stop extended tracking target");
                    }
                } else
                {
                    result = startExtendedTracking();
                    if (!result)
                    {
                        showToast("Failed to start extended tracking target");
                        DebugLog
                                .LOGE("Failed to start extended tracking target");
                    }
                }

                if (result)
                    mExtendedTracking = !mExtendedTracking;

                break;

            case CMD_DATASET_STONES_AND_CHIPS_DATASET:
                switchDatasetAsap(STONES_AND_CHIPS_DATASET_ID);
                break;

            case CMD_DATASET_TARMAC_DATASET:
                switchDatasetAsap(TARMAC_DATASET_ID);
                break;

        }

        return result;
    }

    private void showToast(String text)
    {
        Toast.makeText(FTC, text, Toast.LENGTH_SHORT).show();
    }

    @Override
    public void runOpMode() throws InterruptedException{
        onCreate(Bundle.EMPTY);
        waitForStart();
        onResume();
        while(opModeIsActive()){

        }
        onDestroy();
    }
}
