package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers;

import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers.CameraVision;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.samples.ImageTargets.ImageTargets;

/**
 * Created by joelv on 11/19/2016.
 */

@Autonomous(name = "CameraC", group = "10771")
public class CameraClass extends LinearOpMode {

    public static final String library = "ImageTargetsNative";
    static{
        System.loadLibrary("Vuforia");
        System.loadLibrary(library);
    }

    public native void startCamera();
    public native void stopCamera();
    public native void updateRendering();
    public native void initRendering();
    public native void initTracker();
    public native void deinitTracker();
    public native void loadTrackerData();
    public native void onVuforiaInitializedNative();
    public native void renderFrame();
    public native void initApplicationNative();
    public native void deinitApplicationNative();


    @Override
    public void runOpMode() throws InterruptedException {
        initApplicationNative();
        startCamera();
        initRendering();
        while (opModeIsActive()){
            updateRendering();
        }
        stopCamera();
        deinitApplicationNative();
    }
}

