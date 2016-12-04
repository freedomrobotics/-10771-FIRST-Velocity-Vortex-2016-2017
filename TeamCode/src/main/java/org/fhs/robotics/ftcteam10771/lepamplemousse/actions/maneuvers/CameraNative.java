package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers;

import android.app.Activity;
import android.content.pm.ActivityInfo;
import android.hardware.camera2.CameraDevice;
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Vuforia;
import com.vuforia.samples.ImageTargets.DebugLog;
import com.vuforia.samples.ImageTargets.ImageTargets;
import com.vuforia.samples.ImageTargets.ImageTargetsRenderer;
import com.vuforia.samples.ImageTargets.Texture;
import com.vuforia.samples.ImageTargets.VuforiaSampleGLView;
import com.vuforia.samples.ImageTargets.ui.SampleAppMenu.SampleAppMenu;

import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera.Camera;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera.CameraObject;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera.CameraPreview;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.util.Vector;

/**
 * Created by joelv on 11/23/2016.
 */


@Autonomous (name = "CameraNative", group = "10771")
public class CameraNative extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        //Camera.createCameraObject((Activity)hardwareMap.appContext, hardwareMap.appContext, CameraObject.Downsample.FULL);
        //CameraObject.CameraPreview cameraPreview = new CameraObject.CameraPreview()
        //Activity FTC = ((Activity)hardwareMap.appContext);
        //View rootView = FTC.getWindow().getDecorView().findViewById(android.R.id.content);
        final android.hardware.Camera camera = getCameraInstance();
        Handler h = new Handler(Looper.getMainLooper());
        h.post(new Runnable(){
            @Override
            public void run() {
                CameraPreview preview = new CameraPreview(hardwareMap.appContext, camera);
                LinearLayout container = (LinearLayout) ((Activity)hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
                camera.setDisplayOrientation(90);
                container.addView(preview);
                container.setVisibility(View.VISIBLE);
            }

        });
        waitForStart();
        //((Activity)hardwareMap.appContext).setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
        //((Activity)hardwareMap.appContext).setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);
        while(opModeIsActive()){

        }
        h.post(new Runnable(){
            @Override
            public void run() {
                camera.stopPreview();
                camera.release();
                //camera = null;
            }
        });

    }

    public static android.hardware.Camera getCameraInstance(){
        android.hardware.Camera c = null;
        try{
            c = android.hardware.Camera.open();
        }
        catch (Exception e){
            //Camera doesn't exist
        }
        return c;
    }

    public static android.hardware.Camera release(android.hardware.Camera camera){
        camera.release();
        camera = null;
        return camera;
    }
}