package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers;

import android.app.Activity;
import android.content.pm.ActivityInfo;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.View;
import android.widget.LinearLayout;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera.CameraPreview;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;

import static org.opencv.imgcodecs.Imgcodecs.CV_LOAD_IMAGE_GRAYSCALE;
import static org.opencv.imgcodecs.Imgcodecs.imread;

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
        final String TAG = "Camera";
        final CameraBridgeViewBase preview = (CameraBridgeViewBase)((Activity)hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.JavaCamera);
        Handler h = new Handler(Looper.getMainLooper());
        h.post(new Runnable(){
            @Override
            public void run() {
                /*
                CameraPreview preview = new CameraPreview(hardwareMap.appContext, camera);
                LinearLayout container = (LinearLayout) ((Activity)hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
                camera.setDisplayOrientation(90);
                container.addView(preview);
                container.setVisibility(View.VISIBLE);
                */
                BaseLoaderCallback mLoaderCallBack = new BaseLoaderCallback(hardwareMap.appContext) {
                    @Override
                    public void onManagerConnected(int status) {
                        switch (status) {
                            case LoaderCallbackInterface.SUCCESS:
                            {
                                Log.i(TAG, "OpenCV loaded successfully");
                                preview.enableView();
                            } break;
                            default:
                            {
                                super.onManagerConnected(status);
                            } break;
                        }
                    }
                };
                OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_1_0, hardwareMap.appContext, mLoaderCallBack);
                preview.setVisibility(View.VISIBLE);
                ((Activity)hardwareMap.appContext).setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
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
                preview.disableView();
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