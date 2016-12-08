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
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera.ColorBlobDetector;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.List;

import static org.opencv.imgcodecs.Imgcodecs.CV_LOAD_IMAGE_GRAYSCALE;
import static org.opencv.imgcodecs.Imgcodecs.imread;

/**
 * Created by joelv on 11/23/2016.
 */


@Autonomous (name = "CameraNative", group = "10771")
public class CameraNative extends LinearOpMode implements CameraBridgeViewBase.CvCameraViewListener2{

    private Mat mRgba;
    private Mat mHsv;
    private Mat mHsvFrame;
    private Mat filetered;
    private Mat result;
    private ColorBlobDetector mDetector;
    private Mat mSpectrum;
    private Scalar mBlobColorRgba;
    private Scalar mBlobColorHsv;
    private Size SPECTRUM_SIZE;
    private Scalar CONTOUR_COLOR;
    final String TAG = "Camera";


    @Override
    public void runOpMode() throws InterruptedException {
        //Camera.createCameraObject((Activity)hardwareMap.appContext, hardwareMap.appContext, CameraObject.Downsample.FULL);
        //CameraObject.CameraPreview cameraPreview = new CameraObject.CameraPreview()
        //Activity FTC = ((Activity)hardwareMap.appContext);
        //View rootView = FTC.getWindow().getDecorView().findViewById(android.R.id.content);
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
                Activity FTC = (Activity)hardwareMap.appContext;
                preview.setCvCameraViewListener(CameraNative.this);
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

    @Override
    public void onCameraViewStarted(int width, int height) {
        mRgba = new Mat(height, width, CvType.CV_8UC4);
        mDetector = new ColorBlobDetector();
        mSpectrum = new Mat();
        filetered = new Mat();
        result = new Mat();
        mBlobColorRgba = new Scalar(43,93,13);
        mBlobColorHsv = new Scalar(255);
        SPECTRUM_SIZE = new Size(200, 64);
        mHsvFrame = new Mat();
        mHsv = new Mat();
    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        mRgba = inputFrame.rgba();
        Imgproc.cvtColor(mRgba, mHsvFrame, Imgproc.COLOR_RGB2HSV_FULL);
        Mat colorLabel = mRgba.submat(4, 68, 4, 68);
        colorLabel.setTo(mBlobColorRgba);
        Imgproc.cvtColor(colorLabel, mHsv, Imgproc.COLOR_RGB2HSV_FULL);
        mBlobColorHsv = Core.sumElems(mHsv);
        int pointCount = 64 * 64;
        for (int i = 0; i < mBlobColorHsv.val.length; i++) {
            mBlobColorHsv.val[i] /= pointCount;
        }
        mDetector.setHsvColor(mBlobColorHsv);
        mDetector.process(mRgba);
        Core.inRange(mHsvFrame, mDetector.getLowerLimit(), mDetector.getUpperLimit(), result);
        //List<MatOfPoint> contours = mDetector.getContours();
        //Log.e(TAG, "Contours count: " + contours.size());
        //Imgproc.drawContours(mRgba, contours, -1, CONTOUR_COLOR);
        Mat spectrumLabel = mRgba.submat(4, 4 + mSpectrum.rows(), 70, 70 + mSpectrum.cols());
        mSpectrum.copyTo(spectrumLabel);
        filetered.setTo(new Scalar(0,0,0));
        mRgba.copyTo(filetered, result);
        return filetered;
    }

    @Override
    public void onCameraViewStopped() {
        mRgba.release();
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