package org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera;

import android.app.Activity;
import android.content.Context;

import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.ReturnValues;

/**
 * Class for all camera function. Extend this.
 */
public class Camera{

    protected static final int cameraFullX = 1280;
    protected static final int cameraFullY = 960;

    public static CameraObject cameraObject = null;
    private static int counterCamera = 0;

    /**
     * Creates the camera object and opens it. Refer to {@link CameraObject#CameraObject(Activity,
     * CameraObject.Downsample) CameraObject(Activity, CameraObject.Downsample)} and
     * {@link CameraObject#initCamera()}.
     * <p/>
     * Will only allow a single instance of the camera and restart it with the lower downsampling
     * ratio. Contains a counter that increases on existance.
     *
     * @return A {@link ReturnValues} value for the camera object status.
     * <li><b>ReturnValues.SUCCESS:</b> The camera was successfully initialized and opened. Counter
     * is increased.
     * <li><b>ReturnValues.CAMERA_ALREADY_OPEN:</b> The camera was already opened for whatever reason.
     * If this is returned, an existing object already has the camera.
     * <li><b>ReturnValues.CAMERA_FAILED_TO_OPEN:</b> The camera failed to open for whatever reason.
     * <li><b>ReturnValues.CAMERA_DOESNT_EXIST:</b> This device does not have a camera.
     * <li><b>ReturnValues.CAMERA_OBJECT_EXISTS:</b> The camera object has already been created.
     * Counter is increased.
     */
    public static ReturnValues createCameraObject(Activity activity, CameraObject.Downsample downSample){
        return createCameraObject(activity, activity, downSample);
    }

    /**
     * Creates the camera object and opens it. Refer to {@link CameraObject#CameraObject(Context,
     * CameraObject.Downsample) CameraObject(Context, CameraObject.Downsample)} and
     * {@link CameraObject#initCamera()}
     * <p/>
     * Will only allow a single instance of the camera and restart it with the lower downsampling
     * ratio. Contains a counter that increases on existance.
     *
     * @return A {@link ReturnValues} value for the camera object status.
     * <li><b>ReturnValues.SUCCESS:</b> The camera was successfully initialized and opened. Counter
     * is increased.
     * <li><b>ReturnValues.CAMERA_ALREADY_OPEN:</b> The camera was already opened for whatever reason.
     * If this is returned, an existing object already has the camera.
     * <li><b>ReturnValues.CAMERA_FAILED_TO_OPEN:</b> The camera failed to open for whatever reason.
     * <li><b>ReturnValues.CAMERA_DOESNT_EXIST:</b> This device does not have a camera.
     * <li><b>ReturnValues.CAMERA_OBJECT_EXISTS:</b> The camera object has already been created.
     * Counter is increased.
     */
    public static ReturnValues createCameraObject(Context context, CameraObject.Downsample downSample){
        return createCameraObject((Activity) context, context, downSample);
    }

    /**
     * Creates the camera object and opens it. Refer to {@link CameraObject#CameraObject(Activity,
     * Context, CameraObject.Downsample) CameraObject(Activity, Context, CameraObject.Downsample)}
     * and {@link CameraObject#initCamera()}.
     * <p/>
     * Will only allow a single instance of the camera and restart it with the lower downsampling
     * ratio. Contains a counter that increases on existance.
     *
     * @return A {@link ReturnValues} value for the camera object status.
     * <li><b>ReturnValues.SUCCESS:</b> The camera was successfully initialized and opened. Counter
     * is increased.
     * <li><b>ReturnValues.CAMERA_ALREADY_OPEN:</b> The camera was already opened for whatever reason.
     * If this is returned, an existing object already has the camera.
     * <li><b>ReturnValues.CAMERA_FAILED_TO_OPEN:</b> The camera failed to open for whatever reason.
     * <li><b>ReturnValues.CAMERA_DOESNT_EXIST:</b> This device does not have a camera.
     * <li><b>ReturnValues.CAMERA_OBJECT_EXISTS:</b> The camera object has already been created.
     * Counter is increased. Downsampling changed if lower.
     */
    public static ReturnValues createCameraObject(Activity activity, Context context, CameraObject.Downsample downSample){
        if (cameraObject != null){
            counterCamera++;
            if (downSample.getValue() < cameraObject.cameraData.getDownSample().getValue()){
                cameraObject.stopCamera();
                cameraObject = new CameraObject(activity, context, downSample);
                cameraObject.initCamera();
            }
            return ReturnValues.CAMERA_OBJECT_EXISTS;
        }
        cameraObject = new CameraObject(activity, context, downSample);
        ReturnValues r = cameraObject.initCamera();
        if (r == ReturnValues.SUCCESS){
            counterCamera++;
        }
        return r;
    }

    /**
     * Safely closes the camera for each instance of the class. Must be called however many times
     * createCameraObject was called.
     */
    public static void stopCamera(){
        counterCamera--;
        if (counterCamera == 0){
            cameraObject.stopCamera();
            cameraObject = null;
        }
        if (counterCamera < 0){
            counterCamera = 0;
        }
    }

    /**
     * @return The camera object (see {@link CameraObject}).
     */
    public CameraObject getCameraObject(){
        return cameraObject;
    }
}
