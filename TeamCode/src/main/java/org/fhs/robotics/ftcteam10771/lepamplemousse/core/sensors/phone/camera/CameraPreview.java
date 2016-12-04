package org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera;

import android.content.Context;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

import java.io.IOException;

/**
 * Created by 84ven on 12/4/2016.
 */

public class CameraPreview extends SurfaceView implements SurfaceHolder.Callback{

    private SurfaceHolder mHolder;
    private android.hardware.Camera mCamera;
    private String TAG = "Camera: ";

    public CameraPreview(Context context, android.hardware.Camera camera) {
        super(context);
        mCamera = camera;
        mHolder = getHolder();
        mHolder.addCallback(this);
        mHolder.setType(SurfaceHolder.SURFACE_TYPE_PUSH_BUFFERS);
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {
        try{
            mCamera.setPreviewDisplay(holder);
            mCamera.startPreview();
        }
        catch (IOException e){
            Log.d(TAG, "Error setting preview of Camera");
        }
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
        //Code for when preview is changed

        if (mHolder.getSurface()==null){
            return;
        }
        try{
            mCamera.stopPreview();
        }catch (Exception e){
            //Ignore
        }
        try{
            mCamera.setPreviewDisplay(mHolder);
            mCamera.startPreview();
        }catch(Exception e){
            Log.d(TAG, "Error starting camera again" + e.getMessage());
        }
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {

    }
}
