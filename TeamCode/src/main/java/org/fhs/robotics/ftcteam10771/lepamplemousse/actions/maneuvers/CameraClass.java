package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers;

import android.util.DisplayMetrics;

import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers.CameraVision;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.samples.ImageTargets.ImageTargets;
import com.vuforia.samples.ImageTargets.Texture;

import java.util.Vector;

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
    DisplayMetrics metrics = new DisplayMetrics();
    private int mScreenWidth = 0;
    private int mScreenHeight = 0;

    /** Returns the texture object at the specified index. */
    public Texture getTexture(int i)
    {
        return mTextures.elementAt(i);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        mTextures = new Vector<Texture>();
        loadTextures();
        mScreenWidth = metrics.widthPixels;
        mScreenHeight = metrics.heightPixels;
        initApplicationNative(mScreenWidth, mScreenHeight);
        startCamera(0);
        initRendering();

        waitForStart();

        while (opModeIsActive()){
            updateRendering(mScreenWidth, mScreenHeight);
        }
        stopCamera();
        deinitApplicationNative();
        // Unload texture:
        mTextures.clear();
        mTextures = null;
    }
}

