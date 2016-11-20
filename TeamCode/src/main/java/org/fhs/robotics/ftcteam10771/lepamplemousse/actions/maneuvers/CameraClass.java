package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers;

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


    /** Returns the texture object at the specified index. */
    public Texture getTexture(int i)
    {
        return mTextures.elementAt(i);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        mTextures = new Vector<Texture>();
        loadTextures();

        initApplicationNative();
        startCamera();
        initRendering();

        waitForStart();

        while (opModeIsActive()){
            updateRendering();
        }
        stopCamera();
        deinitApplicationNative();
        // Unload texture:
        mTextures.clear();
        mTextures = null;
    }
}

