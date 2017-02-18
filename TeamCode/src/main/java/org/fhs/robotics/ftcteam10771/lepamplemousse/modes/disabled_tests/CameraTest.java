package org.fhs.robotics.ftcteam10771.lepamplemousse.modes.disabled_tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Controllers;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera.Camera;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera.CameraVision;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.Static;

/**
 * Created by joelv on 2/16/2017.
 */
@Disabled
@Autonomous(name="Camera Switch Test")
public class CameraTest extends OpMode{
    private Controllers controls;

    CameraVision cameraVision;
    boolean init = false;
    boolean switchImage = false;
    int imageIndex = 0;

    @Override
    public void init() {
        Config keymapping = new Config(Static.configPath, Static.configControlFileName + Static.configFileSufffix, telemetry, "keymapping");

        // should catch other errors, but oh well
        // add data logging
        if (keymapping.read() == Config.State.DEFAULT_EXISTS) {
            keymapping.create(true);
            //read without creating file if it still fails.
            if (keymapping.read() == Config.State.DEFAULT_EXISTS)
                keymapping.read(true);
        }

        controls = new Controllers(gamepad1, gamepad2, keymapping);
        controls.initialize();
        cameraVision = new CameraVision(false);
        cameraVision.setAutoTarget(false);
    }

    @Override
    public void loop() {

        init = controls.getToggle("launch");
        switchImage = controls.getToggle("image");

        if (switchImage){
            if (imageIndex==3){
                imageIndex = 0;
            } else imageIndex++;
            cameraVision.setTargetImage(CameraVision.Image.getImage(imageIndex));
        }
        switchImage = false;

        if (init){
            cameraVision.vuforiaInit();
            cameraVision.toggleVuforia(true);
        }
        else{
            cameraVision.toggleVuforia(false);
            cameraVision.vuforiaDeinit();
        }

        cameraVision.runImageTracking();

        if (cameraVision.target()!=null){
            telemetry.addData("Target Image", cameraVision.target().getName());
            telemetry.addData("Z", cameraVision.getZ());
            telemetry.addData("X", cameraVision.getX());
            telemetry.addData("Angle", cameraVision.getAngleToTurn());
            telemetry.update();
        }

    }
}
