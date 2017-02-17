package org.fhs.robotics.ftcteam10771.lepamplemousse.modes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera.Camera;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera.CameraVision;

/**
 * Created by joelv on 2/16/2017.
 */
@Autonomous(name="Camera Switch Test")
public class CameraTest extends OpMode{

    CameraVision cameraVision;
    boolean init = false;
    int imageIndex = 0;

    @Override
    public void init() {
        cameraVision = new CameraVision(false);
        cameraVision.setAutoTarget(false);
    }

    @Override
    public void loop() {

        if (gamepad1.a){
            init = !init;
        }

        if (gamepad1.b){
            if (imageIndex==3){
                imageIndex = 0;
            } else imageIndex++;
            cameraVision.setTargetImage(CameraVision.Image.getImage(imageIndex));
        }

        if (init){
            cameraVision.vuforiaInit();
        }
        else cameraVision.vuforiaDeinit();

        telemetry.addData("Target Image", cameraVision.target().getName());
        telemetry.addData("Z", cameraVision.getZ());
        telemetry.addData("X", cameraVision.getX());
        telemetry.addData("Angle", cameraVision.getAngleToTurn());
        telemetry.update();

    }
}
