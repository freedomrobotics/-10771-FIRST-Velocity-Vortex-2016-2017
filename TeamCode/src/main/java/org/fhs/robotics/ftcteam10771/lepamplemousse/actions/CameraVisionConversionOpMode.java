package org.fhs.robotics.ftcteam10771.lepamplemousse.actions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.UltrasonicRange;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera.CameraVision;

/**
 * Created by joelv on 1/20/2017.
 */

public class CameraVisionConversionOpMode extends LinearOpMode {

    UltrasonicRange range;
    CameraVision cameraVision;

    @Override
    public void runOpMode() throws InterruptedException {
        cameraVision = new CameraVision();
        //range = new UltrasonicRange(hardwareMap.analogInput.get("range"), hardwareMap.digitalChannel.get("switch"));

    }
}
