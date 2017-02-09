package org.fhs.robotics.ftcteam10771.lepamplemousse.actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by joelv on 2/8/2017.
 */
@Autonomous (name="Light Sensor Test")
public class LightSensorTest extends LinearOpMode {

    OpticalDistanceSensor sensor;

    @Override
    public void runOpMode() throws InterruptedException {
        sensor = hardwareMap.opticalDistanceSensor.get("ods");
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Light", sensor.getLightDetected());
            telemetry.addData("Raw Light", sensor.getRawLightDetected());
            telemetry.update();
        }
    }
}
