package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by joelv on 1/11/2017.
 */
@Autonomous (name = "Range")
public class Breakout extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AnalogInput range = hardwareMap.analogInput.get("range");
        DigitalChannel rangeon = hardwareMap.digitalChannel.get("rangeon");
        UltrasonicRange testSensor = new UltrasonicRange(range, rangeon);
        waitForStart();
        testSensor.enable(true);
        while (opModeIsActive()) {
            double in = testSensor.distance();
            telemetry.addData("in", in);
            telemetry.update();
        }
    }
}
