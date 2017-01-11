package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;

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
        int x = 0;
        while (opModeIsActive()) {
            testSensor.enable(x%2 == 0);
            for (int i = 0; i<100; i++) {
                double in = testSensor.distance();
                telemetry.addData("in", in);
                telemetry.update();
            }
            x++;
        }
    }
}
