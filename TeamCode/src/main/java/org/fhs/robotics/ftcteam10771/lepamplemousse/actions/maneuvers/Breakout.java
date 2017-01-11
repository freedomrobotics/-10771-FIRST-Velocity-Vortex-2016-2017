package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
        
        double voltperin = 9.765625e-3;
        waitForStart();
        rangeon.setState(true);
        while (opModeIsActive()) {
            double in = range.getVoltage() / voltperin;
            telemetry.addData("in", in);
        }
    }
}
