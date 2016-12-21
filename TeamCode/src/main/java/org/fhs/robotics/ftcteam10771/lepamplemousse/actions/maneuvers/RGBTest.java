package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by 84ven on 12/21/2016.
 */

public class RGBTest extends LinearOpMode {
    ColorSensor leftSensor;
    ColorSensor rightSensor;
    Adafruit colorHandler;
    Integer[] test = new Integer[3];
    @Override
    public void runOpMode() throws InterruptedException {
        leftSensor = hardwareMap.colorSensor.get("leftRGB");
        //rightSensor = hardwareMap.colorSensor.get("rightRGB");
        colorHandler = new Adafruit(leftSensor);
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.y){
                test[0] = colorHandler.red();
                test[1] = colorHandler.blue();
                test[2] = colorHandler.green();
                telemetry.addData("RedStatus", test[0]);
                telemetry.addData("BlueStatus", test[1]);
                telemetry.addData("GreenStatus", test[2]);
            }
            else telemetry.addData("Null", "null");
            telemetry.update();
        }
    }
}
