package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.LED;

/**
 * Created by 84ven on 12/21/2016.
 *
 * Op Mode testing the RGB color sensor
 *
 * Turns on the LED if it is pointing to the correct
 * side of the beacon
 *
 */
@Autonomous (name = "RGB")
public class RGBTest extends LinearOpMode {
    DeviceInterfaceModule cdim;
    LED leftLED;
    ColorSensor leftSensor;
    ColorSensor rightSensor;
    RGB colorHandler;
    Integer[] test = new Integer[3];
    @Override
    public void runOpMode() throws InterruptedException {
        leftSensor = hardwareMap.colorSensor.get("leftRGB");
        leftLED = hardwareMap.led.get("leftLED");
        //rightSensor = hardwareMap.colorSensor.get("rightRGB");
        colorHandler = new RGB(leftSensor);
        leftLED.enable(false);
        waitForStart();
        leftLED.enable(false);
        while (opModeIsActive()){
            if (leftSensor != null) {
                test[0] = colorHandler.red();
                test[1] = colorHandler.blue();
                test[2] = colorHandler.green();
                telemetry.addData("RedStatus", test[0]);
                telemetry.addData("BlueStatus", test[1]);
                telemetry.addData("GreenStatus", test[2]);
                telemetry.addData("Red", colorHandler.isSide(RGB.Color.RED));
            }
            else telemetry.addData("Color Sensor", "null");
            if (leftLED!=null){
                leftLED.enable(colorHandler.isSide(RGB.Color.RED));
            }
            else telemetry.addData("LED", "null");
            telemetry.update();
        }
    }
}
