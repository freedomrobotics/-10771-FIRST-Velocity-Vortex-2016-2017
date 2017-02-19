package org.fhs.robotics.ftcteam10771.lepamplemousse.modes.tests;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.hardware.ams.AMSColorSensorImpl;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

/**
 * Created by Freedom Robotics on 2/18/2017.
 */
@Autonomous(name="test")
@Disabled
public class color extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        AMSColorSensor colorSensor = (AMSColorSensor) hardwareMap.colorSensor.get("left_rgb");
        colorSensor.setI2cAddress(new I2cAddr(0x29));
        AMSColorSensor colorSensor2 = (AMSColorSensor) hardwareMap.colorSensor.get("right_rgb");
        colorSensor2.setI2cAddress(new I2cAddr(0x30));
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("left", colorSensor.argb());
            telemetry.addData("right", colorSensor2.argb());
            telemetry.update();
        }
    }
}
