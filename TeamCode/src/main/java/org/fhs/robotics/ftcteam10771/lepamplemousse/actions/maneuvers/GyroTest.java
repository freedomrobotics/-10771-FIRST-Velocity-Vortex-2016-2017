package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by joelv on 1/7/2017.
 */

@Autonomous (name="GyroTest")
public class GyroTest extends LinearOpMode{
    BNO055IMU DOF;
    Gyro handler;
    @Override
    public void runOpMode() throws InterruptedException {
        DOF = hardwareMap.get(BNO055IMU.class, "imu");
        //L3GD20H + LSM303
        //handler = new Gyro(DOF);
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Gyro", (DOF==null));
            telemetry.addData("X", handler.raw("X"));
            telemetry.addData("Y", handler.raw("Y"));
            telemetry.addData("Z", handler.raw("Z"));
            telemetry.addData("4", handler.raw("A"));
            telemetry.update();
        }
    }
}
