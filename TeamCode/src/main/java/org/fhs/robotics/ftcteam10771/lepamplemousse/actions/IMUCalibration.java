package org.fhs.robotics.ftcteam10771.lepamplemousse.actions;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu.IMU;

/**
 * Created by joelv on 1/14/2017.
 */
@TeleOp (name="Calibration")
public class IMUCalibration extends LinearOpMode{

    BNO055IMU bno055IMU;
    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        bno055IMU = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new IMU(bno055IMU, false);
        imu.setCalibrationFileName("imu.json");
        imu.imuInit();
        waitForStart();
        while (opModeIsActive()){
            imu.storeCalibration(this, true, gamepad1.a);
        }
    }
}
