package org.fhs.robotics.ftcteam10771.lepamplemousse.modes.calibration;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.IMU;

/**
 * Created by joelv on 2/10/2017.
 */

public class GyroCalibration extends LinearOpMode{
    BNO055IMU bno055IMU;
    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        bno055IMU = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new IMU(bno055IMU, true, false);
        imu.imuInit();
        waitForStart();
        while (opModeIsActive()){
            imu.storeCalibration(this, false, false);
        }
    }
}
