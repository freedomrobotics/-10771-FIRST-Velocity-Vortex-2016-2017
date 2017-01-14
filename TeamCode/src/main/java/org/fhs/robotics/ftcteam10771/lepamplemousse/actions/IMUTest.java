package org.fhs.robotics.ftcteam10771.lepamplemousse.actions;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu.Accelerometer;
import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu.Gyrometer;
import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu.IMU;
import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu.Magnetometer;

/**
 * Created by joelv on 1/14/2017.
 */

public class IMUTest extends LinearOpMode{

    BNO055IMU imu;
    IMU imuHandler;
    Gyrometer gyrometer;

    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(BNO055IMU.class, "BNO");
        imuHandler = new IMU(imu, false);
        gyrometer.setAngleUnit(BNO055IMU.AngleUnit.DEGREES);
        imuHandler.imuInit();
        gyrometer = (Gyrometer)imuHandler;
        gyrometer.testGyro(this, true);
    }
}
