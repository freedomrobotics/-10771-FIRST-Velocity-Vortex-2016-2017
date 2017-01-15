package org.fhs.robotics.ftcteam10771.lepamplemousse.actions;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu.Accelerometer;
import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu.Gyrometer;
import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu.IMU;
import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu.Magnetometer;

/**
 * Created by joelv on 1/14/2017.
 */

@TeleOp(name = "IMU Test")
public class IMUTest extends LinearOpMode{

    BNO055IMU imu;
    IMU imuHandler;
    Gyrometer gyrometer;
    Accelerometer accelerometer;
    Magnetometer magnetometer;

    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(BNO055IMU.class, "BNO");
        imuHandler = new IMU(imu, false);
        gyrometer.setAngleUnit(BNO055IMU.AngleUnit.DEGREES);
        accelerometer.setAccelerationUnit(BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC);
        magnetometer.toggleMagnetometer(true);
        imuHandler.imuInit();
        gyrometer = (Gyrometer)imuHandler;
        accelerometer = (Accelerometer)imuHandler;
        gyrometer.testGyro(this, false);
        accelerometer.testGyro(this, false);
        magnetometer.testMagnetometer(this, false);
    }
}
