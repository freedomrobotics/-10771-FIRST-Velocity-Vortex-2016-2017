package org.fhs.robotics.ftcteam10771.lepamplemousse.actions;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu.Accelerometer;
import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu.Gyrometer;
import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu.IMU;
import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu.Magnetometer;
import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imuInterface.AccelSensor;
import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imuInterface.Gyro;
import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imuInterface.MagneticSensor;

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

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imuHandler = new IMU(imu);
        imuHandler.imuInit(); //possible null pointer here; if there is, check parameters at method

        //Create new instances of subclasses into interface references
        Gyro gyroOutput = imuHandler.getGyrometer();
        AccelSensor accelSensor = imuHandler.getAccelerometer();
        MagneticSensor magneticSensor = imuHandler.getMagnetometer();

        /*
        If interface references fail, change
        the scope of the three subclasses to public
        and uncomment this

        IMU.Gyrometer gyrometer = imuHandler.getGyrometer();
        IMU.Accelerometer accelerometer = imuHandler.getAccelerometer();
        IMU.Magnetometer magnetometer = imuHandler.getMagnetometer();
        */


        //imuHandler.testIMU(this, true);
    }


}
