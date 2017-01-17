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

import static com.qualcomm.hardware.adafruit.BNO055IMU.SensorMode.IMU;
import static org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu.IMU.Axis.*;

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
        //imu.initialize();

        /*
        Even before testing, these interface references already don't work

        Gyro gyroOutput = imuHandler.getGyrometer();
        AccelSensor accelSensor = imuHandler.getAccelerometer();
        MagneticSensor magneticSensor = imuHandler.getMagnetometer();
        */


        /*If interface references fail, change
        the scope of the three subclasses to public
        and uncomment this*/

        IMU.Gyrometer gyroOutput = imuHandler.getGyrometer();
        IMU.Accelerometer accelSensor = imuHandler.getAccelerometer();
        IMU.Magnetometer magneticSensor = imuHandler.getMagnetometer();


        /*
        If the commented code fails, do this.
        IMU.Gyrometer gyroOutput = imuHandler.new IMU.Gyrometer();
        IMU.Accelerometer accelSensor = imuHandler.new IMU.Accelerometer();
        IMU.MagneticSensor magneticSensor = imuHandler.new IMU.MagneticSensor();
        */
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("GyroX", gyroOutput.getOrientation(X));
            telemetry.addData("GyroY", gyroOutput.getOrientation(Y));
            telemetry.addData("GyroZ", gyroOutput.getOrientation(Z));
            telemetry.addData("AccelX", accelSensor.getAcceleration(X));
            telemetry.addData("AccelY", accelSensor.getAcceleration(Y));
            telemetry.addData("AccelZ", accelSensor.getAcceleration(Z));
            telemetry.addData("MagX", magneticSensor.getMagneticFlux(X));
            telemetry.addData("MagY", magneticSensor.getMagneticFlux(Y));
            telemetry.addData("MagZ", magneticSensor.getMagneticFlux(Z));

            //If telemetry does not get data, use this
            //imuHandler.testIMU(this, false);

            /*

            If telemetry does not add the data properly still, this is last resort
            Note uncomment line 37 before using this code

            telemetry.addData("GyroX", imu.getAngularOrientation().firstAngle);
            telemetry.addData("GyroY", imu.getAngularOrientation().secondAngle);
            telemetry.addData("GyroZ", imu.getAngularOrientation().thirdAngle);
            telemetry.addData("AccelX", imu.getAcceleration().xAccel);
            telemetry.addData("AccelY", imu.getAcceleration().yAccel);
            telemetry.addData("AccelZ", imu.getAcceleration().zAccel);
            telemetry.addData("MagX", imu.getMagneticFieldStrength().x);
            telemetry.addData("MagY", imu.getMagneticFieldStrength().y);
            telemetry.addData("MagZ", imu.getMagneticFieldStrength().z);
            */
            
            telemetry.update();
        }
    }


}
