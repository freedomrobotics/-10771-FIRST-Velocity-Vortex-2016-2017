package org.fhs.robotics.ftcteam10771.lepamplemousse.actions;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu.Accelerometer;
import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu.Gyrometer;
import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu.IMU;
import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu.Magnetometer;
import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu.IMU.Axis.X;
import static org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu.IMU.Axis.Y;
import static org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu.IMU.Axis.Z;

/**
 * Created by joelv on 1/14/2017.
 */

@TeleOp(name = "IMU Test4")
public class IMUTest4 extends LinearOpMode{

    BNO055IMU imu;
    IMU imuHandler;
    Gyrometer gyrometer;
    Accelerometer accelerometer;
    Magnetometer magnetometer;
    Config kalmanConfig;

    @Override
    public void runOpMode() throws InterruptedException {

        kalmanConfig = new Config("/IMUtest", "kalman.yml", telemetry, "Kal");

        if (kalmanConfig.read() == Config.State.DEFAULT_EXISTS) {
            kalmanConfig.create(true);
            //read without creating file if it still fails.
            if (kalmanConfig.read() == Config.State.DEFAULT_EXISTS)
                kalmanConfig.read(true);
        }

        Config.ParsedData parsedKalman = kalmanConfig.getParsedData();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imuHandler = new IMU(imu);
        imuHandler.createAccelerationIntegrator(parsedKalman, 4, telemetry);
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

        gyroOutput.enableStream(true);
        accelSensor.enableStream(true);
        magneticSensor.enableStream(true);


        /*
        If the commented code fails, do this.
        IMU.Gyrometer gyroOutput = imuHandler.new IMU.Gyrometer();
        IMU.Accelerometer accelSensor = imuHandler.new IMU.Accelerometer();
        IMU.MagneticSensor magneticSensor = imuHandler.new IMU.MagneticSensor();
        */
        waitForStart();
        //imuHandler.imuThread.start();
        //start accel integrator
        //imuHandler.parameters.accelerationIntegrationAlgorithm.initialize(imuHandler.parameters, new Position(), new Velocity());
        imuHandler.getImu().startAccelerationIntegration(new Position(), new Velocity(), 100);
        while(opModeIsActive()){
            imuHandler.streamIMUData();
            //imuHandler.parameters.accelerationIntegrationAlgorithm.update(new Acceleration(DistanceUnit.METER,
                   // accelSensor.getAbsoluteAcceleration(X), accelSensor.getAbsoluteAcceleration(Y), 0.0, 100));
            //telemetry.addData("GyroX", (int)gyroOutput.convertAngletoSemiPossibleRange(X, gyroOutput.getOrientation(X)));
            //telemetry.addData("GyroY", (int)gyroOutput.convertAngletoSemiPossibleRange(Y, gyroOutput.getOrientation(Y)));
            //telemetry.addData("GyroZ", (int)gyroOutput.convertAngletoSemiPossibleRange(Z, gyroOutput.getOrientation(Z)));
            telemetry.addData("GyroZ", gyroOutput.getOrientation(Z));
            telemetry.addData("======", "========");
            telemetry.addData("AbsX", (int)accelSensor.getAbsoluteAcceleration(X));
            telemetry.addData("Absy", (int)accelSensor.getAbsoluteAcceleration(Y));
            telemetry.addData("======", "========");
            telemetry.addData("AccelX", accelSensor.getAcceleration(X));
            telemetry.addData("AccelY", accelSensor.getAcceleration(Y));
            telemetry.addData("AccelZ", accelSensor.getAcceleration(Z));
            telemetry.addData("======", "========");
            telemetry.addData("Velox", (int)accelSensor.getVelocity(X));
            telemetry.addData("Veloy", (int)accelSensor.getVelocity(Y));
            telemetry.addData("Veloz", (int)accelSensor.getVelocity(Z));
            telemetry.addData("======", "========");
            telemetry.addData("Position", (int)accelSensor.getPosition(X));
            telemetry.addData("Position", (int)accelSensor.getPosition(Y));
            telemetry.addData("Position", (int)accelSensor.getPosition(Z));
            telemetry.addData("======", "========");


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
        //imuHandler.imuThread.interrupt();
    }


}
