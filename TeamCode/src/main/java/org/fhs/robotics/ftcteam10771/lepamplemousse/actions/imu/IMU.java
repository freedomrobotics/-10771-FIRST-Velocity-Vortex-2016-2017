package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imuInterface.*;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.internal.AppUtil;

import java.io.File;


/**
 * Class that handles the robot's IMU device
 * Classes will extend from this
 * Created by joelv on 1/13/2017.
 */
public class IMU {

    public BNO055IMU imu;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    boolean imuInitialized = false;
    private String calibrationFileName = "IMU.json";

    //Runnable variables
    private Orientation orientation = null;    //might move to main IMU class
    private AngularVelocity angularVelocity = null;   //might move to main IMU class
    private Acceleration gravity;
    private Acceleration acceleration;
    private Position position;
    private Velocity velocity;


    //Stream flags that can be toggled on or off
    private boolean gyroStreamEnabled = true;
    private boolean accelStreamEnabled = true;

    /*
        Default constructor
     */
    public IMU(){

    }

    /*
        Constructor that requires reference to an IMU
     */
    public IMU(BNO055IMU imu){
        new IMU(imu, true);
    }

    /*
        Constructor that requires user to indicate both IMU
        and whether to initialize the IMU instantly
     */
    public IMU(BNO055IMU imu, boolean initialize){
        this.imu = imu;
        if (initialize){
            imuInitialized = imu.initialize();
        }
    }

    /**
     * Initializes the IMU with the given parameters
     */
    public void imuInit(){
        imuInitialized = imu.initialize(parameters);
    }

    /**
     * Getter for IMU init state
     * @return the init state
     */
    protected boolean isImuInit(){
        return imuInitialized;
    }

    /**
     * Function that changes the sensor mode
     * @param mode operational mode of the IMU
     */
    public void setSensorMode(BNO055IMU.SensorMode mode){
        parameters.mode = mode;
    }

    /**
     * Setter for the angular unit
     * for both velocity and orientation
     * Default: RADIANS
     * @param unit to use for gyro measurements
     */
    public void setAngleUnit(BNO055IMU.AngleUnit unit){
        parameters.angleUnit = unit;
    }

    /**
     * Sets the parameter at an acceleration unit
     * Default: meters per second
     * @param unit to be used for acceleration
     */
    public void setAccelerationUnit(BNO055IMU.AccelUnit unit){
        parameters.accelUnit = unit;
    }

    /**
     * Store the calibration data of the IMU into a file
     */
    public void storeCalibration(LinearOpMode opMode, Boolean teleopUsed, boolean buttonPressed){
        boolean beginSave;
        if (opMode != null){
            opMode.telemetry.addData("GyroCalib", imu.isGyroCalibrated());
            opMode.telemetry.addData("AccelCalib", imu.isAccelerometerCalibrated());
            opMode.telemetry.addData("MagnetCalib", imu.isMagnetometerCalibrated());
            opMode.telemetry.addData("SystemCalib", imu.isSystemCalibrated());
            opMode.telemetry.update();
        }
        beginSave = teleopUsed ? buttonPressed : getIMUCalibrationStatus();
        if (beginSave){
            BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();
            File calibFile = AppUtil.getInstance().getSettingsFile(calibrationFileName);
            ReadWriteFile.writeFile(calibFile, calibrationData.serialize());
        }
    }

    /**
     * Sets the file to be used by the IMU to read/write calibration data
     * @param fileName the string fileName to be used by the IMU
     */
    public void setCalibrationFileName(String fileName){
        parameters.calibrationDataFile = calibrationFileName = fileName;
    }

    /**
     * Writes the calibration data from the stored file to the IMU
     */
    public void writeCalibrationData(){
        imu.writeCalibrationData(BNO055IMU.CalibrationData.deserialize(calibrationFileName));
    }

    /**
     * Determines if all parts of IMU is calibrated
     * @return if the whole IMU is calibrated
     */
    public boolean getIMUCalibrationStatus(){
        return (imu.isGyroCalibrated() && imu.isAccelerometerCalibrated()
                && imu.isMagnetometerCalibrated() && imu.isSystemCalibrated());
    }

    //Runnable code that streams orientation and angular velocity
    public final Runnable imuRunnable = new Runnable() {
        @Override
        public void run() {
            while (!Thread.interrupted()){
                streamIMUData();
            }
        }
    };

    //The runnable's respective thread
    public final Thread imuThread = new Thread(imuRunnable);

    /**
     * Streams Gyro Data at an instant;
     * To be used in a looping thread
     */
    public void streamIMUData(){
        if (gyroStreamEnabled){
            orientation = imu.getAngularOrientation();
            angularVelocity = imu.getAngularVelocity();
            acceleration = imu.getAcceleration();
            velocity = imu.getVelocity();
            position = imu.getPosition();
        }
    }

    //Axis enumeration
    public enum Axis{
        X,
        Y,
        Z
    }

    /**
     * The class that handles gyrometer outputs
     * which are angular velocity and orientation
     */
    public class Gyrometer implements GyroSensor{

        private final AxesOrder axesOrder = AxesOrder.XYZ;
        private final AxesReference reference = AxesReference.INTRINSIC;

        /*
            Default constructor
         */
        public Gyrometer(){

        }

        /**
         * Gets orientation from private variable used by Runnable
         * @return the private orientation variable
         */
        public Orientation getRunnablOrientation(){
            return orientation;
        }

        /**
         * Gets orientation from private variable used by Runnable
         * @return the private orientation variable
         */
        public AngularVelocity getRunnableAngularVelocity(){
            return angularVelocity;
        }

        /**
         * toggles the stream method of gyro
         * @param state the method's state
         */
        public void enableStream(boolean state){
            gyroStreamEnabled = state;
        }

        /**
         * Getter for an orientation's axis angle
         * @param axis the chosen axis to get angle
         * @param intrinsicReference whether to use intrinsic reference or not
         * @param useRunnable whether to use the private runnable variable
         * @return the angle in parameter's set angle unit
         */
        public float getOrientation(IMU.Axis axis, boolean intrinsicReference, boolean useRunnable){
            Orientation vectorToUse = useRunnable ? orientation : imu.getAngularOrientation();
            if (intrinsicReference){
                vectorToUse = vectorToUse.toAxesReference(reference);
            }
            vectorToUse = vectorToUse.toAxesOrder(axesOrder);
            switch(axis){
                case X:
                    return vectorToUse.firstAngle;
                case Y:
                    return vectorToUse.secondAngle;
                case Z:
                    return vectorToUse.thirdAngle;
                default:
                    return 0f;
            }
        }

        /**
         * Getter for an angular velocity vector's axis angular rate
         * @param axis the chosen axis to get velocity
         * @param intrinsicReference whether to use intrinsic reference or not
         * @param useRunnable whether to use the private runnable variable
         * @return the velocity in parameter's set angle unit
         */
        public float getAngularVelocity(IMU.Axis axis, boolean intrinsicReference, boolean useRunnable){
            AngularVelocity vectorToUse = useRunnable ? angularVelocity : imu.getAngularVelocity();
            if (intrinsicReference){
                vectorToUse = vectorToUse.toAxesReference(reference);
            }
            switch(axis){
                case X:
                    return vectorToUse.firstAngleRate;
                case Y:
                    return vectorToUse.secondAngleRate;
                case Z:
                    return vectorToUse.thirdAngleRate;
                default:
                    return 0f;
            }
        }

        /**
         * Orientation getter
         * Default initrinsic flag = false
         * @param axis the chosen axis
         * @param intrinsicReference whether to use intrinsic reference
         * @return the orienatation
         */
        public float getOrientation(IMU.Axis axis, boolean intrinsicReference){
            return getOrientation(axis, intrinsicReference, false);
        }

        /**
         * Angular Velocity getter
         * Default initrinsic flag = false
         * @param axis the chosen axis
         * @param intrinsicReference whether to use intrinsic reference
         * @return the angular velocity
         */
        public float getAngularVelocity(IMU.Axis axis, boolean intrinsicReference){
            return getAngularVelocity(axis, intrinsicReference, false);
        }

        /**
         * Orientation getter
         * Default initrinsic flag = false
         * @param axis the chosen axis
         * @param useRunnable whether to use private variable
         * @return the orienatation
         */
        public float getOrientation(IMU.Axis axis, Boolean useRunnable){
            return getOrientation(axis, false, useRunnable);
        }

        /**
         * Angular Velocity getter
         * Default initrinsic flag = false
         * @param axis the chosen axis
         * @param useRunnable whether to use private variable
         * @return the angular velocity
         */
        public float getAngularVelocity(IMU.Axis axis, Boolean useRunnable){
            return getAngularVelocity(axis, false, useRunnable);
        }

        /**
         * Orientation getter
         * Default initrinsic flag = false
         * @param axis the chosen axis
         * @return the orienatation
         */
        public float getOrientation(IMU.Axis axis){
            return getOrientation(axis, false, false);
        }

        /**
         * Angular Velocity getter
         * Default initrinsic flag = false
         * @param axis the chosen axis
         * @return the angular velocity
         */
        public float getAngularVelocity(IMU.Axis axis){
            return getAngularVelocity(axis, false, false);
        }
    }

    /**
     * Creates a new instance of the class GyroSensor for the
     * @return
     */
    public IMU.Gyrometer getGyrometer(){
        return new IMU.Gyrometer();
    }

    private class Accelerometer{

    }

    /**
     * Creates a new instance of the class GyroSensor for the
     * @return
     */
    public IMU.Accelerometer getAccelerometer(){
        return new IMU.Accelerometer();
    }

    /**
     * IMU Tester
     * @param opMode the op mode chosen to test the gyro with
     * @param updateTelemetry whether to update the telemetry
     */
    public void testGyro(LinearOpMode opMode, boolean updateTelemetry){

        if (updateTelemetry) opMode.telemetry.update();
    }
}
