package org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.internal.AppUtil;

import java.io.File;

import static org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.IMU.Axis.X;
import static org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.IMU.Axis.Y;
import static org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.IMU.Axis.Z;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;


/**
 * Class that handles the robot's IMU device
 * Classes will extend from this
 * Created by joelv on 1/13/2017.
 */
public class IMU {

    public BNO055IMU imu;
    public BNO055IMU.Parameters parameters = null;
    boolean imuInitialized = false;
    private String calibrationFileName = "imu.json";

    //Runnable variables
    private Orientation orientation = null;    //might move to main IMU class
    private AngularVelocity angularVelocity = null;   //might move to main IMU class
    private final Orientation zero = new Orientation();



    //Stream flags that can be toggled on or off
    private boolean gyroStreamEnabled = false;
    private long streamDelay = 100;
    /*
        Default constructor
     */
    public IMU(){

    }

    /*
        Constructor that requires reference to an IMU
        This time, the boolean flag for auto-init is false;
        After seeing how much (true) damage "true" has done :P
     */
    public IMU(BNO055IMU imu){
        this(imu, false, false);
    }

    /*
        Constructor that requires user to indicate both IMU
        and whether to initialize the IMU instantly
     */
    public IMU(BNO055IMU imu, boolean calibrating, boolean initialize){
        this.imu = imu;
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        if (!calibrating) parameters.calibrationDataFile = calibrationFileName;
        //parameters.accelerationIntegrationAlgorithm = new KalmanFilterAccelerationIntegrator();
        if (initialize){
            imuInitialized = this.imu.initialize(parameters);
        }
    }

    /**
     * Initializes the IMU with the given parameters
     */
    public void imuInit(){
        if (!imuInitialized){
            imuInitialized = imu.initialize(parameters);
        }
    }

    /**
     * Getter for IMU init state
     * @return the init state
     */
    public boolean isImuInit(){
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
     * Store the calibration data of the IMU into a file
     */
    public void storeCalibration(LinearOpMode opMode, Boolean teleopUsed, boolean buttonPressed){
        if (opMode != null){
            opMode.telemetry.addLine(imu.getCalibrationStatus().toString());
            opMode.telemetry.addData("GyroCalib", imu.isGyroCalibrated());
        }
        boolean beginSave = teleopUsed ? buttonPressed : getIMUCalibrationStatus();
        if (beginSave){
            BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();
            File calibFile = AppUtil.getInstance().getSettingsFile(calibrationFileName);
            ReadWriteFile.writeFile(calibFile, calibrationData.serialize());
            opMode.telemetry.addLine("saved to " + calibrationFileName);
        }
        if (opMode!=null) opMode.telemetry.update();
        if (beginSave && !teleopUsed){
            opMode.stop();
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
        return (imu.isGyroCalibrated());
    }

    //Runnable code that streams orientation and angular velocity
    public final Runnable imuRunnable = new Runnable() {
        @Override
        public void run() {
            while (!Thread.currentThread().isInterrupted()){
                streamIMUData();
                try {
                    Thread.sleep(streamDelay);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    };

    public void setStreamDelay(long delay){
        this.streamDelay = delay;
    }

    //The runnable's respective thread
    public final Thread imuThread = new Thread(imuRunnable);

    /**
     * Streams Gyro Data at an instant;
     * To be used in a looping thread
     */
    public void streamIMUData(){
        if (gyroStreamEnabled && isImuInit()){
            orientation = imu.getAngularOrientation();
            //angularVelocity = imu.getAngularVelocity();
        }
        else orientation = zero;
        //clearData();
    }

    /**
     * Clears data for any unused stream variables during streaming
     */
    private void clearData(){
        if (!gyroStreamEnabled){
            orientation = null;
            angularVelocity = null;
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
    public class Gyrometer{

        private final AxesOrder axesOrder = XYZ;
        private final AxesReference reference = AxesReference.EXTRINSIC;
        private final float degrees = 360f;
        private final float radians = (float)(2 * Math.PI);

        /*
            Default constructor
         */
        public Gyrometer(){

        }

        /**
         * Gets orientation straight from IMU feed
         * @return the IMU's input orientation
         */
        public Orientation sensorOrientation(){
            return imu.getAngularOrientation();
        }

        /**
         * Gets angular velocity straight from IMU
         * @return the IMU's input angular velocity
         */
        public AngularVelocity angularVelocity(){
            return imu.getAngularVelocity();
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
            Orientation vectorToUse = useRunnable ? orientation.toAxesReference(reference).toAxesOrder(axesOrder)
                    : imu.getAngularOrientation().toAxesReference(reference).toAxesOrder(axesOrder);
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
            switch(axis){
                case X:
                    return vectorToUse.xRotationRate;
                case Y:
                    return vectorToUse.yRotationRate;
                case Z:
                    return vectorToUse.zRotationRate;
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
            return getOrientation(axis, false, true);
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

        /**
         * Getter for angular velocity
         * @return the angular velocity
         */
        public AngularVelocity getAngularVelocity(){
            return imu.getAngularVelocity();
        }

        /**
         * Getter for orientation
         * @return the orientation
         */
        public Orientation getOrientation(){
            return imu.getAngularOrientation();
        }

        /**
         * Getter for extrinsic orientation
         * @return the extrinsic orientatino
         */
        public Orientation getExtrinsicOrientation(){
            return imu.getAngularOrientation().toAxesReference(AxesReference.EXTRINSIC);
        }

        /**
         * Get the parameter range to use
         * @return the range depending on the IMU's current unit
         */
        float angleParam(){
            return (parameters.angleUnit==BNO055IMU.AngleUnit.RADIANS) ? radians : degrees;
        }

        /**
         * Convert the chosen axis angle to range 0 to 360 or 2pi
         * @param axis the chosen axis to convert
         * @param value the angle to convert
         * @return the converted angle
         */
        public float convert(Axis axis, float value){
            float parameter = angleParam();
            switch (axis) {
                case X:
                    if (value < 0) value += parameter;
                    break;
                case Y:
                    if (value < 0) {
                        if (Math.abs(imu.getAngularOrientation().toAxesOrder(axesOrder).firstAngle) > parameter / 4f) {
                            value = (parameter / 2f) - value;
                        } else value += parameter;
                    } else {
                        if (Math.abs(imu.getAngularOrientation().toAxesOrder(axesOrder).firstAngle) > parameter / 4f) {
                            value = (parameter / 2f) - value;
                        }
                    }
                    break;
                case Z:
                    value += parameter;
                    break;
                default:
                    return value;
            }
            return value;
        }

        /**
         * Converts orientation to range 0 to 360 or 2pi
         * @param orientation the orientation to convert
         * @return the converted orientation
         */
        public Orientation convertOrientation(Orientation orientation){
            orientation = orientation.toAxesReference(reference).toAxesOrder(XYZ);
            orientation.firstAngle = convert(X, orientation.firstAngle);
            orientation.secondAngle = convert(Y, orientation.secondAngle);
            orientation.thirdAngle = convert(Z, orientation.thirdAngle);
            return orientation;
        }

        /**
         * Converts an angle to range -180 to 180 or -pi to pi
         * @param value the angle to convert
         * @return the converted angle
         */
        public float convertAngletoSemiPossibleRange(Axis axis, float value){
            float parameter = angleParam();
            value = convert(axis, value);
            return (value > (parameter/2f) && value < (parameter)) ?
                    value - parameter : value;
        }

        /**
         * Converts the orientation to range -180 to 180 or -pi to pi
         * @param orientation the orientation to convert
         * @return the converted orientation
         */
        public Orientation convertToSemiPossibleRange(Orientation orientation){
            float parameter = angleParam();
            orientation = convertOrientation(orientation);
            orientation.firstAngle = convertAngletoSemiPossibleRange(X, orientation.firstAngle);
            orientation.secondAngle = convertAngletoSemiPossibleRange(Y, orientation.secondAngle);
            orientation.thirdAngle = convertAngletoSemiPossibleRange(Z, orientation.thirdAngle);
            return orientation;
        }
    }

    /**
     * Creates a new instance of the class Gyro
     * for the object
     * @return the new instance of private Gyro class
     */
    public IMU.Gyrometer getGyrometer(){
        return new IMU.Gyrometer();
    }

    /**
     * IMU Tester
     * @param opMode the op mode chosen to test the gyro with
     * @param updateTelemetry whether to update the telemetry
     */
    public void testIMU(LinearOpMode opMode, boolean updateTelemetry){
        opMode.telemetry.addData("GyroX", imu.getAngularOrientation().firstAngle);
        opMode.telemetry.addData("GyroY", imu.getAngularOrientation().secondAngle);
        opMode.telemetry.addData("GyroZ", imu.getAngularOrientation().thirdAngle);
        opMode.telemetry.addData("AccelX", imu.getLinearAcceleration().xAccel);
        opMode.telemetry.addData("AccelY", imu.getLinearAcceleration().yAccel);
        opMode.telemetry.addData("AccelZ", imu.getLinearAcceleration().zAccel);
        opMode.telemetry.addData("MagnetX", imu.getMagneticFieldStrength().x);
        opMode.telemetry.addData("MagnetY", imu.getMagneticFieldStrength().y);
        opMode.telemetry.addData("MagnetZ", imu.getMagneticFieldStrength().z);
        if (updateTelemetry) opMode.telemetry.update();
    }

    /**
     * IMU getter
     * @return this imu
     */
    public BNO055IMU getImu(){
        return imu;
    }

    /**
     * Start IMU thread
     */
    public void start(){
        if (!imuThread.isAlive()) imuThread.start();
    }

    /**
     * Stop IMU thread
     */
    public void stop(){
        if (imuThread.isAlive()) imuThread.interrupt();
    }

    public void close(){
        if (imuThread.isAlive()){
            stop();
        }
        if (imuInitialized){
            imu.close();
        }
        imuInitialized = false;
    }
}
