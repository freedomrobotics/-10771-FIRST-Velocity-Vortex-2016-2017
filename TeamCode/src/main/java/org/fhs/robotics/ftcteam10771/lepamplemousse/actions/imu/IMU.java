package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.AppUtil;

import java.io.File;


/**
 * Class that handles the robot's IMU device
 * Classes will extend from this
 * Created by joelv on 1/13/2017.
 */
public class IMU {

    protected BNO055IMU imu;
    protected static BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    protected boolean imuInitialized = false;
    private String calibrationFileName = "IMU.json";
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

    //Axis enumeration
    public enum Axis{
        X,
        Y,
        Z
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
    protected void setSensorMode(BNO055IMU.SensorMode mode){
        parameters.mode = mode;
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
}
