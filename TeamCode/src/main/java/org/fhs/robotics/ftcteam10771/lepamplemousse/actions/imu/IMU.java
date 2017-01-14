package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/**
 * Class that handles the robot's IMU device
 * Classes will extend from this
 * Created by joelv on 1/13/2017.
 */
public class IMU {

    protected BNO055IMU imu;
    protected static BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    private boolean imuInitialized = false;

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
    public IMU(BNO055IMU imu, boolean initilize){
        this.imu = imu;
        if (initilize){
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

    // todo refer to op mode sample for calibration method
    protected void calibrate(LinearOpMode opMode){

    }
}
