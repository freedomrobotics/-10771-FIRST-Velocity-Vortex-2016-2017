package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu;


import android.bluetooth.BluetoothAssignedNumbers;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/**
 * Class that handles the gyro sensor readings of the IMU
 * Created by joelv on 1/14/2017.
 */
public class Gyrometer {


    private IMU handler;
    private final AxesOrder axesOrder = AxesOrder.XYZ;
    private final AxesReference reference = AxesReference.INTRINSIC;
    private Orientation orientation;
    private AngularVelocity velocity;
    private boolean gyroStreamEnabled = true;

    /*
        Default constructor
     */
    public Gyrometer(){

    }

    /*
        Constructor that allows reference to an IMU
     */
    public Gyrometer(IMU handler){
        new Gyrometer(handler);
    }

    /**
     * Gets orientation straight from IMU feed
     * @return the IMU's input orientation
     */
    public Orientation sensorOrientation(){
        return handler.imu.getAngularOrientation();
    }

    /**
     * Gets angular velocity straight from IMU
     * @return the IMU's input angular velocity
     */
    public AngularVelocity angularVelocity(){
        return handler.imu.getAngularVelocity();
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
        return velocity;
    }

    //Runnable code that streams orientation and angular velocity
    public final Runnable gyroRunnable = new Runnable() {
        @Override
        public void run() {
            while (!Thread.interrupted()){
                streamGyroData();
            }
        }
    };

    //The runnable's respective thread
    public final Thread gyroThread = new Thread(gyroRunnable);

    /**
     * Streams Gyro Data at an instant;
     * To be used in a looping thread
     */
    public void streamGyroData(){
        if (gyroStreamEnabled){
            orientation = sensorOrientation();
            velocity = angularVelocity();
        }

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
        Orientation orientation;
        Orientation vectorToUse = useRunnable ? this.orientation : sensorOrientation();
        if (intrinsicReference){
            orientation = vectorToUse.toAxesReference(reference);
        }
        else orientation = sensorOrientation();
        orientation = vectorToUse.toAxesOrder(axesOrder);
        switch(axis){
            case X:
                return orientation.firstAngle;
            case Y:
                return orientation.secondAngle;
            case Z:
                return orientation.thirdAngle;
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
        AngularVelocity angularVelocity;
        AngularVelocity vectorToUse = useRunnable ? velocity : angularVelocity();
        if (intrinsicReference){
            angularVelocity = vectorToUse.toAxesReference(reference);
        }
        else angularVelocity = vectorToUse;
        switch(axis){
            case X:
                return angularVelocity.firstAngleRate;
            case Y:
                return angularVelocity.secondAngleRate;
            case Z:
                return angularVelocity.thirdAngleRate;
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

    /**
     * Gyro Tester
     * @param opMode the op mode chosen to test the gyro with
     * @param updateTelemetry whether to update the telemetry
     */
    public void testGyro(LinearOpMode opMode, boolean updateTelemetry){
        opMode.telemetry.addData("GyroX", getOrientation(IMU.Axis.X));
        opMode.telemetry.addData("GyroY", getOrientation(IMU.Axis.Y));
        opMode.telemetry.addData("GyroZ", getOrientation(IMU.Axis.Z));
        if (updateTelemetry) opMode.telemetry.update();
    }
}
