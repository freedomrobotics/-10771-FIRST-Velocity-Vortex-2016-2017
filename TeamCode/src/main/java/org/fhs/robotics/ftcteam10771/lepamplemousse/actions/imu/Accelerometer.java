package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Class that handles the readings of the accerlation sensor of IMU
 * Created by joelv on 1/14/2017.
 */
public class Accelerometer extends IMU{


    private Acceleration gravity;
    private Acceleration acceleration;
    private Position position;
    private Velocity velocity;

    /*
        Default constructor
     */
    public Accelerometer(){

    }

    //Runnable code to stream acceleration sensor readings
    public final Runnable accelorometerRunnable = new Runnable() {
        @Override
        public void run() {
            while(!Thread.interrupted()){
                acceleration = acceleration();
                velocity = velocity();
                position = position();
            }
        }
    };

    //Respective thread
    public final Thread accelerometerThread = new Thread(accelorometerRunnable);

    /**
     * Sets the parameter at an acceleration unit
     * Default: meters per second
     * @param unit to be used for acceleration
     */
    public void setAccelerationUnit(BNO055IMU.AccelUnit unit){
        parameters.accelUnit = unit;
    }

    /**
     * Gets the gravity reading from IMU at instance
     * @return IMU's acceleration with respect to gravity
     */
    public Acceleration gravity(){
        return imu.getGravity();
    }

    /**
     * Gets the acceleration from IMU at instance
     * @return IMU's acceleration
     */
    public Acceleration acceleration(){
        return imu.getAcceleration();
    }

    /**
     * Gets the velocity from IMU at instance
     * @return IMU's velocity
     */
    public Velocity velocity(){
        return imu.getVelocity();
    }

    /**
     * Gets the position from IMU at instance
     * @return IMU's position
     */
    public Position position(){
        return imu.getPosition();
    }

    /**
     * Gets the position from private variable
     * @return the private variable position
     */
    public Acceleration getRunnableAccleration(){
        return gravity;
    }

    /**
     * Gets the acceleration from private variable
     * @return private acceleration variable
     */
    public Acceleration getRunnableAcceleration(){
        return acceleration;
    }

    /**
     * Gets the velocity from private variable
     * @return private velocity variable
     */
    public Velocity getRunnableVelocity(){
        return velocity;
    }

    /**
     * Gets the position from private variable
     * @return the private variable position
     */
    public Position getRunnablePositioon(){
        return position;
    }

    /**
     * Returns the acceleration value of an axis
     * @param axis the chosen axis
     * @param useRunnable whether to use private variable
     * @return the accleration
     */
    public double getAcceleration(Axis axis, boolean useRunnable){
        switch (axis){
            case X:
                return useRunnable ? acceleration.xAccel : acceleration().xAccel;
            case Y:
                return useRunnable ? acceleration.yAccel : acceleration().yAccel;
            case Z:
                return useRunnable ? acceleration.zAccel : acceleration().zAccel;
            default:
                return 0.0;
        }
    }

    /**
     * Returns the velocity value of an axis
     * @param axis the chosen axis
     * @param useRunnable whether to use private variable
     * @return the velocity
     */
    public double getVelocity(Axis axis, boolean useRunnable){
        switch (axis){
            case X:
                return useRunnable ? velocity.xVeloc : velocity().xVeloc;
            case Y:
                return useRunnable ? velocity.xVeloc : velocity().xVeloc;
            case Z:
                return useRunnable ? velocity.xVeloc : velocity().xVeloc;
            default:
                return 0.0;
        }
    }

    /**
     * Returns the position value of a specific axis
     * @param axis the chosen axis
     * @param useRunnable whether to use runnable
     * @return the chosen axis
     */
    public double getPosition(Axis axis, boolean useRunnable){
        switch (axis){
            case X:
                return useRunnable ? position.x : position().x;
            case Y:
                return useRunnable ? position.y : position().y;
            case Z:
                return useRunnable ? position.z : position().z;
            default:
                return 0.0;
        }
    }

    /**
     * Returns acceleration of an axis
     * Default Runnable use state: false
     * @param axis chosen axis
     * @return accelration
     */
    public double getAcceleration(Axis axis){
        return getAcceleration(axis, false);
    }

    /**
     * Returns velocity of an axis
     * Default Runnable use state: false
     * @param axis chosen axis
     * @return velocity
     */
    public double getVelocity(Axis axis){
        return getVelocity(axis, false);
    }

    /**
     * Returns position of an axis
     * Default Runnable use state: false
     * @param axis chosen axis
     * @return position
     */
    public double getPosition(Axis axis){
        return getPosition(axis, false);
    }

    /**
     * Tests the IMU acceleration readings
     * @param opMode test op mode chosen
     * @param updateTelemetry whether to update telemetry
     */
    public void testGyro(LinearOpMode opMode, boolean updateTelemetry){
        opMode.telemetry.addData("AccelX", getAcceleration(Axis.X));
        opMode.telemetry.addData("AccelY", getAcceleration(Axis.Y));
        opMode.telemetry.addData("AccelZ", getAcceleration(Axis.Z));
        if (updateTelemetry) opMode.telemetry.update();
    }
}
