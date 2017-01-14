package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Class that handles the ball launcher mechanism motors and servos
 * Created by joelv on 1/13/2017.
 */
public class Launcher {

    /*
        Default constructor
     */
    public Launcher(){

    }

    //The two motors of the launcher mechanism
    private DcMotor launchMotor1 = null;
    private DcMotor launchMotor2 = null;

    //The two varibles to be considered
    //      - angle of the launch
    //      - velocity of the launch
    private double launchAngle = 0.0;
    private double launchSpeed = 0.0;

    /**todo: determine if adjust is to be done by button or analog stick
     * Adjusts angle when user presses a button to increase or decrease
     * @param increasePressed the state of increase button press
     * @param decreasePressed the state of decrease button press
     */
    public void adjustAngle(boolean increasePressed, boolean decreasePressed){
        if ((!(increasePressed && decreasePressed)) && (increasePressed || decreasePressed)){
            launchAngle = increasePressed ? launchAngle + 1.0 : + 0.0;
            launchAngle = decreasePressed ? launchAngle - 1.0 : - 0.0;
        }
    }

    /**
     * Adjusts speed when user presses a button to increase or decrease
     * @param increasePressed the state of increase button press
     * @param decreasePressed the state of decrease button press
     */
    public void adjustSpeed(boolean increasePressed, boolean decreasePressed){
        if ((!(increasePressed && decreasePressed)) && (increasePressed || decreasePressed)){
            launchSpeed = increasePressed ? launchSpeed + 1.0 : + 0.0;
            launchSpeed = decreasePressed ? launchSpeed - 1.0 : - 0.0;
        }
    }

    /**
     * Gets the angle of the launch
     * @return the launch angle
     */
    public double getLaunchAngle(){
        return launchAngle;
    }

    /**
     * Gets the launch speed of the motors
     * @return the launch speed
     */
    public double getLaunchSpeed(){
        return launchSpeed;
    }
}
