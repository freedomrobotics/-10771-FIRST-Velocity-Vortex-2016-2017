package org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LED;

import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Alliance;

import static org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.RGB.Direction.BOTH;
import static org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.RGB.Direction.LEFT;
import static org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.RGB.Direction.RIGHT;

/**
 * The class that handles RGB RGB devices
 * that have color sensors and LEDs
 * TODO: change the name of the class
 * Created by joelv on 11/23/2016.
 */
public class RGB {

    private ColorSensor leftColorSensor = null;
    private ColorSensor rightColorSensor = null;
    private LED leftLED = null;
    private LED rightLED = null;
    private float[] hsv = new float[3];

    /*
        Constructor with 2 RGB devices
     */
    public RGB(ColorSensor leftColorSensor, ColorSensor rightColorSensor, LED leftLED, LED rightLED){
        this.leftColorSensor = leftColorSensor;
        this.rightColorSensor = rightColorSensor;
        this.leftLED = leftLED;
        this.rightLED = rightLED;
    }

    /*
        Constructor with one RGB device
     */
    public RGB(ColorSensor sensor, LED led){
        this(sensor, null, led, null);
    }

    /*
        Constructor with 2 color sensors
     */
    public RGB(ColorSensor leftColorSensor, ColorSensor rightColorSensor){
        this(leftColorSensor, rightColorSensor, null, null);
    }

    /*
        Constructor with 2 color sensors and a LED
     */
    public RGB(ColorSensor sensor1, ColorSensor sensor2, LED led){
        this(sensor1, sensor2, led, null);
    }

    /*
        Constructor with a single color sensor
     */
    public RGB(ColorSensor sensor){
        this(sensor, null, null, null);
    }

    /*
        Default constructor
     */
    public RGB(){

    }

    public enum Direction{
        LEFT,
        RIGHT,
        NEITHER,
        BOTH
    };

    /**
     * Converts the alliance enumeration
     * to team color enumeration
     * @param alliance status of the robot
     * @return the team color it is in
     */
    private int convertAllianceStatus(Alliance alliance){
        if (alliance==Alliance.RED_ALLIANCE || alliance==Alliance.RED_ALLIANCE_INSIDE
                || alliance==Alliance.RED_ALLIANCE_OUTSIDE){
            return Color.RED;
        }
        else if (alliance==Alliance.BLUE_ALLIANCE || alliance==Alliance.BLUE_ALLIANCE_INSIDE
                || alliance==Alliance.BLUE_ALLIANCE_OUTSIDE){
            return Color.BLUE;
        }
        return 0;
    }

    /**
     * Switches the beacon on or off
     *
     * @param direction which LED to use
     * @param on true for on and false for off
     */
    public void switchLED(Direction direction, boolean on){
        if (direction==LEFT){
            leftLED.enable(on);
        }
        else if(direction==RIGHT){
            rightLED.enable(on);
        }
        else if(direction==BOTH){
            leftLED.enable(on);
            rightLED.enable(on);
        }
        else return;
    }

    //Chooses left LED by default
    public void switchLED(boolean on){
        switchLED(LEFT, on);
    }

    //Turns left LED on by default
    public void switchLED(){
        switchLED(LEFT, true);
    }

    /**
     * Returns the red value of
     * a chosen RGB sensor
     *
     * @param direction LEFT or RIGHT RGB sensor
     * @return The red integer value of chosen sensor
     */
    public int red(Direction direction){
        if (direction==LEFT){
            if (leftColorSensor != null) {
                return leftColorSensor.red();
            }
        }
        else if (direction==RIGHT){
            if (rightColorSensor != null) {
                return rightColorSensor.red();
            }
        }
        else {
            return 404;
        }
        return 0;
    }

    /**
     * Returns the green value of
     * a chosen RGB sensor
     *
     * @param direction LEFT or RIGHT RGB sensor
     * @return The green integer value of chosen sensor
     */
    public int green(Direction direction){
        if (direction==LEFT){
            if (leftColorSensor != null) {
                return leftColorSensor.green();
            }
        }
        else if (direction==RIGHT){
            if (rightColorSensor != null) {
                return rightColorSensor.green();
            }
        }
        else {
            return 404;
        }
        return 0;
    }

    /**
     * Returns the blue value of
     * a chosen RGB sensor
     *
     * @param direction LEFT or RIGHT RGB sensor
     * @return The blue integer value of chosen sensor
     */
    public int blue(Direction direction){
        if (direction==LEFT){
            if (leftColorSensor != null) {
                return leftColorSensor.blue();
            }
        }
        else if (direction==RIGHT){
            if (rightColorSensor != null) {
                return rightColorSensor.blue();
            }
        }
        else {
            return 404;
        }
        return 0;
    }

    //Chooses left color sensor red by default
    public int red(){
        return red(LEFT);
    }

    //Chooses left color sensor red by default
    public int green(){
        return green(LEFT);
    }

    //Chooses left color sensor red by default
    public int blue(){
        return blue(LEFT);
    }

    public void convertToHSV(Direction direction){
        Color.RGBToHSV(red(direction), green(direction), blue(direction), hsv);
    }

    public void convertToHSV(){
        convertToHSV(LEFT);
    }

    /**
     * Indicate which alliance the beacon side
     * belongs to
     * @param direction of which color sensor to use
     * @return the alliance it belongs to
     */
    public Alliance beaconSide(Direction direction){
        convertToHSV(direction);
        if (hsv[2]<3 || hsv[1]<3){
            return Alliance.UNKNOWN;
        }
        if (hsv[0] < 10 || hsv[0] > 350) {
            return Alliance.RED_ALLIANCE;
        }
        else if (hsv[0] > 225 || hsv[0] < 250){
            return Alliance.BLUE_ALLIANCE;
        }
        return Alliance.UNKNOWN;
    }

    public Alliance beaconSide(){
        return beaconSide(LEFT);
    }

    public float getHue(Direction direction){
        convertToHSV(direction);
        return (hsv[0]);
    }

    public float getSaturation(Direction direction){
        convertToHSV(direction);
        return (hsv[1]);
    }

    public float getBrightness(Direction direction){
        convertToHSV(direction);
        return (hsv[2]);
    }

    public float getHue(){
        return getHue(LEFT);
    }

    public float getSaturation(){
        return getSaturation(LEFT);
    }

    public float getBrightness(){
        return getBrightness(LEFT);
    }
}
