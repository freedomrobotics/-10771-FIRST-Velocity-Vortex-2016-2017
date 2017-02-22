package org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LED;

import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
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

    private Config.ParsedData beaconSettings;
    private ColorSensor leftColorSensor = null;
    private ColorSensor rightColorSensor = null;
    private LED leftLED = null;
    private LED rightLED = null;
    private float[] hsv = new float[3];
    float blue_min, blue_max, red_min, red_max, value_min, saturation_min;

    /*
        Constructor with 2 RGB devices
     */
    public RGB(ColorSensor leftColorSensor, ColorSensor rightColorSensor, LED leftLED, LED rightLED,
               Config.ParsedData settings){
        this.leftColorSensor = leftColorSensor;
        this.rightColorSensor = rightColorSensor;
        this.leftLED = leftLED;
        this.rightLED = rightLED;
        Config.ParsedData colorSettings = settings.subData("beacon");
        red_min = colorSettings.subData("red").getFloat("min");
        red_max = colorSettings.subData("red").getFloat("max");
        blue_min = colorSettings.subData("blue").getFloat("min");
        blue_max = colorSettings.subData("blue").getFloat("max");
        value_min = colorSettings.getFloat("value_min");
        saturation_min = colorSettings.getFloat("saturation_min");
    }

    /*
        Constructor with one RGB device
     */
    public RGB(ColorSensor sensor, LED led, Config.ParsedData settings){
        this(sensor, null, led, null, settings);
    }

    /*
        Constructor with 2 color sensors
     */
    public RGB(ColorSensor leftColorSensor, ColorSensor rightColorSensor, Config.ParsedData settings){
        this(leftColorSensor, rightColorSensor, null, null, settings);
    }

    /*
        Constructor with 2 color sensors and a LED
     */
    public RGB(ColorSensor sensor1, ColorSensor sensor2, LED led, Config.ParsedData settings){
        this(sensor1, sensor2, led, null, settings);
    }

    /*
        Constructor with a single color sensor
     */
    public RGB(ColorSensor sensor, Config.ParsedData settings){
        this(sensor, null, null, null, settings);
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
                return leftColorSensor.red()/4;
            }
        }
        else if (direction==RIGHT){
            if (rightColorSensor != null) {
                return rightColorSensor.red()/4;
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
                return leftColorSensor.green()/4;
            }
        }
        else if (direction==RIGHT){
            if (rightColorSensor != null) {
                return rightColorSensor.green()/4;
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
                return leftColorSensor.blue()/4;
            }
        }
        else if (direction==RIGHT){
            if (rightColorSensor != null) {
                return rightColorSensor.blue()/4;
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
        if (hsv[2] < value_min && hsv[1] < saturation_min){
            return Alliance.UNKNOWN;
        }
        if (red_max < red_min) {
            if (hsv[0] > red_min || hsv[0] < red_max)
                return Alliance.RED_ALLIANCE;
        }
        if (hsv[0] > red_min && hsv[0] < red_max) {
            return Alliance.RED_ALLIANCE;
        }
        else if (hsv[0] > blue_min && hsv[0] < blue_max){
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
