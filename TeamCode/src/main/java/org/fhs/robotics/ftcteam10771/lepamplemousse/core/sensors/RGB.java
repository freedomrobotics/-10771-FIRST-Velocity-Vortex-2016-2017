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
        Constructor with 2 LEDs
     */
    public RGB(LED leftLED, LED rightLED){
        this(null, null, leftLED, rightLED);
    }

    /*
        Constructor with 2 color sensors and a LED
     */
    public RGB(ColorSensor sensor1, ColorSensor sensor2, LED led){
        this(sensor1, sensor2, led, null);
    }

    /*
        Constructor with a color sensor and 2 LEDs
     */
    public RGB(ColorSensor sensor, LED led1, LED led2){
        this(sensor, null, led1, led2);
    }

    /*
        Constructor with a single color sensor
     */
    public RGB(ColorSensor sensor){
        this(sensor, null, null, null);
    }

    /*
        Constructor with a single LED
     */
    public RGB(LED led){
        this(null, null, led, null);
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
     * Determines if the beacon side pointed by chosen sensor
     * is the color of choice
     *
     * @param teamColor the color of the alliance
     * @param direction of the color sensor with respect to the robot
     * @return whether or not the side of beacon is correct color
     */
    private boolean isSide(int teamColor, Direction direction){
        if (teamColor == Color.RED){
            return (red(direction) > blue(direction));
        }
        else if (teamColor == Color.BLUE){
            return (blue(direction) > red(direction));
        }
        else return false;
    }

    //Chooses left sensor by default
    private boolean isSide(int teamColor){
        return isSide(teamColor, LEFT);
    }

    /**
     * Method to determine if beacon side is correct using alliance enum
     * @param alliance the status of alliance
     * @param direction LEFT or RIGHT sensor
     * @return whether beacon side matches team color
     */
    public boolean isSide(Alliance alliance, Direction direction){
        if (convertAllianceStatus(alliance) != 0){
            return isSide(convertAllianceStatus(alliance), direction);
        }
        else return false;
    }

    /**
     * Method to determine if beacon side is correct using alliance enum
     * Default sensor: LEFT
     * @param alliance the status of alliance
     * @return whether beacon side matches team color
     */
    public boolean isSide(Alliance alliance){
        if (convertAllianceStatus(alliance) != 0){
            return isSide(convertAllianceStatus(alliance));
        }
        else return false;
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
    public void swtichLED(){
        switchLED(LEFT, true);
    }

    private void indicateCorrect(Direction direction, int teamColor){
        switchLED(direction, isSide(teamColor, direction));
    }

    /**
     * Turns the led on after determining that the beacon side matches
     * @param direction which sensor to use
     * @param alliace the alliance status of the robot
     */
    public void indicateCorrcect(Direction direction, Alliance alliace){
        if (convertAllianceStatus(alliace) != 0){
            indicateCorrect(direction, convertAllianceStatus(alliace));
        }
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

    /**
     * Tests to see if the LEFT and RIGHT enumerations
     * work
     *
     * @param direction The chosen enumeration
     * @return The value associated with enumeration
     */
    public int testEnumeration(Direction direction){
        if (direction==LEFT){
            return 1;
        }
        else if (direction==RIGHT){
            return 2;
        }
        else return 0;
    }

    /**
     * Tests a chosen RGB in an op mode
     *
     * @param direction which RGB sensor
     * @param opMode the op mode used to test
     */
    public void testRGB(Direction direction, LinearOpMode opMode, boolean updateTelemetry){
        opMode.telemetry.addData("red", red(direction));
        opMode.telemetry.addData("green", green(direction));
        opMode.telemetry.addData("blue", blue(direction));
        if (updateTelemetry) opMode.telemetry.update();
    }

    //Chooses left RGB sensor by default and not to update the telemetry imediately
    public void testRGB(LinearOpMode opMode){
        testRGB(LEFT, opMode, false);
    }

    //Chooses left sensor by default
    public void testRGB(LinearOpMode opMode, boolean updateTelemetry){
        testRGB(LEFT, opMode, updateTelemetry);
    }

    public void convertToHSV(Direction direction){
        Color.RGBToHSV(red(direction), green(direction), blue(direction), hsv);
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
}
