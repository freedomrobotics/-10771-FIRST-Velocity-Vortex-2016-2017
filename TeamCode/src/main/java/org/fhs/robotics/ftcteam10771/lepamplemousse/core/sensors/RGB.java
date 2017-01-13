package org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.LED;

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
        new RGB(sensor, null, led, null);
    }

    /*
        Constructor with 2 color sensors
     */
    public RGB(ColorSensor leftColorSensor, ColorSensor rightColorSensor){
        new RGB(leftColorSensor, rightColorSensor, null, null);
    }

    /*
        Constructor with 2 LEDs
     */
    public RGB(LED leftLED, LED rightLED){
        new RGB(null, null, leftLED, rightLED);
    }

    /*
        Constructor with 2 color sensors and a LED
     */
    public RGB(ColorSensor sensor1, ColorSensor sensor2, LED led){
        new RGB(sensor1, sensor2, led, null);
    }

    /*
        Constructor with a color sensor and 2 LEDs
     */
    public RGB(ColorSensor sensor, LED led1, LED led2){
        new RGB(sensor, null, led1, led2);
    }

    /*
        Constructor with a single color sensor
     */
    public RGB(ColorSensor sensor){
        new RGB(sensor, null, null, null);
    }

    /*
        Constructor with a single LED
     */
    public RGB(LED led){
        new RGB(null, null, led, null);
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

    public enum Color{
        RED,
        GREEN,
        BLUE,
    }

    /**
     * Determines if the beacon side pointed by chosen sensor
     * is the color of choice
     *
     * @param teamColor the color of the alliance
     * @param direction of the color sensor with respect to the robot
     * @return whether or not the side of beacon is correct color
     */
    public boolean isSide(Color teamColor, Direction direction){
        if (teamColor == Color.RED){
            return (red(direction) > blue(direction));
        }
        else if (teamColor == Color.BLUE){
            return (blue(direction) > red(direction));
        }
        else return false;
    }

    //Chooses left sensor by default
    public boolean isSide(Color teamColor){
        return isSide(teamColor, LEFT);
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

    public void indicateCorrect(Direction direction, Color teamColor){
        switchLED(direction, isSide(teamColor, direction));
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
    public void testRGB(Direction direction, LinearOpMode opMode){
        opMode.telemetry.addData("red", red(direction));
        opMode.telemetry.addData("green", green(direction));
        opMode.telemetry.addData("blue", blue(direction));
        opMode.telemetry.update();
    }

    //Chooses left RGB sensor by default
    public void testRGB(LinearOpMode opMode){
        testRGB(LEFT, opMode);
    }

    /**
     * Tests a chosen led in the class
     * by blinking on and off every second
     *
     * @param direction Choose between left and right LEDs
     * @throws InterruptedException
     */
    public void testLED(Direction direction) throws InterruptedException{
        if (direction==LEFT){
            if (leftLED!=null){
                for (int i=0; i < 11; i++){
                    leftLED.enable(i%2==0);
                    wait(1000);
                }
            }
        }
        else if(direction==RIGHT){
            if (rightLED!=null){
                for (int i=0; i < 11; i++){
                    rightLED.enable(i%2==0);
                    wait(1000);
                }
            }
        }
    }
}
