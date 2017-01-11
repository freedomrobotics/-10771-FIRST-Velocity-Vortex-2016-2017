package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

/**
 * Class that handles the Adafruit
 * Range sensor
 *
 * Created by joelv on 1/11/2017.
 */
public class UltrasonicRange {

    private AnalogInput rangeSensor;
    private DigitalChannel toggler;
    private final double scaleFactor = 5.0 / 512.0;

    /*
        Default constructors
     */
    UltrasonicRange() {

    }

    /**
     * Constructor for a full range sensor
     *
     * @param analogInput    the device which gives voltage inputs
     * @param digitalChannel the output that gives boolean state
     */
    UltrasonicRange(AnalogInput analogInput, DigitalChannel digitalChannel) {
        rangeSensor = analogInput;
        toggler = digitalChannel;
        //todo test to see if it works
        toggler.setMode(DigitalChannelController.Mode.OUTPUT);
    }

    /**
     * Enable or disable the range sensor
     * @param state true for on or false for off
     */
    public void enable(boolean state) {
        toggler.setState(state);
    }

    /**
     * Gives the distance in inches
     *
     * @return the distance in inches
     */
    public double distance() {
        if (rangeSensor != null && toggler != null) {
            if (toggler.getState()) {
                return rangeSensor.getVoltage() / scaleFactor;
            }
            return 404;
        }
        return 0;
    }

    /**
     * Getter for range sensor state
     */
    public boolean ultrasonicState(){
        return toggler.getState();
    }
}
