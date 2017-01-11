package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;

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
            return 1;
        }
        return 0;
    }
}
