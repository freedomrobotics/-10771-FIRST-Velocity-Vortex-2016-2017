package org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
    private boolean rangeStreamEnabled = true;

    /*
        Default constructors
     */
    public UltrasonicRange() {

    }

    /**
     * Constructor for a full range sensor
     *
     * @param analogInput    the device which gives voltage inputs
     * @param digitalChannel the output that gives boolean state
     */
    public UltrasonicRange(AnalogInput analogInput, DigitalChannel digitalChannel) {
        rangeSensor = analogInput;
        toggler = digitalChannel;
        //todo test to see if it works
        toggler.setMode(DigitalChannelController.Mode.OUTPUT);
        toggler.setState(rangeStreamEnabled);
    }

    //The variable to be used during looping thread
    private double distance;

    /**
     * A runnable thread that streams the distance
     * every instant
     */
    public final Runnable rangeRunnable = new Runnable() {
        @Override
        public void run() {
            while (!Thread.currentThread().isInterrupted()){
                streamDistance();
            }
        }
    };

    //The respective thread that runs the runnable
    public final Thread rangeThread = new Thread(rangeRunnable);

    /**
     * Enable the range sensor
     */
    public void enable() {
        toggler.setState(true);
    }

    /**
     * Disable the range sensor
     */
    public void disable() {
        toggler.setState(false);
    }

    /**
     * Gives the distance in inches
     *
     * @return the distance in inches
     */
    public double distance() {
        if (rangeSensor != null && toggler != null) {
            if (!toggler.getState()){
                toggler.setState(true);
                //try{Thread.sleep(8);}
                //catch(InterruptedException e){ e.printStackTrace();}
                double d = rangeSensor.getVoltage() / scaleFactor;
                toggler.setState(false);
                return d;
            }
            return rangeSensor.getVoltage();
        }
        return 0;
    }

    /**
     * Is range sensor on?
     * @return the state of the range sensor
     */
    public boolean state(){
        return toggler.getState();
    }

    /**
     * Returns the distance provided by sensor
     * @return distance provided by sensor
     */
    public double getDistance(){
        return distance;
    }

    /**
     * Functions that streams the distance in an instance
     */
    public void streamDistance(){
        if (rangeStreamEnabled) distance = distance();
        else distance = 0.0;
    }

    /**
     * Turn stream method on or off
     * @param state the state of the method
     */
    public void toggleStream(boolean state){
        rangeStreamEnabled = state;
        toggler.setState(rangeStreamEnabled);
    }

    /**
     * Tests a sensor in any test linear op mode
     * @param opMode the linear op mode used to test
     * @param updateTelemetry whether to update telemetry
     */
    public void testRangeSensor(LinearOpMode opMode, boolean updateTelemetry){
        opMode.telemetry.addData("Inches", distance());
        if (updateTelemetry) opMode.telemetry.update();
    }
}
