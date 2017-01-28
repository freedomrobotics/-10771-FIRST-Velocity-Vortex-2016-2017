package org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

/**
 * Class that handles the Adafruit
 * Range sensor
 *
 * Created by joelv on 1/11/2017.
 */
public class UltrasonicRange2 {

    private AnalogInput rangeSensor;
    private DigitalChannel toggler;
    private final double scaleFactor = 5.0 / 512.0;
    private boolean rangeStreamEnabled = false;
    private long averagingTime = 10;
    private long aimTime;
    private long polls;

    /**
     * Constructor for a full range sensor
     *
     * @param analogInput    the device which gives voltage inputs
     * @param digitalChannel the output that gives boolean state
     */
    public UltrasonicRange2(AnalogInput analogInput, DigitalChannel digitalChannel) {
        rangeSensor = analogInput;
        toggler = digitalChannel;
        //todo test to see if it works
        toggler.setMode(DigitalChannelController.Mode.OUTPUT);
    }

    //The variable to be used during looping thread
    private double distance;

    /**
     * A runnable thread that streams the distance
     * every instant
     */
    private final Runnable rangeRunnable = new Runnable() {
        @Override
        public void run() {
            while (!Thread.interrupted() && rangeStreamEnabled){
                streamDistance();
            }
        }
    };

    //The respective thread that runs the runnable
    private final Thread rangeThread = new Thread(rangeRunnable);


    //friendlier
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
     * Gives the instantaneous distance in inches
     *
     * @return the distance in inches
     */
    private double distance() {
        if (rangeSensor != null && toggler != null) {
            //if it's not enabled, might as well enable it for the split second
            // might not work because of lag, so in that case, uncomment sleep
            if (!toggler.getState()){
                toggler.setState(true);
                //try{Thread.sleep(8);}
                //catch(InterruptedException e){ e.printStackTrace();}
                double d = rangeSensor.getVoltage() / scaleFactor;
                toggler.setState(false);
                return d;
            }
            return rangeSensor.getVoltage() / scaleFactor;
        }
        //This is when the rangesensor cannot be found, earlier it was just returning 404 if it was not enabled
        return -404;
    }

    //friendlier name
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
        if (!rangeStreamEnabled)
            return distance();
        return distance;
    }

    /**
     * Functions that streams the distance in an instance
     */
    private void streamDistance(){
        if (rangeStreamEnabled){
            if (System.currentTimeMillis() < aimTime) {
                //I was lazy
                double distance = this.distance * polls++;
                distance += distance();
                this.distance = distance / polls;
            } else{
                distance = distance();
                polls = 0;
                aimTime = System.currentTimeMillis() + averagingTime;
            }
        }
        else distance = 0.0;
    }

    /**
     * Turn stream method on or off
     * @param state the state of the method
     */
    public void toggleStream(boolean state){
        rangeStreamEnabled = state;
        //enable disable the sensor depending on rangestream on or off
        disable();
        if (state) {
            rangeThread.start();
            enable();
            aimTime = System.currentTimeMillis() + averagingTime;
        }
    }

    /**
     * across what period of time should the streaming function average the values
     * @param millis time in milliseconds
     */
    public void setAveragingTime(long millis){
        averagingTime = millis;
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
