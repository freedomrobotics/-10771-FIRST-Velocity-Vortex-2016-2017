package org.fhs.robotics.ftcteam10771.lepamplemousse.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;

/**
 * The catapult module for the robot. This object creates a thread which
 * manages the function of the entire catapult. The catapult uses a Modern
 * Robotics Distance sensor in order to detect when it is down.
 * <p>Call {@link #launch()} to launch after starting the thread with
 * {@link #start()}. When finished, {@link #stop()} must be called or the
 * robot controller app may crash</p>
 * <p>Configuration structure:
 * <ul>
 * <li><b>map_name:</b> The mapped name of the catapult motor
 * <li><b>reversed:</b> Whether or not the catapult motor is driven in reverse.
 * <li><b>light_sensor:</b> <ul>
 *     <li><b>map_name:</b> The mapped name of the optical distance sensor
 *     <li><b>threshold:</b> The value the Optical Distance Sensor needs to read in order to register the catapult
 * </ul>
 * <li><b>power:</b> The power the driving motor should use
 * <li><b>position_tuning:</b> The encoder steps to move beyond the detection point of teh catapult on the motor
 * <li><b>tuning_power:</b> The power when tuning to position
 * <li><b>forward_position:</b> A generally arbitrary number that's used to move the motor forward
 * <li><b>grace:</b> The grace period in which the light sensor should be ignored to make sure the catapult fully activates
 * <li><b>oscillation_max:</b> The max power to use for the oscillation. Saw wave type oscillation.
 * <li><b>power_increment:</b> The increment per millisecond of the motor in the oscillation
 * </ul></p>
 */

public class Catapult {
    private final DcMotor catapult;
    private final OpticalDistanceSensor stopSensor;

    /** The runnable thread which manages the actual functions of the catapult */
    private Thread catapultThread;

    /** The amount the catapult moves after detecting light break to prepare for launch */
    private final int positionTuning;   //Recommended: 5
    /** The power for when tuning */
    private final float positionTuningPower;
    /** The amount of light needed to let the robot know the catapult is close to ready */
    private final float breakThreshold;
    /** The amount of light needed to let the robot know the catapult is close to ready */
    private final int forwardPosition;
    /** The grace period in milliseconds to wait for for the catapult to pass */
    private final int grace;

    /** The current power of the catapult motor */
    private float motorPower = 0.0f;
    /** The amount to increment the motor power by per millisecond for the oscillation function */
    private final float increment;      //Recommended: 0.5f
    /** Max power of the oscillation */
    private final float oscillationMax;
    /** Motor power to use for driving the motor during moves */
    private final float launchPower;

    /** Launch flag for the catapult */
    private boolean launch = false;

    private long lastTimeNano;

    /**
     * Creates and prepares the Catapult module. Takes the
     * settings <b>specific to the catapult</b>
     * @param catapult The DcMotor that drives the actual catapult
     * @param stopSensor The Optical Distance Sensor that acts as the stop sensor
     * @param catapultSettings The settings <b>specific to the catapult</b>
     */
    public Catapult(DcMotor catapult, OpticalDistanceSensor stopSensor,
                    Config.ParsedData catapultSettings){
        this.catapult = catapult;
        this.stopSensor = stopSensor;

        //Set motor direction
        catapult.setDirection(DcMotor.Direction.FORWARD);
        if (catapultSettings.getBool("reversed"))
            catapult.setDirection(DcMotor.Direction.REVERSE);

        //Set motor settings
        catapult.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapult.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Store values needed from the configuration
        breakThreshold = catapultSettings.subData("light_sensor").getFloat("threshold");
        oscillationMax = catapultSettings.getFloat("oscillation_max");
        launchPower = catapultSettings.getFloat("power");
        positionTuning = catapultSettings.getInt("position_tuning");
        positionTuningPower = catapultSettings.getFloat("tuning_power");
        increment = catapultSettings.getFloat("power_increment");
        forwardPosition = catapultSettings.getInt("forward_position");
        grace = catapultSettings.getInt("grace");
        catapultThread = new Thread(catapultRunnable);
    }

    /**
     * The runnable thread which manages the actual functions of the catapult
     */
    private Runnable catapultRunnable = new Runnable() {
        @Override
        public void run() {
            while(!Thread.currentThread().isInterrupted())
            {
                if (catapultReady()) {
                    catapult.setPower(positionTuningPower);
                    //if catapult is ready, reset the launch variable and lock it in position.
                    catapult.setTargetPosition(catapult.getCurrentPosition() + positionTuning);
                    while(catapult.getCurrentPosition() > catapult.getTargetPosition() + 10
                            || catapult.getCurrentPosition() < catapult.getTargetPosition() - 10
                            && !Thread.currentThread().isInterrupted());
                    launch = false;
                    lastTimeNano = System.nanoTime();
                    //until launch is called, oscillate teh catapult power
                    while (!Thread.currentThread().isInterrupted() && !launch) {
                        if (!catapultReady()) break;
                        oscillate();
                        Thread.yield();
                    }

                    //In case the previous loop was broken for interrupted signal, break to the end here
                    if (Thread.currentThread().isInterrupted())
                        break;

                    //run then grace the catapult
                    runCatapult();
                    try {
                        Thread.sleep(grace);
                    } catch (InterruptedException e) {
                        //if inturrupted, make sure to stop motors, then end it all.
                        catapult.setTargetPosition(0);
                        catapult.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        e.printStackTrace();
                    }
                }
                //runs this until catapult is ready for whatever reason.
                runCatapult();
                Thread.yield();
            }
            //Last things run (outside the isInterrupted() loop).
            catapult.setTargetPosition(0);
            catapult.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    };

    /**
     * Launch the catapult!
     */
    public void launch(){
        launch = true;
    }

    /**
     * Start the catapult thread
     */
    public void start() {
        catapultThread.start();
    }

    /**
     * Stop the catapult thread
     */
    public void stop(){
        catapultThread.interrupt();
    }

    /**
     * Oscillates the motor power
     */
    private void oscillate(){
        int changeTime = (int) (System.nanoTime() - lastTimeNano);
        lastTimeNano += changeTime;
        motorPower += increment * (changeTime * 1e-6);
        if (motorPower >= oscillationMax){
            motorPower -= oscillationMax;
        }
        catapult.setPower(motorPower);
    }

    /**
     * Drives the catapult forward
     */
    private void runCatapult(){
        motorPower = launchPower;
        catapult.setPower(motorPower);
        catapult.setTargetPosition(catapult.getCurrentPosition() + forwardPosition);
    }

    /**
     * Checks to see if the catapult is at the ready point
     * @return whether or not it's close to ready point
     */
    public boolean catapultReady(){
        return stopSensor.getLightDetected() > breakThreshold;
    }

    /**
     * Retrieves the current catapult position
     * @return The current catapult position
     */
    public int getCatapultPosition(){
        return catapult.getCurrentPosition();
    }

    /**
     * @return The current light level of the optical distance sensor
     */
    public double getLight(){return stopSensor.getLightDetected();}
}
