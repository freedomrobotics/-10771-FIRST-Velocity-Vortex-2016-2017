package org.fhs.robotics.ftcteam10771.lepamplemousse.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Controllers;

/**
 * Created by joelv on 2/7/2017.
 */

public class CatapultOld {

    private DcMotor rotator;
    private OpticalDistanceSensor stopSensor;
    private boolean ready;
    private float motorPower=0.0f;
    private Controllers controllers;
    public Thread catapultThread;
    private int readyPosition=1;
    private int margin=5;
    private int targetPosition=1440;
    private float increment=0.01f;
    private boolean button = false;
    Config.ParsedData settings;
    private boolean launch;

    public CatapultOld(DcMotor motor, OpticalDistanceSensor opticalDistanceSensor,
                       Controllers controllers, Config.ParsedData settings){
        rotator = motor;
        stopSensor = opticalDistanceSensor;
        this.controllers = controllers;
        this.settings = settings.subData("catapult");
        rotator.setDirection(DcMotorSimple.Direction.REVERSE);
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        targetPosition = this.settings.getInt("target_position");
        //rotator.setTargetPosition(targetPosition);//put config
        //the real culprit of the divide by 0
        //readyPosition = settings.getInt("ready_position");
        readyPosition = this.settings.getInt("ready_position");
        margin = this.settings.getInt("position_margin");
        increment = this.settings.getFloat("power_increment");
        catapultThread = new Thread(runSomething);
    }

    public Runnable runSomething = new Runnable() {
        @Override
        public void run() {
            while(!Thread.currentThread().isInterrupted())
            {
                if (catapultReady()) {
                    rotator.setTargetPosition(rotator.getCurrentPosition() + margin);
                    while (!Thread.currentThread().isInterrupted() && !launch) {
                        oscillate();
                    }
                    if (!Thread.currentThread().isInterrupted()) {
                        launchCatapult();
                        try {
                            Thread.sleep(settings.getInt("grace"));
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                }
                if (!Thread.currentThread().isInterrupted()) launchCatapult();
            }
            rotator.setTargetPosition(0);
            rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    };

    public Runnable runCatapult = new Runnable() {
        @Override
        public void run() {
            while(!Thread.currentThread().isInterrupted())
            {
                if (rotator.getCurrentPosition() > rotator.getCurrentPosition() + margin){
                    if (catapultReady()){
                        while(!button() && !Thread.currentThread().isInterrupted() && !button){
                            oscillate();
                        }
                        button = true;
                        while(catapultReady() && !Thread.currentThread().isInterrupted()){
                            launchCatapult();
                        }
                    }
                }
                else {
                    launchCatapult();
                }
                //returnToSensor();
            }
        }
    };

    public Runnable runEncoder = new Runnable() {
        @Override
        public void run() {
            while (!Thread.interrupted()){
                if (rotator.getCurrentPosition() > rotator.getCurrentPosition() + margin){
                    if (encoderReady()){
                        while(!button()){
                            oscillate();
                        }
                        while(encoderReady()){
                            launchCatapult();
                        }
                    }
                }
                else {
                    launchCatapult();
                }
                returnToReady();
            }
        }
    };

    /**
     * Whether or not the button has been pressed
     * @return
     */
    private Boolean button(){
        return controllers.getDigital("launch");
    }

    public void setLaunch(boolean state){
        launch = state;
    }

    /**
     * Returns light readings from sensor
     * @return A value between 0 and 1
     */
    public double sensorReadings(){
        return stopSensor.getLightDetected();
    }

    /**
     * Oscillates the motor back and forth in place
     */
    public void oscillate(){
        float max = settings.getFloat("oscillation_max");
        motorPower += increment;
        if (motorPower>=max){
            motorPower = 0;
        }
        rotator.setPower(motorPower);
    }

    public void launchCatapult(){
        motorPower = settings.getFloat("launch_power");
        rotator.setPower(motorPower);
        rotator.setTargetPosition(rotator.getCurrentPosition() + targetPosition);
    }

    public boolean catapultReady(){
        return stopSensor.getLightDetected() >
                settings.getFloat("light_tolerance");
    }

    public boolean encoderReady(){
        int modular;
        modular = rotator.getCurrentPosition() % readyPosition;
        if (modular<readyPosition-margin && modular > margin) return false;
        else return true;
    }

    private void returnToSensor(){
        float min = settings.getFloat("oscillation_max")+0.1f;
        if (motorPower>min){
            motorPower -= Math.abs(increment);
        }
        rotator.setPower(motorPower);
    }

    private void returnToReady(){
        int modular = rotator.getCurrentPosition() % readyPosition;
        motorPower = (readyPosition-modular/readyPosition);
        if (motorPower<settings.getFloat("oscillation_max")+0.1f){
            motorPower = settings.getFloat("oscillation_max")+0.1f;
        }
        rotator.setPower(motorPower);
    }

    public int getCatapultPosition(){
        return rotator.getCurrentPosition();
    }
    public int getReadyPosition(){
        return readyPosition;
    }
    public int getTargetPosition(){
        return targetPosition;
    }
    public double getLight(){return stopSensor.getLightDetected();}

    //TODO: Put catapult power and light tolerance in settings
}
