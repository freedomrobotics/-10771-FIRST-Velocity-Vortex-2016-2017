package org.fhs.robotics.ftcteam10771.lepamplemousse.core.mechanisms;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Controllers;

/**
 * Created by joelv on 2/7/2017.
 */

public class Catapult {

    private DcMotor rotator;
    private OpticalDistanceSensor stopSensor;
    private boolean ready;
    private float motorPower=0.0f;
    private Controllers controllers;
    public Thread catapultThread;
    private boolean buttonPressed = false;
    Config.ParsedData settings;

    public Catapult(DcMotor motor, OpticalDistanceSensor opticalDistanceSensor,
                    Controllers controllers, Config.ParsedData settings){
        rotator = motor;
        stopSensor = opticalDistanceSensor;
        this.controllers = controllers;
        this.settings = settings.subData("catapult");
        rotator.setDirection(DcMotorSimple.Direction.REVERSE);
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        catapultThread = this.settings.getBool("use_encoder") ? new Thread(runEncoder) : new Thread(runCatapult);
    }

    public Runnable runCatapult = new Runnable() {
        @Override
        public void run() {
            while(!Thread.interrupted())
            {
                if (catapultReady()){
                    while(!button()){
                        oscillate();
                    }
                    while(catapultReady()){
                        rotator.setPower(1.0);
                    }
                }
                rotator.setPower(1.0);
            }
        }
    };

    public Runnable runEncoder = new Runnable() {
        @Override
        public void run() {
            while (!Thread.interrupted()){
                if (encoderReady()){
                    buttonPressed = false;
                    while(!button()){
                        oscillate();
                    }
                    buttonPressed = true;
                    while(encoderReady() && buttonPressed){
                        launchCatapult();
                    }
                }
                returnToReady();
            }
        }
    };

    public boolean catapultReady(){
        return stopSensor.getLightDetected() <
                settings.getFloat("light_tolerance");
    }

    private Boolean button(){
        return controllers.getDigital("launch");
    }

    public double sensorReadings(){
        return stopSensor.getLightDetected();
    }


    /**
     * Oscillates the motor back and forth in place
     */
    public void oscillate(){
        float increment = settings.getFloat("power_increment");
        if (Math.abs(motorPower)>0.3f){
            motorPower = 0.1f;
        }
        if (motorPower>=0.2f){
            increment = -Math.abs(increment);
        }
        else if (motorPower>=-0.2f){
            increment = Math.abs(increment);
        }
        motorPower += increment;
    }

    public void launchCatapult(){
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setTargetPosition(settings.getInt("target_position"));//put config
        motorPower = 1.0f;
        rotator.setPower(motorPower);
    }

    private void returnToReady(){
        int targetPosition = settings.getInt("target_position");
        motorPower = (targetPosition-rotator.getCurrentPosition()/targetPosition);
        rotator.setTargetPosition(targetPosition);
        rotator.setPower(motorPower);
    }

    public boolean encoderReady(){
        int modular;
        int targetPosition = settings.getInt("target_position");
        int margin = settings.getInt("position_margin");
        modular = rotator.getCurrentPosition() % targetPosition;
        if (modular>targetPosition-margin && modular > margin) return false;
        else return true;
    }

    public int getCatapultPosition(){
        return rotator.getCurrentPosition();
    }

    public int getTargetPosition(){
        return settings.getInt("target_position");
    }

    //TODO: Put catapult power and light tolerance in settings
}
