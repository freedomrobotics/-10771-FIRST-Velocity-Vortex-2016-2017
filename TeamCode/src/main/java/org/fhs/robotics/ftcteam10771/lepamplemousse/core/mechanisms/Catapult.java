package org.fhs.robotics.ftcteam10771.lepamplemousse.core.mechanisms;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private Controllers controllers;
    Config.ParsedData settings;

    public Catapult(DcMotor motor, OpticalDistanceSensor opticalDistanceSensor,
                    Controllers controllers, Config.ParsedData settings){
        rotator = motor;
        stopSensor = opticalDistanceSensor;
        this.controllers = controllers;
        this.settings = settings;
    }

    public Runnable runCatapult = new Runnable() {
        @Override
        public void run() {
            while(!Thread.interrupted())
            {
                if (catapultReady()){
                    while(!button()){
                        rotator.setPower(0.0);
                    }
                    while(catapultReady()){
                        rotator.setPower(1.0);
                    }
                }
                rotator.setPower(1.0);
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

    public Thread catapultThread = new Thread(runCatapult);

    //TODO: Put catapult power and light tolerance in settings
}
