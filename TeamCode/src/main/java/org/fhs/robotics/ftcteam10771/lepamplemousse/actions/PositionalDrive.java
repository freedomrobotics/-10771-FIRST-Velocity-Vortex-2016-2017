package org.fhs.robotics.ftcteam10771.lepamplemousse.actions;

import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.entities.Entity;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.entities.Robot;

/**
 * Drive class to a position
 * Created by joelv on 2/5/2017.
 */
public class PositionalDrive {

    Drive drive;
    float initialX = 0.0f;
    float initialY = 0.0f;
    Config.ParsedData settings;
    //add imu later

    public PositionalDrive(Drive drive, Config.ParsedData settings){
        this.drive = drive;
        this.settings = settings;
        //todo get initial x and initial y in the settings config file
    }

    /*
        I really do not know how the robot object would update its position
        THIS ONLY THEORETICALLY WORKS WITHOUT ROBOT ROTATION
        Method is untested but there is a test in the framework_test branch under
        Test_drive3, where the encoder outputs are calculated into X and Y coordinates
    */
    public void updateUsingEncoders(){
        drive.robot.position.setX(getX());
        drive.robot.position.setY(getY());
    }

    /*
        I also do not know the status of the IMU accelerometer(Kalman Filter) whether it is finished or not
     */
    public void updateUsingIMU(){

    }

    private float getX(){
        float inch_per_pulse = 4f  * (float)Math.PI / settings.subData("encoder").getFloat("output_pulses");
        double motorAngle = Math.toRadians(settings.subData("drivetrain").getFloat("motor_angle"));
        float A = -drive.frMotor.getCurrentPosition()*inch_per_pulse;
        float B = -drive.flMotor.getCurrentPosition()*inch_per_pulse;
        float C = -drive.blMotor.getCurrentPosition()*inch_per_pulse;
        float D = -drive.brMotor.getCurrentPosition()*inch_per_pulse;
        float AC = ((A*(float)Math.cos(Math.PI-motorAngle)) + (C*(float)Math.cos(Math.PI-motorAngle)))/2.0f;
        float BD = ((B*(float)Math.cos(motorAngle)) + (D*(float)Math.cos(motorAngle)))/2.0f;
        return  ((AC + BD) / 2.0f) + initialX;
    }

    private float getY(){
        float inch_per_pulse = 4f  * (float)Math.PI / settings.subData("encoder").getFloat("output_pulses");
        double motorAngle = Math.toRadians(settings.subData("drivetrain").getFloat("motor_angle"));
        float A = -drive.frMotor.getCurrentPosition()*inch_per_pulse;
        float B = -drive.flMotor.getCurrentPosition()*inch_per_pulse;
        float C = -drive.blMotor.getCurrentPosition()*inch_per_pulse;
        float D = -drive.brMotor.getCurrentPosition()*inch_per_pulse;
        float AC = ((A*(float)Math.sin(Math.PI-motorAngle)) + (C*(float)Math.sin(Math.PI-motorAngle)))/2.0f;
        float BD = ((B*(float)Math.sin(motorAngle)) + (D*(float)Math.sin(motorAngle)))/2.0f;
        return  ((AC + BD) / 2.0f) + initialY;
    }
}
