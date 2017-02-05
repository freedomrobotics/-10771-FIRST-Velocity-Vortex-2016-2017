package org.fhs.robotics.ftcteam10771.lepamplemousse.actions;

import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;

/**
 * Drive class to a position
 * Created by joelv on 2/5/2017.
 */
public class PositionalDrive {

    Drive drive;
    float initialX = 0.0f;
    float initialY = 0.0f;
    Config.ParsedData settings;

    public PositionalDrive(Drive drive, Config.ParsedData settings){
        this.drive = drive;
        this.settings = settings;
        //todo get initial x and initial y in the settings config file
    }

    public void updateUsingEncoders(){
        float A =
    }

    private float getX(){
        float inch_per_pulse = 4f  * (float)Math.PI / settings.subData("encoder").getFloat("encoder_pulses");
        double motorAngle = Math.toRadians(settings.subData("drivetrain").getFloat("motor_angle"));
        float A = -drive.frMotor.getCurrentPosition()*inch_per_pulse;
        float B = -drive.flMotor.getCurrentPosition()*inch_per_pulse;
        float C = -drive.blMotor.getCurrentPosition()*inch_per_pulse;
        float D = -drive.brMotor.getCurrentPosition()*inch_per_pulse;
        return (A*(float)Math.cos(Math.PI-motorAngle)) + (B*(float)Math.cos(motorAngle))
                + (C*(float)Math.cos(Math.PI-motorAngle)) + (D*(float)Math.cos(motorAngle));
    }
}
