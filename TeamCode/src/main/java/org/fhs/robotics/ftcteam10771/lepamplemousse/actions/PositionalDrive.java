package org.fhs.robotics.ftcteam10771.lepamplemousse.actions;

import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.entities.Entity;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.entities.Robot;

import java.util.List;

/**
 * Drive class to a position
 * Created by joelv on 2/5/2017.
 */
public class PositionalDrive {

    private String settingID = "positional_drive";
    private String team = "red";
    private String position = "general";
    private Drive drive;
    private float initialX = 0.0f;
    private float initialY = 0.0f;
    private Config.ParsedData settings;
    private Config.ParsedData fieldmap;
    private float xMargin = 5.0f;
    private float yMargin = 5.0f;
    int scriptLength;
    List<String> commands;
    //add imu later

    public PositionalDrive(Drive drive, Config.ParsedData settings, Config.ParsedData fieldmap){
        this.drive = drive;
        this.settings = settings;
        this.fieldmap = fieldmap;
        team = settings.getString("alliance");
        String position = settings.getString("position");
        if (((!position.equals("inside"))) && (!position.equals("outside"))){
            position = "inside";
        }
        drive.robot.position.setX(fieldmap.subData(team).subData(position).getFloat("x"));
        drive.robot.position.setY(fieldmap.subData(team).subData(position).getFloat("y"));
        //todo get initial x and initial y in the settings config file(completed?)
        //todo learn how to convert yml list to a list of strings
        commands = (List<String>) fieldmap.subData(team).getObject("script");
        xMargin = settings.subData(settingID).getFloat("x_margin");
        yMargin = settings.subData(settingID).getFloat("y_margin");
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

    public void setCoordinate(String location){
        drive.setPosition(fieldmap.subData(team).subData(location).getFloat("x"), fieldmap.subData(team).subData(location).getFloat("y"));
    }

    private boolean atLocation(){
        float robotX = drive.robot.position.getX();
        float robotY = drive.robot.position.getY();
        float setX = drive.vectorR.getX();
        float setY = drive.vectorR.getY();
        return ((xMargin>Math.abs(robotX-setX))&&(yMargin>Math.abs(robotY-setY)));
    }

    public void startScript(){
        for (String command : commands){
            setCoordinate(command);
            //TODO: PUT SOMETHING THAT PREVENTS THE FOR LOOP FROM HAPPENING IN ONE INSTANCE, LIKE A WHILE LOOP OR SOMETHING
            while (!atLocation()){
                drive.startPosition();
            }
        }
    }
}
