package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

import org.firstinspires.ftc.teamcode.R;

import static org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers.Adafruit.Direction.BOTH;
import static org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers.Adafruit.Direction.LEFT;
import static org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers.Adafruit.Direction.NEITHER;
import static org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers.Adafruit.Direction.RIGHT;

/**
 * Created by User on 11/23/2016.
 */
public class Adafruit {

    private ColorSensor colorSensor1 = null;
    private ColorSensor colorSensor2 = null;

    public Adafruit(ColorSensor sensor){
        colorSensor1 = sensor;
        colorSensor2 = null;
    }

    public Adafruit(){

    }

    public Adafruit(ColorSensor sensor1, ColorSensor sensor2){
        colorSensor1 = sensor1;
        colorSensor2 = sensor2;
    }

    public enum Direction{
        LEFT,
        RIGHT,
        NEITHER,
        BOTH
    };

    public boolean isCorrectSide(String teamColor){
        if (teamColor == "red"){
            return (red() > blue());
        }
        else if (teamColor == "blue"){
            return (blue() > red());
        }
        else return false;
    }

    public Direction chooseBeaconSide(String color){
        if (color=="red"){
            if (colorSensor1.red() > colorSensor2.red()){
                return LEFT;
            }
            else if (colorSensor1.red() < colorSensor2.red()){
                return RIGHT;
            }
            else if (colorSensor1.red()==colorSensor2.red()){
                return BOTH;
            }
            else return NEITHER;
        }
        else if(color=="blue"){
            if (colorSensor1.blue() > colorSensor2.blue()){
                return LEFT;
            }
            else if (colorSensor1.blue() < colorSensor2.blue()){
                return RIGHT;
            }
            else if (colorSensor1.blue()==colorSensor2.blue()){
                return BOTH;
            }
            else return NEITHER;
        }
        return NEITHER;
    }

    public int test(Direction direction){
        if (direction==LEFT){
            return 1;
        }
        else if (direction==RIGHT){
            return 2;
        }
        else return 0;
    }

    public int red(Direction direction){
        if (direction==LEFT){
            if (colorSensor1 != null) {
                return colorSensor1.red();
            }
        }
        else if (direction==RIGHT){
            if (colorSensor2 != null) {
                return colorSensor2.red();
            }
        }
        else {
            return 404;
        }
        return 0;
    }

    public int green(Direction direction){
        if (direction==LEFT){
            if (colorSensor1 != null) {
                return colorSensor1.green();
            }
        }
        else if (direction==RIGHT){
            if (colorSensor2 != null) {
                return colorSensor2.green();
            }
        }
        else {
            return 404;
        }
        return 0;
    }

    public int blue(Direction direction){
        if (direction==LEFT){
            if (colorSensor1 != null) {
                return colorSensor1.blue();
            }
        }
        else if (direction==RIGHT){
            if (colorSensor2 != null) {
                return colorSensor2.blue();
            }
        }
        else {
            return 404;
        }
        return 0;
    }
    public int red(){
        return red(LEFT);
    }

    public int green(){
        return green(LEFT);
    }

    public int blue(){
        return blue(LEFT);
    }
            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
}
