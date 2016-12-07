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
import com.qualcomm.robotcore.hardware.DigitalChannelController;

import org.firstinspires.ftc.teamcode.R;

import static org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers.Adafruit.direction.BOTH;
import static org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers.Adafruit.direction.LEFT;
import static org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers.Adafruit.direction.NEITHER;
import static org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers.Adafruit.direction.RIGHT;

/**
 * Created by User on 11/23/2016.
 */
@Autonomous(name = "ColorSensorClass", group = "10771")
public class Adafruit {

    private ColorSensor colorSensor1 = null;
    private ColorSensor colorSensor2 = null;

    public Adafruit(){

    }

    public Adafruit(ColorSensor sensor1, ColorSensor sensor2){
        colorSensor1 = sensor1;
        colorSensor2 = sensor2;
    }

    float hsvValues1[] = {0F, 0F, 0F};
    float hsvValues2[] = {0F, 0F, 0F};

            // convert the RGB values to HSV values.
    public void convertToHSV(){
        Color.RGBToHSV((colorSensor1.red() * 255) / 800, (colorSensor1.green() * 255) / 800, (colorSensor1.blue() * 255) / 800, hsvValues1);
        Color.RGBToHSV((colorSensor2.red()) * 255 / 800, (colorSensor2.green() * 255) / 800, (colorSensor2.blue() * 255) / 800, hsvValues2);
    }

    public enum direction{
        LEFT,
        RIGHT,
        NEITHER,
        BOTH
    };

    public direction chooseBeaconSide(String color){
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

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
}
