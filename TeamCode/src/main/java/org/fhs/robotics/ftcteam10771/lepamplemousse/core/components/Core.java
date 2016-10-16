package org.fhs.robotics.ftcteam10771.lepamplemousse.core.components;

import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera.Camera;

/**
 * Core variables with generic names to be aliased later. Since all of the
 * variables/objects/references will be initialized as null, they should only
 * take a little or no memory space. All possible components should be defined
 * here generically.
 */
public class Core {

    public static TouchSensor touchSensor[];
    public static LightSensor lightSensor[];
    public static ColorSensor colorSensor[];
    public static IrSeekerSensor irSeeker[];
    public static GyroSensor gyrometer[];
    public static AccelerationSensor accelerometer[];
    public static DcMotor motor[];
    public static Servo servo[];
    public static Camera camera[];

}


