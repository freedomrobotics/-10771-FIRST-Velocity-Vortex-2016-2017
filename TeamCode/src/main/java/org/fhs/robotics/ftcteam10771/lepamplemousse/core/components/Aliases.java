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

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * This file either provides a more descriptive or useful alias, or it
 * creates a Java list or hashmap with names from the configuration files
 * so that it's easier to call upon them and any number of references can
 * be made without loss of too much efficiency
 * <p/>
 * todo Implement getters and setter and mutex or mutex-like operations for safe multithreading in the future
 * todo better yet, create an interface or abstract class or like with everything already preset.
 */
public class Aliases {
    /**
     * Motor Map. Values from the Components config file can be used to
     * create an alias reference any motor. Retrieve using ~.motorMap.get("name");
     * Set using ~.motorMap.put("name", motor);
     * Cleared using ~.motorMap.clear("name");
     */
    public static Map<String, DcMotor> motorMap = new HashMap<String, DcMotor>();

    /**
     * Servo Map. Values from the Components config file can be used to
     * create an alias reference any motor. Retrieve using ~.servoMap.get("name");
     * Set using ~.servoMap.put("name", servo);
     * Cleared using ~.servoMap.clear("name");
     */
    public static Map<String, Servo> servoMap = new HashMap<String, Servo>();

    /**
     * Touch Sensor Map. Values from Components config file can be used to
     * create an alias reference any touch sensor.  Retrieve using ~.touchSensorMap.get("name");
     * Set using ~.touchSensorMap.put("name", light sensor);
     * Cleared using ~.touchSensorMap.clear("name");
     */
    public static Map<String, TouchSensor> touchSensorMap = new HashMap<String, TouchSensor>();

    /**
     * Light Sensor Map. Values from Components config file can be used to
     * create an alias reference any light sensor.  Retrieve using ~.lightSensorMap.get("name");
     * Set using ~.lightSensorMap.put("name", light sensor);
     * Cleared using ~.lightSensorMap.clear("name");
     */
    public static Map<String, LightSensor> lightSensorMap = new HashMap<String, LightSensor>();

    /**
     * Color Sensor Map. Values from Components config file can be used to
     * create an alias reference any color sensor.  Retrieve using ~.colorSensorMap.get("name");
     * Set using ~.colorSensorMap.put("name", light sensor);
     * Cleared using ~.colorSensorMap.clear("name");
     */
    public static Map<String, ColorSensor> colorSensorMap = new HashMap<String, ColorSensor>();

    /**
     * IR Seeker Map. Values from Components config file can be used to
     * create an alias reference any ir seeker.  Retrieve using ~.irSeekerMap.get("name");
     * Set using ~.irSeekerMap.put("name", light sensor);
     * Cleared using ~.irSeekerMap.clear("name");
     */
    public static Map<String, IrSeekerSensor> irSeekerMap = new HashMap<String, IrSeekerSensor>();

    /**
     * GyroSensor Map. Values from Components config file can be used to
     * create an alias reference any GyroSensor.  Retrieve using ~.gyrometerMap.get("name");
     * Set using ~.gyrometerMap.put("name", light sensor);
     * Cleared using ~.gyrometerMap.clear("name");
     */
    public static Map<String, GyroSensor> gyrometerMap = new HashMap<String, GyroSensor>();

    /**
     * AccelSensor Map. Values from Components config file can be used to
     * create an alias reference any AccelSensor.  Retrieve using ~.accelerometerMap.get("name");
     * Set using ~.accelerometerMap.put("name", light sensor);
     * Cleared using ~.accelerometerMap.clear("name");
     */
    public static Map<String, AccelerationSensor> accelerometerMap = new HashMap<String, AccelerationSensor>();

    /**
     * Camera Sensor Map. Values from Components config file can be used to
     * create an alias reference any camera sensor.  Retrieve using ~.cameraSensorMap.get("name");
     * Set using ~.cameraMap.put("name", light sensor);
     * Cleared using ~.cameraMap.clear("name");
     */
    public static Map<String, Camera> cameraMap = new HashMap<String, Camera>();

    //region setters

    public static void put(String key, DcMotor motor) {
        motorMap.put(key, motor);
    }

    public static void put(String key, Servo servo) {
        servoMap.put(key, servo);
    }

    public static void put(String key, TouchSensor touchSensor) {
        touchSensorMap.put(key, touchSensor);
    }

    public static void put(String key, LightSensor lightSensor) {
        lightSensorMap.put(key, lightSensor);
    }

    public static void put(String key, ColorSensor colorSensor) {
        colorSensorMap.put(key, colorSensor);
    }

    public static void put(String key, IrSeekerSensor irSeeker) {
        irSeekerMap.put(key, irSeeker);
    }

    public static void put(String key, GyroSensor gyrometer) {
        gyrometerMap.put(key, gyrometer);
    }

    public static void put(String key, AccelerationSensor accelerometer) {
        accelerometerMap.put(key, accelerometer);
    }

    public static void put(String key, Camera camera) {
        cameraMap.put(key, camera);
    }

    public static void put(List<String> key, DcMotor motor) {
        for (int i = 0; i < key.size(); i++) {
            motorMap.put(key.get(i), motor);
        }
    }

    public static void put(List<String> key, Servo servo) {
        for (int i = 0; i < key.size(); i++) {
            servoMap.put(key.get(i), servo);
        }
    }

    public static void put(List<String> key, TouchSensor touchSensor) {
        for (int i = 0; i < key.size(); i++) {
            touchSensorMap.put(key.get(i), touchSensor);
        }
    }

    public static void put(List<String> key, LightSensor lightSensor) {
        for (int i = 0; i < key.size(); i++) {
            lightSensorMap.put(key.get(i), lightSensor);
        }
    }

    public static void put(List<String> key, ColorSensor colorSensor) {
        for (int i = 0; i < key.size(); i++) {
            colorSensorMap.put(key.get(i), colorSensor);
        }
    }

    public static void put(List<String> key, IrSeekerSensor irSeeker) {
        for (int i = 0; i < key.size(); i++) {
            irSeekerMap.put(key.get(i), irSeeker);
        }
    }

    public static void put(List<String> key, GyroSensor gyrometer) {
        for (int i = 0; i < key.size(); i++) {
            gyrometerMap.put(key.get(i), gyrometer);
        }
    }

    public static void put(List<String> key, AccelerationSensor accelerometer) {
        for (int i = 0; i < key.size(); i++) {
            accelerometerMap.put(key.get(i), accelerometer);
        }
    }

    public static void put(List<String> key, Camera camera) {
        for (int i = 0; i < key.size(); i++) {
            cameraMap.put(key.get(i), camera);
        }
    }

    public static void clearAll(){
        motorMap.clear();
        servoMap.clear();
        touchSensorMap.clear();
        lightSensorMap.clear();
        colorSensorMap.clear();
        irSeekerMap.clear();
        gyrometerMap.clear();
        accelerometerMap.clear();
        cameraMap.clear();
    }

    //endregion
}
