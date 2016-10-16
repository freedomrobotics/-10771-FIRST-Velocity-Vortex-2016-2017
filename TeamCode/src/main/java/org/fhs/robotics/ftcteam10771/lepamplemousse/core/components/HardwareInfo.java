package org.fhs.robotics.ftcteam10771.lepamplemousse.core.components;

import com.qualcomm.robotcore.util.SerialNumber;

/**
 * A class for storing a device's information
 * Created by joelv on 12/3/2015.
 */
public class HardwareInfo {

    public int deviceIndex;
    public int portNumber;
    public SerialNumber modSerial;
    public TYPE deviceType;
    public DEVICE deviceName;
    public MODEL deviceModel;
    /**
     * Constructor for Module Information
     *
     * @param moduleType   Module purpose
     * @param moduleModel  Module Model
     * @param moduleSerial Model's Serial Number
     */
    HardwareInfo(DEVICE moduleType, MODEL moduleModel, SerialNumber moduleSerial) {
        deviceType = TYPE.MODULE;
        deviceName = moduleType;
        deviceModel = moduleModel;
        deviceIndex = 0;
        modSerial = moduleSerial;
        portNumber = 0;
    }
    /**
     * The constructor for sensors(and also motors) information
     *
     * @param sensorType       The category/purpose (ie Light, Touch, etc.)
     * @param sensorModel      The model(ie Modern Robotics)
     * @param sensorPortNumber Port number(which port on the module?)
     * @param deviceNumber     Humanly decided label of the device(ie Motor 1, motor 2, etc)
     */
    HardwareInfo(DEVICE sensorType, MODEL sensorModel, int sensorPortNumber, int deviceNumber, boolean sensor) {
        deviceIndex = deviceNumber - 1;
        if (sensor) deviceType = TYPE.SENSOR;
        else deviceType = TYPE.MECHANICAL;
        deviceName = sensorType;
        deviceModel = sensorModel;
        portNumber = sensorPortNumber;
        modSerial = null;
    }

    HardwareInfo() {
        //Default Constructor
    }

    /**
     * Enumeration for category of device
     */
    public enum TYPE {
        MODULE,
        SENSOR,
        MECHANICAL,
    }

    /**
     * Enumeration for identifying the device
     */
    public enum DEVICE {
        DEVICE_INTERFACE_MODULE,
        LEGACY_MODULE,
        MOTOR_CONTROLLER,
        SERVO_CONTROLLER,
        TOUCH_SENSOR,
        LIGHT_SENSOR,
        IR_SEEKER,
        GYRO,
        ACCELERATION_SENSOR,
        COLOR_SENSOR,
        MOTOR,
        SERVO
    }

    /**
     * Enumeration for the model of the device
     */
    public enum MODEL {
        MODERN_ROBOTICS, NXT, ADAFRUIT
    }
}
