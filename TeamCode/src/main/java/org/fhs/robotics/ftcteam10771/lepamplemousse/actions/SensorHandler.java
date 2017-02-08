package org.fhs.robotics.ftcteam10771.lepamplemousse.actions;

import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.RGB;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.UltrasonicRange;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera.CameraVision;

/**
 * Class that contains the sensor classes to be used
 * by the autonomous drive class
 * Created by joelv on 2/7/2017.
 */
public class SensorHandler {

    public RGB dualColorSensors;
    public CameraVision cameraVision;
    public UltrasonicRange rangeSensor;
    //public IMU imuSensor;

    public SensorHandler(RGB colorClass, CameraVision cameraVision, UltrasonicRange rangeSensor){
        dualColorSensors = colorClass;
        this.cameraVision = cameraVision;
        this.rangeSensor = rangeSensor;
    }

    public boolean rgbInit(){
        return (dualColorSensors!=null);
    }

    public boolean cameraInit(){
        return (cameraVision!=null);
    }

    public boolean rangeInit(){
        return (rangeSensor!=null);
    }

    /*
    public boolean imuInit(){
     */

}
