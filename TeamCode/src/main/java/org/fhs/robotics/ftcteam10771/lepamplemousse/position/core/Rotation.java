package org.fhs.robotics.ftcteam10771.lepamplemousse.position.core;

/**
 * Created by Adam Li on 12/9/2016.
 */

public class Rotation {

    protected float rot = 0.0f;

    public void setDegrees(float rotDegrees){
        rot = degreesToRadians(rotDegrees);
    }
    public void setRadians(float rotRadians){
        rot = rotRadians;
    }

    public float getDegrees(){
        return radiansToDegrees(rot);
    }
    public float getRadians(){
        return rot;
    }

    public static float radiansToDegrees(float rad){
        return (rad / (float)Math.PI) * 180.0f;
    }
    public static float degreesToRadians(float deg){
        return (deg / 180.0f) * (float)Math.PI;
    }
}
