package org.fhs.robotics.ftcteam10771.lepamplemousse.position.core;

/**
 * Created by Adam Li on 12/9/2016.
 */

public class Rotation {

    public float rot = 0.0f;

    public void setDegrees(float rotDegrees){
        rot = (rotDegrees / 180.0f) * (float)Math.PI;
    }
    public void setRadians(float rotRadians){
        rot = rotRadians;
    }

    public float getDegrees(){
        return (rot / (float)Math.PI) * 180.0f;
    }
    public float getRadians(){
        return rot;
    }
}
