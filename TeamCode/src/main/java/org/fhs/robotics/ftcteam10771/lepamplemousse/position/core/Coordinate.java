package org.fhs.robotics.ftcteam10771.lepamplemousse.position.core;

/**
 * Created by Adam Li on 12/9/2016.
 */

public class Coordinate {

    // TODO: 12/9/2016 move to constuctor for configuration of units
    private static final float cmConversionFactor = 1f;

    public float x = 0.0f;
    public float y = 0.0f;

    public void setX(float x){
        this.x = x;
    }
    public void setY(float y){
        this.y = y;
    }

    public float getX(){
        return x;
    }
    public float getY(){
        return y;
    }

    public static float mm(float mm){
        return mm * 10f * cmConversionFactor;
    }

    public static float cm(float cm){
        return cm * cmConversionFactor;
    }

    public static float in(float in){
        return in * 2.54f * cmConversionFactor;
    }

    public static float dm(float dm){
        return dm * .1f * cmConversionFactor;
    }

    public static float m(float m){
        return m * .01f * cmConversionFactor;
    }

    public static float ft(float ft){
        return ft * 30.48f * cmConversionFactor;
    }
}
