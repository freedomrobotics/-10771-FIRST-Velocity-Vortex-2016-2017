package org.fhs.robotics.ftcteam10771.lepamplemousse.position.core;

/**
 * Created by Adam Li on 12/9/2016.
 */

public class Coordinate {

    // TODO: 12/9/2016 move to constuctor for configuration of units
    private static final float cmConversionFactor = 1f;

    public enum UNIT{
        CM_TO_UNIT, MM_TO_UNIT, IN_TO_UNIT, DM_TO_UNIT, M_TO_UNIT, FT_TO_UNIT, UNIT_TO_CM, UNIT_TO_MM, UNIT_TO_IN, UNIT_TO_DM, UNIT_TO_M, UNIT_TO_FT,

    }

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

    public static float convertTo(float x, UNIT convert){
        return x * getConversionFactor(convert);
    }

    public static float getConversionFactor(UNIT convert){
        switch (convert){
            case MM_TO_UNIT:
                return 10f * cmConversionFactor;
            case CM_TO_UNIT:
                return cmConversionFactor;
            case IN_TO_UNIT:
                return 2.54f * cmConversionFactor;
            case DM_TO_UNIT:
                return .1f * cmConversionFactor;
            case M_TO_UNIT:
                return .01f * cmConversionFactor;
            case FT_TO_UNIT:
                return 30.48f * cmConversionFactor;
            case UNIT_TO_MM:
                return 1f/(10f * cmConversionFactor);
            case UNIT_TO_CM:
                return 1f/cmConversionFactor;
            case UNIT_TO_IN:
                return 1f/(2.54f * cmConversionFactor);
            case UNIT_TO_DM:
                return 1f/(.1f * cmConversionFactor);
            case UNIT_TO_M:
                return 1f/(.01f * cmConversionFactor);
            case UNIT_TO_FT:
                return 1f/(30.48f * cmConversionFactor);
            default:
                return -1;
        }
    }
}
