package org.fhs.robotics.ftcteam10771.lepamplemousse.position.core;

/**
 * Created by Adam Li on 12/9/2016.
 */

public class Rotation {

    protected float rot = 0.0f;

    private static final float threepitwo = (float)(3 * Math.PI / 2);
    private static final float pitwo = (float)(Math.PI / 2);
    private static final float twopi = (float)(Math.PI * 2);

    public void setDegrees(float rotDegrees){
        rot = degreesToRadians(rotDegrees);
    }
    public void setRadians(float rotRadians){
        rot = rotRadians > 0 ? rotRadians % twopi : twopi - (rotRadians % twopi);
    }

    public float getDegrees(){
        return radiansToDegrees(rot);
    }
    public float getRadians(){
        return rot;
    }

    /** for heading from -2pi to 0 */
    public void setHeading(float headingRadius) {
        rot = headingRadius <= (threepitwo) ? -(headingRadius + threepitwo) : (pitwo - headingRadius);
    }
    /** for heading from -360 to 0 */
    public void setHeadinDeg(float headingDegrees) {
        setHeading(degreesToRadians(headingDegrees));
    }

    /** for heading from -2pi to 0 */
    public float getHeading(){
        return rot <= (threepitwo) ? -(rot + threepitwo) : (pitwo - rot);
    }
    /** for heading from -360 to 0 */
    public float getHeadingDeg(){
        return radiansToDegrees(getHeading());
    }

    public static float radiansToDegrees(float rad){
        return (rad / (float)Math.PI) * 180.0f;
    }
    public static float degreesToRadians(float deg){
        return (deg / 180.0f) * (float)Math.PI;
    }
}
