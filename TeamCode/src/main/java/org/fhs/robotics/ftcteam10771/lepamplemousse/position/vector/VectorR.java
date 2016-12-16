package org.fhs.robotics.ftcteam10771.lepamplemousse.position.vector;

import org.fhs.robotics.ftcteam10771.lepamplemousse.position.core.Coordinate;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.core.Rotation;

/**
 * Created by Adam Li on 12/10/2016.
 */

public class VectorR {
    Coordinate coordinate;
    Rotation rotation;

    // Vector
    //param
    public float getX(){
        return 0;
    }
    public float getY(){
        return 0;
    }
    public void setX(float x){}
    public void setY(float y){}
    //polar
    public float getTheta(){
        return 0;
    }
    public float getRadius(){
        return 0;
    }
    public void setTheta(float theta){}
    public void setRadius(float radius){}

    // Rotation
    public float getRad(){
        return 0;
    }
    public float getDeg(){
        return 0;
    }
    public float getRawR(){
        return 0;
    }
    public void setDeg(float deg){}
    public void setRad(float rad){}
    public void setRawR(float r){}

    // All
    //param
    public void setAllRaw(float x, float y, float r){}
    public void setAllRad(float x, float y, float rad){}
    public void setAllDeg(float x, float y, float deg){}
    //polar
    void setAllRawPol(float theta, float radius, float r){}
    void setAllRadPol(float theta, float radius, float rad){}
    void setAllDegPol(float theta, float radius, float deg){}
}
