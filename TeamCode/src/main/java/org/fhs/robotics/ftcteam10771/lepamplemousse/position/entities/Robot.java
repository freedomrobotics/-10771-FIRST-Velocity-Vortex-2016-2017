package org.fhs.robotics.ftcteam10771.lepamplemousse.position.entities;


import org.fhs.robotics.ftcteam10771.lepamplemousse.position.core.Rotation;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.vector.VectorR;

/**
 * Created by Adam Li on 1/13/2016.
 */
public class Robot implements Entity{

    //includes a 3 inch buffer all around the 18 inches. (24 inch)
    private static final float sizeXY = 609.6f;

    public Robot(float posX, float posY, float rotRad){
        rotation.setRadians(rotRad);
        position.setX(posX);
        position.setY(posY);
        size.setX(sizeXY);
        size.setY(sizeXY);
        offset.setX(sizeXY/2);
        offset.setY(sizeXY/2);
    }

    public Robot(float posX, float posY){
        this(posX, posY, 0);
    }

    public Robot(){
        this(0, 0, 0);
    }

    @Override
    public Rotation getRotation() {
        return rotation;
    }

    @Override
    public Position getPosition() {
        return position;
    }

    @Override
    public Size getSize() {
        return size;
    }

    @Override
    public Offset getOffset() {
        return offset;
    }

    @Override
    public VectorR getVectorR() {
        return new VectorR();
    }
}
