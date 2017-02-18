package org.fhs.robotics.ftcteam10771.lepamplemousse.position.entities;


import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Alliance;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.core.Coordinate;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.core.Rotation;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.vector.VectorR;

/**
 * Created by Adam Li on 1/13/2016.
 * todo document
 */
public class Robot implements Entity{

    //includes a 3 inch buffer all around the 18 inches. (24 inch)
    private static final float sizeXY = 609.6f;
    private Alliance alliance;
    private VectorR vectorR;

    public Robot(Coordinate initialPosition, Coordinate initialSize,
                 Rotation initialRotation, Alliance alliance){
        rotation.setRadians(initialRotation.getRadians());
        position.setX(initialPosition.getX());
        position.setY(initialPosition.getY());
        size.setX(initialSize.getX());
        size.setY(initialSize.getY());
        offset.setX(initialSize.getX()/2);
        offset.setY(initialSize.getY()/2);
        this.alliance = alliance;

        vectorR = new VectorR(position, rotation);
    }

    public Robot(Coordinate initialPosition, Coordinate initialSize,
                 Rotation initialRotation){
        this(initialPosition, initialSize, initialRotation, Alliance.UNKNOWN);
    }

    public Robot(Coordinate initialPosition, Rotation initialRotation,
                 Alliance alliance){
        this(initialPosition, new Size(), initialRotation, alliance);
        rotation.setRadians(initialRotation.getRadians());
        position.setX(initialPosition.getX());
        position.setY(initialPosition.getY());
        size.setX(sizeXY);
        size.setY(sizeXY);
        offset.setX(sizeXY/2);
        offset.setY(sizeXY/2);
        this.alliance = alliance;

        vectorR = new VectorR(position, rotation);
    }

    public Robot(Coordinate initialPosition, Rotation initialRotation){
        this(initialPosition, initialRotation, Alliance.UNKNOWN);
    }

    public Robot(float posX, float posY, float rotRad, Alliance alliance){
        rotation.setRadians(rotRad);
        position.setX(posX);
        position.setY(posY);
        size.setX(sizeXY);
        size.setY(sizeXY);
        offset.setX(sizeXY/2);
        offset.setY(sizeXY/2);
        this.alliance = alliance;

        vectorR = new VectorR(position, rotation);
    }

    public Robot(float posX, float posY, float rotRad){
        this(posX, posY, rotRad, Alliance.UNKNOWN);
    }

    public Robot(float posX, float posY){
        this(posX, posY, 0, Alliance.UNKNOWN);
    }

    public Robot(){
        this(0, 0, 0, Alliance.UNKNOWN);
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
        return vectorR;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }
}
