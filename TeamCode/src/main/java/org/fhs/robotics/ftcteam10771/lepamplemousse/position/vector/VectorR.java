package org.fhs.robotics.ftcteam10771.lepamplemousse.position.vector;

import org.fhs.robotics.ftcteam10771.lepamplemousse.position.core.Coordinate;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.core.Rotation;

/**
 * 2 Dimensional Vectors with Rotational values
 * todo write
 */

public class VectorR {
    Coordinate coordinate;
    Rotation rotation;

    /**
     * Default constructor. Generates a zero vector with zero rotation
     */
    public VectorR(){
        this(new Coordinate(), new Rotation());
    }

    /**
     * Generates a {@link VectorR} object with the provided {@link Coordinate} object and zero rotation
     * @param coordinate The {@link Coordinate} object of the vector
     */
    public VectorR(Coordinate coordinate){
        this(coordinate, new Rotation());
    }

    /**
     * Generates a {@link VectorR} object with a zero vector and the provided {@link Rotation} object
     * @param rotation The {@link Rotation} object representing rotation
     */
    public VectorR(Rotation rotation){
        this(new Coordinate(), rotation);
    }

    /**
     * Generates a {@link VectorR} object with the provided {@link Coordinate} and {@link Rotation} objects
     * @param coordinate The {@link Coordinate} object of the vector
     * @param rotation The {@link Rotation} object representing rotation
     */
    public VectorR(Coordinate coordinate, Rotation rotation){
        this.coordinate = coordinate;
        this.rotation = rotation;
    }

//region Vector
    //parametric
    /**
     * Obtains the raw X value of the vector in units. <b>Use
     * {@link Coordinate#convertTo(float, Coordinate.UNIT)} to convert to usable values.</b>
     * @return  The raw X value of the vector
     */
    public float getX(){
        return coordinate.getX();
    }

    /**
     * Obtains the raw Y value of the vector in units. <b>Use
     * {@link Coordinate#convertTo(float, Coordinate.UNIT)} to convert to usable values.</b>
     * @return  The raw Y value of the vector
     */
    public float getY(){
        return coordinate.getY();
    }

    /**
     * Sets the raw X value of the vector in units. <b>Use
     * {@link Coordinate#convertTo(float, Coordinate.UNIT)} to convert to robot units.</b>
     */
    public void setX(float x){
        coordinate.setX(x);
    }

    /**
     * Sets the raw Y value of the vector in units. <b>Use
     * {@link Coordinate#convertTo(float, Coordinate.UNIT)} to convert to robot units.</b>
     */
    public void setY(float y){
        coordinate.setY(y);
    }

    //polar
    /**
     * Obtains the theta value of the vector in the polar plane.
      * @return The theta, or angle, value of the vector in radians
     */
    public float getTheta(){
        return (float)Math.atan2(coordinate.getY(), coordinate.getX());
    }

    /**
     * Obtains the radius value of the vector in the polar plane. <b>Use
     * {@link Coordinate#convertTo(float, Coordinate.UNIT)} to convert to usable values.</b>
     * @return The raw radius value of the vector in the polar plane
     */
    public float getRadius(){
        return (float)Math.sqrt(coordinate.getX()*coordinate.getX()+coordinate.getY()*coordinate.getY());
    }

    /**
     * Sets the polar theta value of the vector in radians. <b><i>IF BOTH RADIUS AND THETA ARE
     * CHANGING, USE {@link this#setPolar(float, float)} TO PREVENT EXCESSIVE MATHS.</i></b>
     * @param theta The theta, or angle, value to set for the vector in radians.
     */
    public void setTheta(float theta){
        float radius = getRadius();
        coordinate.setX((float)(radius*Math.cos(theta)));
        coordinate.setY((float)(radius*Math.sin(theta)));
    }

    /**
     * Sets the polar radius value of the vector. <b><i>IF BOTH RADIUS AND THETA ARE
     * CHANGING, USE {@link this#setPolar(float, float)} TO PREVENT EXCESSIVE MATHS.</i></b>
     * <b>Use {@link Coordinate#convertTo(float, Coordinate.UNIT)} to convert to
     * usable values.</b>
     * @param radius The radius value to set for the vector.
     */
    public void setRadius(float radius){
        float theta = getTheta();
        coordinate.setX((float)(radius*Math.cos(theta)));
        coordinate.setY((float)(radius*Math.sin(theta)));
    }

    /**
     * Sets the polar coordinate values of the vector.<b>Use
     * {@link Coordinate#convertTo(float, Coordinate.UNIT)} to convert the radius value
     * to robot units.</b>
     * @param radius The radius value to set for the vector.
     * @param theta The theta, or angle, value to set for the vector in radians.
     */
    public void setPolar(float radius, float theta){
        coordinate.setX((float)(radius*Math.cos(theta)));
        coordinate.setY((float)(radius*Math.sin(theta)));
    }
//endregion

//region Rotation
    /**
     * @return The rotation value in radians.
     */
    public float getRad(){
        return rotation.getRadians();
    }

    /**
     * @return The rotation value in degrees.
     */
    public float getDeg(){
        return rotation.getDegrees();
    }

    /**
     * @return The raw rotation value. There is no reason for this
     * since rotations are stored in radians.
     */
    public float getRawR(){
        return rotation.getRadians();
    }

    /**
     * Sets the rotation value in degrees
     */
    public void setDeg(float deg){
        rotation.setDegrees(deg);
    }

    /**
     * Sets the rotation value in radians
     */
    public void setRad(float rad){
        rotation.setRadians(rad);
    }

    /**
     * Sets the raw rotation value. There is no advantage to this since
     * rotations are stored in radians regardless.
     */
    public void setRawR(float r){
        rotation.setRadians(r);
    }
//endregion

//region All
    //parametric
    /**
     * Sets all of the parameters of the parametric vector. <b>Use
     * {@link Coordinate#convertTo(float, Coordinate.UNIT)} to convert non-angular values
     * to robot units.</b>
     * @param x The raw x value to set
     * @param y The raw y value to set
     * @param r The raw rotational value
     */
    public void setAllRaw(float x, float y, float r){
        setX(x);
        setY(y);
        setRawR(r);
    }

    /**
     * Sets all of the parameters of the parametric vector. <b>Use
     * {@link Coordinate#convertTo(float, Coordinate.UNIT)} to convert non-angular values
     * to robot units.</b>
     * @param x The raw x value to set
     * @param y The raw y value to set
     * @param rad The rotational value in radians
     */
    public void setAllRad(float x, float y, float rad){
        setX(x);
        setY(y);
        setRad(rad);
    }

    /**
     * Sets all of the parameters of the parametric vector. <b>Use
     * {@link Coordinate#convertTo(float, Coordinate.UNIT)} to convert non-angular values
     * to robot units.</b>
     * @param x The raw x value to set
     * @param y The raw y value to set
     * @param deg The rotational value in degrees
     */
    public void setAllDeg(float x, float y, float deg){
        setX(x);
        setY(y);
        setDeg(deg);
    }

    //polar
    /**
     * Sets all of the parameters of the polar vector. <b>Use
     * {@link Coordinate#convertTo(float, Coordinate.UNIT)} to convert non-angular values
     * to robot units.</b>
     * @param theta The raw theta value to set
     * @param radius The raw radius value to set
     * @param r The raw rotational value
     */
    public void setAllRawPol(float theta, float radius, float r){
        setPolar(radius, theta);
        setRawR(r);
    }

    /**
     * Sets all of the parameters of the polar vector. <b>Use
     * {@link Coordinate#convertTo(float, Coordinate.UNIT)} to convert non-angular values
     * to robot units.</b>
     * @param theta The raw theta value to set
     * @param radius The raw radius value to set
     * @param rad The rotational value in radians
     */
    public void setAllRadPol(float theta, float radius, float rad){
        setPolar(radius, theta);
        setRawR(rad);
    }

    /**
     * Sets all of the parameters of the polar vector. <b>Use
     * {@link Coordinate#convertTo(float, Coordinate.UNIT)} to convert non-angular values
     * to robot units.</b>
     * @param theta The raw theta value to set
     * @param radius The raw radius value to set
     * @param deg The rotational values in degrees
     */
    public void setAllDegPol(float theta, float radius, float deg){
        setPolar(radius, theta);
        setRawR(deg);
    }
//endregion

    public Coordinate getVectorAsCoordinate(){
        return coordinate;
    }

    public Rotation getRotationAsRotation(){
        return rotation;
    }

    public void setVectorAsCoordinate(Coordinate coordinate){
        this.coordinate = coordinate;
    }

    public void setRotationAsRotation(Rotation rotation){
        this.rotation = rotation;
    }
}
