package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * The methods for approaching the targets like buttons, debris, etc.
 * Created by joelv on 11/23/2015.
 */
public class AutoDrive {

    private DcMotor frontRight;
    private DcMotor fronLeft;
    private DcMotor backLeft;
    private DcMotor backRight;

    private double vectorDirection;
    private double vectorMagnitude;
    private double rotationalPower;
    private double ACShaftPower;
    private double BDShaftPower;
    private double ACRotationalPower;
    private double BDRotationalPower;
    private boolean simultaneous = false;
    private boolean swingCounterClockwise = false;
    private boolean swingClockwise = false;

    AutoDrive() {
        //Default constructor
    }

    AutoDrive(DcMotor fR, DcMotor fL, DcMotor bL, DcMotor bR){
        frontRight = fR;
        fronLeft = fL;
        backLeft = bL;
        backRight = bR;
    }

    public void drive(double vectorDirectionInDegrees, double radius){
        vectorMagnitude = radius;
        vectorDirection = Math.toRadians(vectorDirectionInDegrees);
        ACShaftPower = ((Math.sin(vectorDirection-(Math.PI/4)))*vectorMagnitude);
        BDShaftPower = ((Math.cos(vectorDirection-(Math.PI/4)))*vectorMagnitude);
        ACRotationalPower = BDRotationalPower = 0;
        if (!simultaneous) setPower();
    }

    /**
     * The method to drive the robot in a certain direction
     * @param vectorDriectionInDegrees The angle of the vector in Radians
     */
    public void drive(double vectorDriectionInDegrees, Speed speed){
        if (speed==Speed.FAST) drive(vectorDriectionInDegrees, 1);
        else if (speed==Speed.SLOW) drive(vectorDriectionInDegrees, 0.5);
    }

    public void drive(Direction translationDirection, Speed speed){
        double magnitude = 0;
        double vectorAngle = 0;
        if (speed==Speed.FAST){
            magnitude = 1;
        }
        else if(speed==Speed.SLOW){
            magnitude = 0.5;
        }
        switch (translationDirection){
            case FORWARDS:
                vectorAngle = 90;
                break;
            case BACKWARDS:
                vectorAngle = 270;
                break;
            case LEFT:
                vectorAngle = 180;
                break;
            case RIGHT:
                vectorAngle = 0;
                break;
            case FORWARD_RIGHT:
                vectorAngle = 45;
                break;
            case FORWARD_LEFT:
                vectorAngle = 135;
                break;
            case BACKWARD_LEFT:
                vectorAngle = 225;
                break;
            case BACKWARD_RIGHT:
                vectorAngle = 315;
                break;
            default:
                stop();
                break;
        }
        drive(vectorAngle, magnitude);
    }

    /**
     * Rotates the robot around a point
     * @param rotationDirection
     */
    public void rotate(Direction rotationDirection, double rotatePower){
        rotationalPower = rotatePower;
        switch (rotationDirection){
            case CLOCKWISE:
                rotationalPower = -Math.abs(rotationalPower);
                break;
            case COUNTERCLOCKWISE:
                rotationalPower = Math.abs(rotationalPower);
                break;
            default:
                rotationalPower = Math.abs(rotationalPower);
                break;
        }
        ACRotationalPower = (rotationalPower*Math.abs(rotationalPower))/(Math.abs(rotationalPower)+Math.abs(ACShaftPower));
        BDRotationalPower = (rotationalPower*Math.abs(rotationalPower))/(Math.abs(rotationalPower)+Math.abs(BDShaftPower));
        ACShaftPower = BDShaftPower = 0;
        if (!simultaneous) setPower();
    }

    public void rotate(Direction rotationDirection, Speed speed){
        if (speed==Speed.FAST){
            rotate(rotationDirection, 1);
        }
        else rotate(rotationDirection, 0.5);
    }

    public void driveRotate(double vectorDirectionInDegrees, double radius, Direction rotateDirection, double rotatePower){
        simultaneous = true;
        drive(vectorDirectionInDegrees, radius);
        rotate(rotateDirection, rotatePower);
        setPower();
        simultaneous = false;
    }


    public void driveRotate(Direction translation, Speed translationSpeed, Direction rotation, Speed rotationSpeed){
        simultaneous = true;
        drive(translation, translationSpeed);
        rotate(rotation, rotationSpeed);
        setPower();
        simultaneous = false;
    }

    public void swingRotate(Direction rotationDirection, Speed speed){
        if (rotationDirection==Direction.COUNTERCLOCKWISE){
            swingCounterClockwise = true;
            swingClockwise = false;
            rotate(rotationDirection, speed);
        }
        else if (rotationDirection==Direction.CLOCKWISE){
            swingClockwise = true;
            swingCounterClockwise = false;
            rotate(rotationDirection, speed);
        }
        else stop();
        swingClockwise = swingCounterClockwise = false;
    }

    /**
     * Stops the robot by
     * setting motor powers to zero
     */
    public void stop(){
        ACRotationalPower = ACShaftPower = 0;
        BDRotationalPower = BDShaftPower = 0;
        setPower();
    }

    private void setPower(){
        if (!swingClockwise) frontRight.setPower((-ACRotationalPower)+(ACShaftPower*(1.0-Math.abs(ACRotationalPower))));
        else frontRight.setPower(0);
        if (!swingCounterClockwise) fronLeft.setPower((BDRotationalPower)+(BDShaftPower*(1.0-Math.abs(BDRotationalPower))));
        else fronLeft.setPower(0);
        backLeft.setPower((ACRotationalPower)+(ACShaftPower*(1.0-Math.abs(ACRotationalPower))));
        backRight.setPower((-BDRotationalPower)+(BDShaftPower*(1.0-Math.abs(BDRotationalPower))));
    }

    public enum Direction{
        LEFT,
        RIGHT,
        FORWARDS,
        BACKWARDS,
        FORWARD_RIGHT,
        FORWARD_LEFT,
        BACKWARD_LEFT,
        BACKWARD_RIGHT,
        CLOCKWISE,
        COUNTERCLOCKWISE,
        NULL
    };

    public enum Speed{
        FAST,
        SLOW
    };
}
