package org.fhs.robotics.ftcteam10771.lepamplemousse.actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.entities.Robot;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.vector.VectorR;
import java.lang.Math;

/**
 * Created by Adam Li on 10/27/2016.
 * Class to manage how the robot drives (smart control schemes, drivetrain types, etc.)
 */
public class VectorDrive {
    VectorR vectorR;
    Robot robot;
    DcMotor frMotor;
    DcMotor flMotor;
    DcMotor brMotor;
    DcMotor blMotor;
    Config.ParsedData driveSettings;
    boolean vectorDriveActive;
    boolean blueTeam;
    boolean relativeDrive;
    boolean joystickControl;

    Runnable driveRunnable = new Runnable() {
        @Override
        public void run() {

            while (!Thread.interrupted()) {

                //sets values from the vectorR needed for movement
                float joystickTheta = vectorR.getTheta();
                float absoluteTheta = 0;
                float robotTheta;
                float joystickRadius = vectorR.getRadius();
                float rotationalPower = vectorR.getRad();
                float robotRotation = robot.getVectorR().getRad();

                if(relativeDrive) { //if the robot drives relative to the field
                    robotTheta = joystickTheta; //the direction of the joystick is the direction of motion
                }else {
                    if (blueTeam) { //if our team is on blue
                        if (joystickTheta > ((Math.PI)) / 2) { //keeps the theta value positive while rotating the polar coordinates pi/2 ccw
                            absoluteTheta = (float) (joystickTheta - (Math.PI) / 2);
                        } else {
                            absoluteTheta = (float) (joystickTheta + (3 * (Math.PI)) / 2);
                        }
                    } else {
                        absoluteTheta = joystickTheta; //the field coordinate does not need to be adjusted
                    }

                    if ((absoluteTheta + robotRotation) < (2 * Math.PI)) { //sets the robotTheta to the direction the robot has to move while keeping the theta value positive
                        robotTheta = (float) (absoluteTheta + robotRotation);
                    } else {
                        robotTheta = (float) (absoluteTheta + robotRotation - 2 * (Math.PI));
                    }
                }

                //calculates the shaft magnitude (AC shaft has diagonal motors "A" and "C")
                float ACShaftPower = (float) ((Math.sin(robotTheta - (Math.PI/ 4))) * joystickRadius);
                float BDShaftPower = (float) ((Math.cos(robotTheta - (Math.PI / 4))) * joystickRadius);

                //calculates the motor powers
                frMotor.setPower((-ACShaftPower)+(ACShaftPower*(1.0-Math.abs(ACShaftPower))));
                flMotor.setPower((BDShaftPower)+(BDShaftPower*(1.0-Math.abs(BDShaftPower))));
                blMotor.setPower((ACShaftPower)+(ACShaftPower*(1.0-Math.abs(ACShaftPower))));
                brMotor.setPower((-BDShaftPower)+(BDShaftPower*(1.0-Math.abs(BDShaftPower))));

                //waits before refreshing motor powers
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    };

    Thread driveThread = new Thread(driveRunnable); //initializes driveThread based on driveRunnable

    /**
     * Constructor
     */
    public VectorDrive(VectorR vectorR, Robot robot, DcMotor frMotor,
                       DcMotor flMotor, DcMotor brMotor, DcMotor blMotor,
                       Config.ParsedData settings){

        this.vectorR = vectorR;
        this.robot = robot;
        this.frMotor = frMotor;
        this.flMotor = flMotor;
        this.brMotor = brMotor;
        this.blMotor = blMotor;
        this.driveSettings = settings.subData("drivetrain");
        this.vectorDriveActive = false;
        this.joystickControl = false;

        this.blueTeam = false;
        if (settings.getString("alliance") == "blue")
            this.blueTeam = true;
        this.relativeDrive = false;

        DcMotor.RunMode runMode = DcMotor.RunMode.RUN_USING_ENCODER;
        frMotor.setMode(runMode);
        flMotor.setMode(runMode);
        brMotor.setMode(runMode);
        blMotor.setMode(runMode);

        if(driveSettings.subData("motors").subData("front_right").getBool("reversed")){frMotor.setDirection(DcMotorSimple.Direction.REVERSE);}else{frMotor.setDirection(DcMotorSimple.Direction.FORWARD);}
        if(driveSettings.subData("motors").subData("front_left").getBool("reversed")){flMotor.setDirection(DcMotorSimple.Direction.REVERSE);}else{flMotor.setDirection(DcMotorSimple.Direction.FORWARD);}
        if(driveSettings.subData("motors").subData("back_left").getBool("reversed")){blMotor.setDirection(DcMotorSimple.Direction.REVERSE);}else{blMotor.setDirection(DcMotorSimple.Direction.FORWARD);}
        if(driveSettings.subData("motors").subData("back_right").getBool("reversed")){brMotor.setDirection(DcMotorSimple.Direction.REVERSE);}else{brMotor.setDirection(DcMotorSimple.Direction.FORWARD);}
    }

    /**
     * Starts driveThread and changes vectorDriveActive to true
     */
    public void startVelocity(){
        vectorDriveActive = true;
        driveThread.start();
    }

    /**
     * Uses driveThread to move robot to position
     */
    public void startPosition(){
        VectorR positionVectorR = new VectorR();
        this.vectorR = positionVectorR;

        vectorDriveActive = true;
        driveThread.start();

    }

    /**
     * Stops driveThread and changes vectorDriveActive to false
     * Resets relativeDrive to true
     */
    public void stop(){
        if(driveThread.isAlive()){driveThread.interrupt();}

        this.relativeDrive = true;
        vectorDriveActive = false;
    }

    /**
     * Sets what the robot drives relative to
     *
     * @param isRelative (true: robot drives relative to field/ false: robot drives relative to own orientation)
     */
    public void setRelative(boolean isRelative){
        relativeDrive = isRelative;
    }
}
