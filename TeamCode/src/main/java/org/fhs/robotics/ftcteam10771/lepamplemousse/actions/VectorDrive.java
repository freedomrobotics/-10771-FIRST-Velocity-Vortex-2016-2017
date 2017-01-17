package org.fhs.robotics.ftcteam10771.lepamplemousse.actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.entities.Robot;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.vector.VectorR;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.Math;

/**
 * Created by Adam Li on 10/27/2016.
 * Class to manage how the robot drives (smart control schemes, drivetrain types, etc.)
 */
public class VectorDrive {
    private final Telemetry telemetry;
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
                float joystickTheta = (float) Math.atan2(-vectorR.getY(), vectorR.getX());
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
                float ACShaftPower = (float) -((Math.sin(robotTheta - (Math.PI / 4))) * joystickRadius);
                float BDShaftPower = (float) -((Math.cos(robotTheta - (Math.PI / 4))) * joystickRadius);

                //sets the motor power where the ratio of input from translational motion is dictated by the magnitude of the rotational motion
                double ACRotationalPower = (rotationalPower+ACShaftPower) == 0 ? 0 : (rotationalPower*Math.abs(rotationalPower))/(Math.abs(rotationalPower)+Math.abs(ACShaftPower));
                double BDRotationalPower = (rotationalPower+BDShaftPower) == 0 ? 0 : (rotationalPower*Math.abs(rotationalPower))/(Math.abs(rotationalPower)+Math.abs(BDShaftPower));

                double fr = (-ACRotationalPower)+(ACShaftPower*(1.0-Math.abs(ACRotationalPower)));
                double fl = (BDRotationalPower)+(BDShaftPower*(1.0-Math.abs(BDRotationalPower)));
                double bl = (ACRotationalPower)+(ACShaftPower*(1.0-Math.abs(ACRotationalPower)));
                double br = (-BDRotationalPower)+(BDShaftPower*(1.0-Math.abs(BDRotationalPower)));

                //calculates the motor powers
                frMotor.setPower(Range.scale(fr, -1, 1, -.778, .778));
                flMotor.setPower(Range.scale(fl, -1, 1, -.778, .778));
                blMotor.setPower(Range.scale(bl, -1, 1, -.778, .778));
                brMotor.setPower(Range.scale(br, -1, 1, -.778, .778));

                /* spams the console
                telemetry.addData("Speed-FR", fr);
                telemetry.addData("Speed-FL", fl);
                telemetry.addData("Speed-BL", bl);
                telemetry.addData("Speed-BR", br);
                */
            }
        }
    };

    Thread driveThread = new Thread(driveRunnable); //initializes driveThread based on driveRunnable

    /**
     * Constructor
     */
    public VectorDrive(VectorR vectorR, Robot robot, DcMotor frMotor,
                       DcMotor flMotor, DcMotor blMotor, DcMotor brMotor,
                       Config.ParsedData settings, Telemetry telemetry){

        this.vectorR = vectorR;
        this.robot = robot;
        this.frMotor = frMotor;
        this.flMotor = flMotor;
        this.brMotor = brMotor;
        this.blMotor = blMotor;
        this.driveSettings = settings.subData("drivetrain");
        this.vectorDriveActive = false;
        this.joystickControl = false;
        this.telemetry = telemetry;

        this.blueTeam = false;
        if (settings.getString("alliance") == "blue")
            this.blueTeam = true;
        this.relativeDrive = false;

        DcMotor.RunMode runMode = DcMotor.RunMode.RUN_USING_ENCODER;
        frMotor.setMode(runMode);
        flMotor.setMode(runMode);
        brMotor.setMode(runMode);
        blMotor.setMode(runMode);

        if(driveSettings.subData("motor").subData("front_right").getBool("reversed")){frMotor.setDirection(DcMotor.Direction.REVERSE);}else{frMotor.setDirection(DcMotor.Direction.FORWARD);}
        if(driveSettings.subData("motor").subData("front_left").getBool("reversed")){flMotor.setDirection(DcMotor.Direction.REVERSE);}else{flMotor.setDirection(DcMotor.Direction.FORWARD);}
        if(driveSettings.subData("motor").subData("back_left").getBool("reversed")){blMotor.setDirection(DcMotor.Direction.REVERSE);}else{blMotor.setDirection(DcMotor.Direction.FORWARD);}
        if(driveSettings.subData("motor").subData("back_right").getBool("reversed")){brMotor.setDirection(DcMotor.Direction.REVERSE);}else{brMotor.setDirection(DcMotor.Direction.FORWARD);}
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
        //shouldn't be a point to this
        VectorR positionVectorR = new VectorR();
        this.vectorR = positionVectorR;

        //this flag should be enough to announce that a math change is needed. Robot's current
        // position can be gained from getVectorR and the vectorR provided is the aim position.
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
