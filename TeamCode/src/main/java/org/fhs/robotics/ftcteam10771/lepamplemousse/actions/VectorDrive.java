package org.fhs.robotics.ftcteam10771.lepamplemousse.actions;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    Config.ParsedData settings;
    boolean vectorDriveActive;
    boolean blueTeam;
    boolean relativeDrive;
    boolean joystickControl;

    Runnable driveRunnable = new Runnable() {
        @Override
        public void run() {

            VectorR currentVectorR;

            while (!Thread.interrupted()) {

                //chooses which vectorR to set currentVectorR to (determins whether the robot is controlled by autonomous or joystick)
                if(joystickControl){
                    currentVectorR = vectorR;
                }else{
                    currentVectorR = null; //autonomous vector R
                }

                //sets values from the vectorR needed for movement
                float joystickTheta = currentVectorR.getTheta();
                float absoluteTheta = 0;
                float robotTheta;
                float joystickRadius = currentVectorR.getRadius();
                float rotationalPower = currentVectorR.getRad();
                float robotRotation = robot.getVectorR().getRad();

                if(relativeDrive){ //if the robot drives relative to the field
                    if(blueTeam){ //if our team is on blue
                        if(joystickTheta > ((Math.PI))/2){ //keeps the theta value positive while rotating the polar coordinates pi/2 ccw
                            absoluteTheta = (float) (joystickTheta - (Math.PI)/2);
                        }else{
                            absoluteTheta = (float) (joystickTheta + (3*(Math.PI))/2);
                        }
                    }else{
                        absoluteTheta = joystickTheta; //the field coordinate does not need to be adjusted
                    }

                    if((absoluteTheta+robotRotation)<(2*Math.PI)){ //sets the robotTheta to the direction the robot has to move while keeping the theta value positive
                        robotTheta = (float) (absoluteTheta + robotRotation);
                    }else{
                        robotTheta = (float) (absoluteTheta + robotRotation - 2*(Math.PI));
                    }
                }else{
                    robotTheta = joystickTheta; //the direction of the joystick is the direction of motion
                }

                //calculates the shaft magnitude (AC shaft has diagonal motors "A" and "C")
                float ACShaftPower = (float) ((Math.sin(absoluteTheta - (Math.PI/ 4))) * joystickRadius);
                float BDShaftPower = (float) ((Math.cos(absoluteTheta - (Math.PI / 4))) * joystickRadius);

                //calculates the motor powers
                frMotor.setPower((-robotTheta) + ((ACShaftPower) * (1.0 - (Math.abs(rotationalPower)))));
                flMotor.setPower((robotTheta) + ((BDShaftPower) * (1.0 - (Math.abs(rotationalPower)))));
                blMotor.setPower((robotTheta) + ((ACShaftPower) * (1.0 - (Math.abs(rotationalPower)))));
                brMotor.setPower((-robotTheta) + ((BDShaftPower) * (1.0 - (Math.abs(rotationalPower)))));

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
        this.settings = settings;
        this.vectorDriveActive = false;

        this.blueTeam = settings.getBool("blue_team");
        this.relativeDrive = settings.getBool("relative_drive");

        DcMotor.RunMode runMode = DcMotor.RunMode.RUN_USING_ENCODER;
        frMotor.setMode(runMode);
        flMotor.setMode(runMode);
        brMotor.setMode(runMode);
        blMotor.setMode(runMode);

    }

    /**
     * Starts driveThread and changes vectorDriveActive to true
     */
    public void initiateVelocity(){
        vectorDriveActive = true;
        driveThread.start();
    }

    /**
     * Uses driveThread to move robot to position
     */
    public void initiatePosition(){
        vectorDriveActive = true;
        driveThread.start();

    }

    /**
     * Stops driveThread and changes vectorDriveActive to false
     * Resets relativeDrive to true
     */
    public void endDriveThread(){
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

    /**
     * Sets runMode of all motors to input
     *
     * @param runMode desired Runmode(DcMotor.RunMode)
     */
    public void setRunMode(DcMotor.RunMode runMode){
        frMotor.setMode(runMode);
        flMotor.setMode(runMode);
        brMotor.setMode(runMode);
        blMotor.setMode(runMode);
    }

    /**
     * sets whether the robot will be controlled by input or autonomous when driveThread is launched
     *
     * @param setJoystickControl (true: robot will be controlled by joystick/ false: robot will be controlled by autonomous)
     */
    public void setJoystickControl(boolean setJoystickControl){
        joystickControl = setJoystickControl;
    }
}
