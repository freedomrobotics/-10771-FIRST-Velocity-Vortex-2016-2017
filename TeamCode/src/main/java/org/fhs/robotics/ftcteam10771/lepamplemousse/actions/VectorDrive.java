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

    Runnable driveRunnable = new Runnable() {
        @Override
        public void run() {

        while (!Thread.interrupted()) {
            float joystickTheta = vectorR.getTheta();
            float absoluteTheta;
            float fieldTheta;
            float joystickRadius = vectorR.getRadius();
            float rotationalPower = vectorR.getRad();
            float robotRotation = robot.getVectorR().getRad();

            if(blueTeam){
                if(joystickTheta <= (3*(Math.PI))/2){
                    absoluteTheta = (float) (joystickTheta + (Math.PI)/2);
                }else{
                    absoluteTheta = (float) (joystickTheta - (3*(Math.PI))/2);
                }
            }else{
                absoluteTheta = joystickTheta;
            }

            if(relativeDrive){
                if(absoluteTheta <= robotRotation){
                    fieldTheta = (float) (absoluteTheta + robotRotation);
                }else{
                    fieldTheta = (float) (absoluteTheta + robotRotation - 2*(Math.PI));
                }
            }else{
                fieldTheta = absoluteTheta;
            }

            float ACShaftPower = (float) ((Math.sin(absoluteTheta - (Math.PI/ 4))) * joystickRadius);
            float BDShaftPower = (float) ((Math.cos(absoluteTheta - (Math.PI / 4))) * joystickRadius);

            frMotor.setPower((-fieldTheta) + ((ACShaftPower) * (1.0 - (Math.abs(rotationalPower)))));
            flMotor.setPower((fieldTheta) + ((BDShaftPower) * (1.0 - (Math.abs(rotationalPower)))));
            blMotor.setPower((fieldTheta) + ((ACShaftPower) * (1.0 - (Math.abs(rotationalPower)))));
            brMotor.setPower((-fieldTheta) + ((BDShaftPower) * (1.0 - (Math.abs(rotationalPower)))));

            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        }
    };

    Thread driveThread = new Thread(driveRunnable);

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
     * This creates the drive thread for velocity drive.
     */
    public void initiateVelocity(){
        vectorDriveActive = true;
        driveThread.start();
    }

    public void initiatePosition(){
        vectorDriveActive = true;
        driveThread.start();
    }

    public void initiateAutonomouse(){
        vectorDriveActive = true;
    }

    public void endDriveThread(){
        if(driveThread.isAlive()){driveThread.interrupt();}

        this.relativeDrive = true;
        vectorDriveActive = false;
    }

    public void setRelative(boolean isRelative){
        relativeDrive = isRelative;
    }

    public void setRunMode(DcMotor.RunMode runMode){
        frMotor.setMode(runMode);
        flMotor.setMode(runMode);
        brMotor.setMode(runMode);
        blMotor.setMode(runMode);
    }
}
