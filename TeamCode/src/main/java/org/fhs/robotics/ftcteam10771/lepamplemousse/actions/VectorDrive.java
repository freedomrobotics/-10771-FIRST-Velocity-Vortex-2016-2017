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

    Runnable driveRunnable = new Runnable() {
        @Override
        public void run() {

            while (!Thread.interrupted()) {
                float joystickTheta = vectorR.getTheta();
                float joystickRadius = vectorR.getRadius();
                float rotationalPower = vectorR.getRad();

                float ACShaftPower = (float) ((Math.sin(joystickTheta - (Math.PI/ 4))) * joystickRadius);
                float BDShaftPower = (float) ((Math.cos(joystickTheta - (Math.PI / 4))) * joystickRadius);

                frMotor.setPower((-joystickRadius) + ((ACShaftPower) * (1.0 - (Math.abs(rotationalPower)))));
                flMotor.setPower((joystickRadius) + ((BDShaftPower) * (1.0 - (Math.abs(rotationalPower)))));
                blMotor.setPower((joystickRadius) + ((ACShaftPower) * (1.0 - (Math.abs(rotationalPower)))));
                brMotor.setPower((-joystickRadius) + ((BDShaftPower) * (1.0 - (Math.abs(rotationalPower)))));
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

    }

    public void startDriveThread(){
        driveThread.start();
        vectorDriveActive = true;
    }

    public void endDriveThread(){
        driveThread.interrupt();
        vectorDriveActive = false;
    }
}
