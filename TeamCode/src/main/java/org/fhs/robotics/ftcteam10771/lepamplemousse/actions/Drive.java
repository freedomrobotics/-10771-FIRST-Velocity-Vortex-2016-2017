package org.fhs.robotics.ftcteam10771.lepamplemousse.actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config.ParsedData;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.entities.Robot;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.vector.VectorR;

/**
 * Created by Adam Li on 10/27/2016.
 * Class to manage how the robot drives (smart control schemes, drivetrain types, etc.)
 */
public class Drive {
    VectorR vectorR;
    Robot robot;
    DcMotor frMotor;
    DcMotor flMotor;
    DcMotor brMotor;
    DcMotor blMotor;
    ParsedData settings;

    public enum MOTORS {
        FRMOTOR;
    }

    public Drive(VectorR vectorR, Robot robot, DcMotor frMotor,
                 DcMotor flMotor, DcMotor brMotor, DcMotor blMotor,
                 ParsedData settings){
        this.vectorR = vectorR;
        this.robot = robot;
        this.frMotor = frMotor;
        this.flMotor = flMotor;
        this.brMotor = brMotor;
        this.blMotor = blMotor;

        settings.getFloat("wheel");
        settings.subData("lr_motor").getBool("reversed");
    }

    public void init(){

    }

    public void begin(){

    }

    public void setDirection(MOTORS motor, DcMotorSimple.Direction direction){
        if(motor == MOTORS.FRMOTOR){
            frMotor.setDirection(direction);
        }
    }
}
