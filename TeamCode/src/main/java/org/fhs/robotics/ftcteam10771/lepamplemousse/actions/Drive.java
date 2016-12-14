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
public class Drive {
    VectorR vectorR;
    Robot robot;
    DcMotor frMotor;
    DcMotor flMotor;
    DcMotor brMotor;
    DcMotor blMotor;
    Config.ParsedData settings;

    public enum MOTORS {
        FRMOTOR;
    }

    public Drive(VectorR vectorR, Robot robot, DcMotor frMotor,
                 DcMotor flMotor, DcMotor brMotor, DcMotor blMotor,
                 Config.ParsedData settings){

        this.vectorR = vectorR;
        this.robot = robot;
        this.frMotor = frMotor;
        this.flMotor = flMotor;
        this.brMotor = brMotor;
        this.blMotor = blMotor;
        this.settings = settings;

    }

    public void init(){

    }

    public void begin(){

    }

    public void setDirection(float theta, float translationalSpeed, float angularVelocity){

        double adjustedTheta;
        if((theta + vectorR.getRad()) > (2 * Math.PI)){
            adjustedTheta = theta + vectorR.getRad() - (2 * Math.PI);
        }else{
            adjustedTheta = theta + vectorR.getRad();
        }

        double ACShaftPower = ((Math.sin(adjustedTheta-(Math.PI/4)))*translationalSpeed);
        double BDShaftPower = ((Math.cos(adjustedTheta-(Math.PI/4)))*translationalSpeed);

        frMotor.setPower((-translationalSpeed) + ((ACShaftPower)*(1.0 - (Math.abs(angularVelocity)))));
        flMotor.setPower((translationalSpeed) + ((BDShaftPower)*(1.0 - (Math.abs(angularVelocity)))));
        blMotor.setPower((translationalSpeed) + ((ACShaftPower)*(1.0 - (Math.abs(angularVelocity)))));
        brMotor.setPower((-translationalSpeed) + ((BDShaftPower)*(1.0 - (Math.abs(angularVelocity)))));

    }
}
