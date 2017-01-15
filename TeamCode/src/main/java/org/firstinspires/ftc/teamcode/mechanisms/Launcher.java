package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Class that handles the ball launcher mechanism motors and servos
 * Created by joelv on 1/13/2017.
 */
public class Launcher {

    //The two motors of the launcher mechanism
    private DcMotor intake = null;
    private DcMotor launch= null;

    //Power to set the launch motor at
    private double launchPower = 0.5;
    private final double intakePower = 1.0;
    private final double launchIncrement = 0.01;
    private final double MAX_REV_PER_SEC = 152.0 / 60.0;

    /*
        Default constructor
     */
    public Launcher(){

    }

    /*
        Constructor that takes one motor to assign to intake
     */
    public Launcher(DcMotor motor){
        intake = motor;
        this.intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /*
        Constructor that assigns the two motors of the mechanism
     */
    public Launcher(DcMotor intake, DcMotor launch){
        this.intake = intake;
        this.launch = launch;
        // todo determine correct directions of each motor
        this.intake.setDirection(DcMotorSimple.Direction.FORWARD);
        this.launch.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Launch mechanism is turned on with button press
     * or by plugging true in the parameter
     * @param button to press or the flag to toggle launch motor
     */
    public void launch(boolean button){
        if (button){
            launch.setPower(launchPower);
        }
        else launch.setPower(0.0);
    }

    /**
     * Intake mechanism is turned on with button press
     * @param button
     */
    public void intake(boolean button){
        if (button){
            intake.setPower(intakePower);
        }
        else intake.setPower(0.0);
    }

    /**
     * Launch mechanism power dependent
     * on analog input
     * @param joystick the input from analog stick
     */
    public void launch(float joystick){
        launch.setPower((double)joystick);
    }

    /**
     * Intake mechanism method solely
     * dependent on analog input
     * @param joystick
     */
    public void intake(float joystick){
        intake.setPower((double)joystick);
    }

    /**
     * Set the launch power manually through code
     * @param launchPower the power between -1.0 to 1.0
     */
    public void setLaunchPower(double launchPower){
        if (launchPower >= -1.0 && launchPower <= 1.0){
            this.launchPower = launchPower;
        }
    }
    
    /**
     * The user can adjust the power with the analog stick
     * @param joystick the value from the joystick used
     * @param sensitivity the sensitivity from 1 to 10 of analog stick
     */
    public void adjustPower(float joystick, int sensitivity){
        int joystickSensitivity;
        if (sensitivity < 1){
            joystickSensitivity = 1;
        }
        else if (sensitivity > 10){
            joystickSensitivity = 10;
        }
        else joystickSensitivity = sensitivity;
        double increment = launchIncrement * (double)joystick * joystickSensitivity;
        if (Math.abs(launchPower+launchIncrement) < 1){
            launchPower =+ increment;
        }
    }

    /**
     * Adjust the Power of launcher with the joystick
     * with default sensitivity of 5 (1-10)
     * @param joystick the value gained from the joystick
     */
    public void adjustPower(float joystick){
        adjustPower(joystick, 5);
    }

    /**
     * Increases the power if user holds down button
     * @param button to press to increase power
     * @param incrementFactor the number to multiply increments
     */
    public void increasePower(boolean button, int incrementFactor){
        if (button) adjustPower(1, incrementFactor);
    }

    /**
     * Decreases the power if user holds down button
     * @param button to press to decrease the power
     * @param decrementFactor the number which to multiply decrement
     */
    public void decresePower(boolean button, int decrementFactor){
        if (button) adjustPower(-1, decrementFactor);
    }

    /**
     * Increase power with button hold
     * Default increment = 0.01
     * @param button to press to increase power
     */
    public void increasePower(boolean button){
        increasePower(button, 1);
    }

    /**
     * Decrease power with button hold
     * Default decrement = 0.01
     * @param button to press to decrease power
     */
    public void decreasePower(boolean button){
        decresePower(button, 1);
    }

    /**
     * Gets the launch power of the motors
     * @return the launch power
     */
    public double getLaunchPower(){
        return launchPower;
    }

    /**
     * convert the motor's power variable to motor velocity
     * TODO: This is wrong, so it needs fixing or deleting
     * @param gearRadius radius of the gear attached to launch motor
     * @return the object's velocity
     */
    public double  convertToVelocity(double gearRadius){
        return 2 * Math.PI * gearRadius * launchPower / MAX_REV_PER_SEC;
    }

    /**
     * Convert velocity to motor power
     * TODO: This conversion is wrong
     * @param objectVelocity the velocity of the motor launch
     * @param gearRadius radius of the wheel/gear attached to motor
     * @return the motor power between -1 to 1
     */
    public double convertToMotorPower(double objectVelocity, double gearRadius){
        double result = objectVelocity * MAX_REV_PER_SEC * 0.5 / Math.PI / gearRadius;
        return (result >= -1.0 && result <= 1.0) ? result : 0.0;
    }
}
