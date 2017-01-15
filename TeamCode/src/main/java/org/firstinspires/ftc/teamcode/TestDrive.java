package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Matthew on 11/14/2016.
 */

@TeleOp(name = "TestDrive")
public class TestDrive extends LinearOpMode{

    //initializes motors in order of standard graph quadrants
    private DcMotor motorA;
    private DcMotor motorB;
    private DcMotor motorC;
    private DcMotor motorD;

    @Override
    public void runOpMode() throws InterruptedException{

        motorA = hardwareMap.dcMotor.get("motorFrontRight"); //sets variable motorA to front right motor
        motorB = hardwareMap.dcMotor.get("motorFrontLeft"); //sets variable motorB to front left motor
        motorC = hardwareMap.dcMotor.get("motorBackLeft"); //sets variable motorC to back left motor
        motorD = hardwareMap.dcMotor.get("motorBackRight"); //sets variable motorD to back right motor

        /*
        In our mecanum setup, the two front wheels are chain driven and the two rear wheels are
         direct gear driven. What this means is that the two front wheels are spinning the same
         direction as the motor and the two rear wheels are spinning the opposite. Since the
         motors on the right need to be spinning clockwise to move forward, the motors on the
         right need to be reversed. Thus the front right and the back left motors need to be
         reversed. That is, motors A and C. - Adam Li
         */
        motorA.setDirection(DcMotor.Direction.REVERSE);
        motorB.setDirection(DcMotor.Direction.FORWARD);
        motorC.setDirection(DcMotor.Direction.REVERSE);
        motorD.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        while(opModeIsActive()){
            
            double joystickTheta = Math.atan2((gamepad1.left_stick_y),(gamepad1.left_stick_x)); //declares the angle of joystick position in standard polar coordinates
            double joystickRadius = Math.sqrt((gamepad1.left_stick_x)*(gamepad1.left_stick_x)+(gamepad1.left_stick_y)*(gamepad1.left_stick_y)); //declares the magnitude of the radius of the joystick position

            double ACShaftPower = -((Math.sin(joystickTheta-(Math.PI/4)))*joystickRadius); //sets the power of the shaft containing motors A and C using the radius to scale the sin value of the joystickTheta
            double BDShaftPower = -((Math.cos(joystickTheta-(Math.PI/4)))*joystickRadius); //sets the power of the shaft containing motors B and D using the radius to scale the cos value of the joystickTheta
            // Halved rotationPower value to allow for simultaneous translation and rotation when fully depressed - Adam Li
            double rotationalPower = (gamepad1.right_trigger - gamepad1.left_trigger) / 2; //sets the power of rotation by finding the difference between the left and right triggers

            //sets the motor power where the ratio of input from translational motion is dictated by the magnitude of the rotational motion
            /*
            We know the right side needs to be driven in reverse to rotate right, so motors A
             and D have negative rotationalPower values. We also know forward translation
             involves positive motor values for all when rotation is disregarded, so motors C
             and D have signage on shaft power changed to positive again.
             */
            motorA.setPower((-rotationalPower) + ((ACShaftPower)*(1.0 - (Math.abs(rotationalPower)))));
            motorB.setPower((rotationalPower) + ((BDShaftPower)*(1.0 - (Math.abs(rotationalPower)))));
            motorC.setPower((rotationalPower) + ((ACShaftPower)*(1.0 - (Math.abs(rotationalPower)))));
            motorD.setPower((-rotationalPower) + ((BDShaftPower)*(1.0 - (Math.abs(rotationalPower)))));

            idle();

        }
    }
}