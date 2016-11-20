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

        //reverses left side motors to make forwards positive rotation for all motors.
        motorB.setDirection(DcMotor.Direction.REVERSE);
        motorC.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()){

            double joystickTheta = Math.atan2((gamepad1.left_stick_y),(gamepad1.left_stick_x)); //declares the angle of joystick position in standard polar coordinates
            double joystickRadius = Math.sqrt((gamepad1.left_stick_x)*(gamepad1.left_stick_x)+(gamepad1.left_stick_y)*(gamepad1.left_stick_y)); //declares the magnitude of the radius of the joystick position

            if((gamepad1.left_stick_x==0)&&(gamepad1.left_stick_y==0))/*if either triggers are pressed*/{

                if (gamepad1.left_trigger != 0)/*if the left trigger is pressed*/{
                    //CCW zero clarence rotation (speed determined my magnitude of left trigger)
                    motorA.setPower(-(gamepad1.right_trigger));
                    motorB.setPower(gamepad1.right_trigger);
                    motorC.setPower(gamepad1.right_trigger);
                    motorD.setPower(-(gamepad1.right_trigger));
                }else/*if only the right trigger is pressed*/{
                    //CW zero clarence rotation (speed determined my magnitude of right trigger)
                    motorA.setPower(gamepad1.left_trigger);
                    motorB.setPower(-(gamepad1.left_trigger));
                    motorC.setPower(-(gamepad1.left_trigger));
                    motorD.setPower(gamepad1.left_trigger);
                }

            }else/*if neither triggers are pressed*/{

                //translational movement determined by vector of left joystick
                motorA.setPower((Math.sin(joystickTheta-(Math.PI/4)))*joystickRadius); //same power as motor C but in opposite direction
                motorB.setPower((Math.cos(joystickTheta-(Math.PI/4)))*joystickRadius); //same power as motor D but in opposite direction
                motorC.setPower((Math.sin(joystickTheta-(Math.PI/4)))*joystickRadius);
                motorD.setPower((Math.cos(joystickTheta-(Math.PI/4)))*joystickRadius);


            }

            idle();

        }
    }
}