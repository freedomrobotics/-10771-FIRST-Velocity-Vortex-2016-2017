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

    private DcMotor motorA;
    private DcMotor motorB;
    private DcMotor motorC;
    private DcMotor motorD;

    @Override
    public void runOpMode() throws InterruptedException{

        motorA = hardwareMap.dcMotor.get("motorFrontRight");
        motorB = hardwareMap.dcMotor.get("motorFrontLeft");
        motorC = hardwareMap.dcMotor.get("motorBackLeft");
        motorD = hardwareMap.dcMotor.get("motorBackRight");

        motorB.setDirection(DcMotor.Direction.REVERSE);
        motorC.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){

            //double leftYAxis = -gamepad1.left_stick_y;
            //double leftXAxis = gamepad1.left_stick_x;

            double joystickTheta = Math.atan2((-gamepad1.left_stick_y),(gamepad1.left_stick_x));
            double joystickRadius = Math.sqrt((gamepad1.left_stick_x)*(gamepad1.left_stick_x)+(-gamepad1.left_stick_y)*(-gamepad1.left_stick_y));

            if((gamepad1.left_stick_x==0)&&(gamepad1.left_stick_y==0)){
                if (gamepad1.left_trigger != 0) {
                    motorA.setPower(gamepad1.left_trigger);
                    motorB.setPower(-(gamepad1.left_trigger));
                    motorC.setPower(-(gamepad1.left_trigger));
                    motorD.setPower(gamepad1.left_trigger);
                } else {
                    motorA.setPower(-(gamepad1.right_trigger));
                    motorB.setPower(gamepad1.right_trigger);
                    motorC.setPower(gamepad1.right_trigger);
                    motorD.setPower(-(gamepad1.right_trigger));
                }
            }else{
                motorA.setPower((Math.sin(Math.sin(joystickTheta-(Math.PI/2))))*joystickRadius);
                motorB.setPower((Math.cos(Math.sin(joystickTheta-(Math.PI/2))))*joystickRadius);
                motorC.setPower((Math.sin(Math.sin(joystickTheta-(Math.PI/2))))*joystickRadius);
                motorD.setPower((Math.cos(Math.sin(joystickTheta-(Math.PI/2))))*joystickRadius);

                /*
                motorA.setPower((-leftXAxis+leftYAxis)/2);
                motorD.setPower((leftXAxis+leftYAxis)/2);
                motorC.setPower((-leftXAxis+leftYAxis)/2);
                motorD.setPower((leftXAxis+leftYAxis)/2);
                */
            }

            idle();
        }
    }
}