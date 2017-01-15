package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Freedom Robotics on 11/19/2016.
 */

@TeleOp(name = "Test the Motor Mapping")
public class MotorTest extends LinearOpMode {

    private DcMotor motorA;
    private DcMotor motorB;
    private DcMotor motorC;
    private DcMotor motorD;

    @Override
    public void runOpMode() throws InterruptedException{

        motorA = hardwareMap.dcMotor.get("motorFrontRight");
        motorB = hardwareMap.dcMotor.get("motorFrontLeft");
        motorC = hardwareMap.dcMotor.get("motorBackLeft");
        motorD = hardwareMap.dcMotor.get("motorBackRight"); //initializes motor

        waitForStart();

        while(opModeIsActive()) {
            motorA.setPower(0.0);
            motorB.setPower(0.0);
            motorC.setPower(0.0);
            motorD.setPower(0.0);
            if (gamepad1.b)
                motorA.setPower(.125);
            if (gamepad1.y)
                motorB.setPower(.125);
            if (gamepad1.x)
                motorC.setPower(.125);
            if (gamepad1.a)
                motorD.setPower(.125);
        }
    }
}
