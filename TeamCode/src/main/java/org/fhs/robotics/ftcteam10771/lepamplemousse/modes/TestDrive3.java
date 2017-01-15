package org.fhs.robotics.ftcteam10771.lepamplemousse.modes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

/**
 * Created by Matthew on 11/14/2016.
 */

@TeleOp(name = "TestDrive3")
public class TestDrive3 extends LinearOpMode{

    //initializes motors in order of standard graph quadrants
    private DcMotor motorA;
    private DcMotor motorB;
    private DcMotor motorC;
    private DcMotor motorD;
    private DcMotor intakeMotor;
    private double intakePower = 0.0;
    private boolean intakeF = false, intakeB = false;
    private List<String> toggle = new LinkedList<>();

    @Override
    public void runOpMode() throws InterruptedException{

        motorA = hardwareMap.dcMotor.get("motorFrontRight"); //sets variable motorA to front right motor
        motorB = hardwareMap.dcMotor.get("motorFrontLeft"); //sets variable motorB to front left motor
        motorC = hardwareMap.dcMotor.get("motorBackLeft"); //sets variable motorC to back left motor
        motorD = hardwareMap.dcMotor.get("motorBackRight"); //sets variable motorD to back right motor

        intakeMotor = hardwareMap.dcMotor.get("motorIntake");

        motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

            if (intakePower < 0){
                intakePower = 0;
            } if (intakePower > 1){
                intakePower = 1;
            }
            intakePower += gamepad1.right_stick_y / 5000;

            double joystickTheta = Math.atan2((gamepad1.left_stick_y),(gamepad1.left_stick_x)); //declares the angle of joystick position in standard polar coordinates
            double joystickRadius = Math.sqrt((gamepad1.left_stick_x)*(gamepad1.left_stick_x)+(gamepad1.left_stick_y)*(gamepad1.left_stick_y)); //declares the magnitude of the radius of the joystick position

            double ACShaftPower = -((Math.sin(joystickTheta-(Math.PI/4)))*joystickRadius); //sets the power of the shaft containing motors A and C using the radius to scale the sin value of the joystickTheta
            double BDShaftPower = -((Math.cos(joystickTheta-(Math.PI/4)))*joystickRadius); //sets the power of the shaft containing motors B and D using the radius to scale the cos value of the joystickTheta

            // Halved rotationPower value to allow for simultaneous translation and rotation when fully depressed - Adam Li
            double rotationalPower = gamepad1.right_trigger - gamepad1.left_trigger; //sets the power of rotation by finding the difference between the left and right triggers
            double ACRotationalPower = (rotationalPower+ACShaftPower) == 0 ? 0 : (rotationalPower*Math.abs(rotationalPower))/(Math.abs(rotationalPower)+Math.abs(ACShaftPower));
            double BDRotationalPower = (rotationalPower+BDShaftPower) == 0 ? 0 : (rotationalPower*Math.abs(rotationalPower))/(Math.abs(rotationalPower)+Math.abs(BDShaftPower));

            //sets the motor power where the ratio of input from translational motion is dictated by the magnitude of the rotational motion
            /*
            We know the right side needs to be driven in reverse to rotate right, so motors A
             and D have negative rotationalPower values. We also know forward translation
             involves positive motor values for all when rotation is disregarded, so motors C
             and D have signage on shaft power changed to positive again.
             */

            double powerA = (-ACRotationalPower)+(ACShaftPower*(1.0-Math.abs(ACRotationalPower)));
            double powerB = (BDRotationalPower)+(BDShaftPower*(1.0-Math.abs(BDRotationalPower)));
            double powerC = (ACRotationalPower)+(ACShaftPower*(1.0-Math.abs(ACRotationalPower)));
            double powerD = (-BDRotationalPower)+(BDShaftPower*(1.0-Math.abs(BDRotationalPower)));
            motorA.setPower(Range.scale(powerA, -1, 1, -.778, .778));
            motorB.setPower(Range.scale(powerB, -1, 1, -.778, .778));
            motorC.setPower(Range.scale(powerC, -1, 1, -.778, .778));
            motorD.setPower(Range.scale(powerD, -1, 1, -.778, .778));

            telemetry.addData("Speed-FR", powerA);
            telemetry.addData("Speed-FL", powerB);
            telemetry.addData("Speed-BL", powerC);
            telemetry.addData("Speed-BR", powerD);

            telemetry.addData("IntakeSpeed", intakePower);

            if (gamepad1.b && !toggle.contains("intakeF")){
                intakeB = !intakeB;
                intakeF = false;
                toggle.add("intakeB");
            } if (!gamepad1.b && toggle.contains("intakeB")){
                toggle.remove("intakeB");
            }
            if (gamepad1.a && !toggle.contains("intakeF")){
                intakeF = !intakeF;
                intakeB = false;
                toggle.add("intakeF");
            } if (!gamepad1.a && toggle.contains("intakeF")){
                toggle.remove("intakeF");
            }

            if (intakeF){
                intakeMotor.setPower(Range.scale(intakePower, 0, 1, -.778, .778));
            } else if (intakeB){
                intakeMotor.setPower(-Range.scale(intakePower, 0, 1, -.778, .778));
            } else {
                intakeMotor.setPower(0);
            }

            telemetry.update();

            //idle();

        }
    }
}