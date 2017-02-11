package org.fhs.robotics.ftcteam10771.lepamplemousse.modes.final_op_modes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Components;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Controllers;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.components.Aliases;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.mechanisms.Catapult;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.Static;

import java.util.LinkedList;
import java.util.List;

/**
 * Created by Matthew on 11/14/2016.
 */

@TeleOp(name = "TestDrive3")
public class TestDrive3 extends LinearOpMode{

    //initializes motors in order of standard graph quadrants
    private DcMotor motorFR;
    private DcMotor motorFL;
    private DcMotor motorBL;
    private DcMotor motorBR;
    private DcMotor intakeMotor;
    private double intakePower = 0.0;
    private boolean intakeF = false, intakeB = false;
    private List<String> toggle = new LinkedList<>();
    private Config rawSettings;
    private Config.ParsedData settings;
    private Components components;
    private Controllers controls;
    private Catapult catapult;

    private static final String TAG = "TestDrive3Debug";
    private float bumperPos;
    private float initialX = 0.0f;
    private float initialY = 0.0f;

    private long lastTime;

    @Override
    public void runOpMode() throws InterruptedException{

        Config keymapping = new Config(Static.configPath, Static.configControlFileName + Static.configFileSufffix, telemetry, "keymapping");

        Log.d(TAG, "keymapping");

        // should catch other errors, but oh well
        // add data logging
        if (keymapping.read() == Config.State.DEFAULT_EXISTS) {
            keymapping.create(true);
            Log.d(TAG, "keymapping-read-fail-create");
            //read without creating file if it still fails.
            if (keymapping.read() == Config.State.DEFAULT_EXISTS)
                keymapping.read(true);
            Log.d(TAG, "keymapping-read-again");
        }

        Config components = new Config(Static.configPath, Static.configCompFileName + Static.configFileSufffix, telemetry, "components");
        Log.d(TAG, "components");
        if (components.read() == Config.State.DEFAULT_EXISTS) {
            components.create(true);
            Log.d(TAG, "components-read-fail-create");
            if (components.read() == Config.State.DEFAULT_EXISTS)
                components.read(true);
            Log.d(TAG, "components-read-again");
        }

        rawSettings = new Config(Static.configPath, Static.configVarFileName + Static.configFileSufffix, telemetry, "settings");
        Log.d(TAG, "settings");
        if (rawSettings.read() == Config.State.DEFAULT_EXISTS) {
            rawSettings.create(true);
            Log.d(TAG, "settings-read-fail-create");
            if (rawSettings.read() == Config.State.DEFAULT_EXISTS)
                rawSettings.read(true);
            Log.d(TAG, "settings-read-again");
        }

        settings = rawSettings.getParsedData();
        Log.d(TAG, "settings-parse");


        this.components = new Components(hardwareMap, telemetry, components);
        Log.d(TAG, "components-object");
        this.components.initialize();
        Log.d(TAG, "components-init");
        controls = new Controllers(gamepad1, gamepad2, keymapping);
        controls.initialize();
        Log.d(TAG, "controllers-init");

        //sets variables to motors
        motorFR = hardwareMap.dcMotor.get(settings.subData("drivetrain").subData("motor").subData("front_right").getString("map_name"));
        motorFL = hardwareMap.dcMotor.get(settings.subData("drivetrain").subData("motor").subData("front_left").getString("map_name"));
        motorBL = hardwareMap.dcMotor.get(settings.subData("drivetrain").subData("motor").subData("back_left").getString("map_name"));
        motorBR = hardwareMap.dcMotor.get(settings.subData("drivetrain").subData("motor").subData("back_right").getString("map_name"));

        Log.d(TAG, settings.subData("drivetrain").subData("motor").subData("front_right").getString("map_name"));
        Log.d(TAG, settings.subData("drivetrain").subData("motor").subData("front_left").getString("map_name"));
        Log.d(TAG, settings.subData("drivetrain").subData("motor").subData("back_left").getString("map_name"));
        Log.d(TAG, settings.subData("drivetrain").subData("motor").subData("back_right").getString("map_name"));

        Log.d(TAG, "motor setup");

        intakeMotor = hardwareMap.dcMotor.get("motorIntake");

        Log.d(TAG, "motor setup 2");

        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Log.d(TAG, "motor setup 3");

        /*
        In our mecanum setup, the two front wheels are chain driven and the two rear wheels are
         direct gear driven. What this means is that the two front wheels are spinning the same
         direction as the motor and the two rear wheels are spinning the opposite. Since the
         motors on the right need to be spinning clockwise to move forward, the motors on the
         right need to be reversed. Thus the front right and the back left motors need to be
         reversed. That is, motors A and C. - Adam Li
         */
        /*motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);*/

        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        if (settings.subData("drivetrain").subData("motor").subData("front_right").getBool("reversed")) motorFR.setDirection(DcMotor.Direction.REVERSE);
        if (settings.subData("drivetrain").subData("motor").subData("front_left").getBool("reversed")) motorFL.setDirection(DcMotor.Direction.REVERSE);
        if (settings.subData("drivetrain").subData("motor").subData("back_left").getBool("reversed")) motorBL.setDirection(DcMotor.Direction.REVERSE);
        if (settings.subData("drivetrain").subData("motor").subData("back_right").getBool("reversed")) motorBR.setDirection(DcMotor.Direction.REVERSE);

        Log.d(TAG, "motor setup 4");

        Config.ParsedData bumpers = settings.subData("bumper");

        float bumperRange = bumpers.getFloat("full_rotate");
        float bumperPreset = bumpers.getFloat("preset");
        float bumperVel = bumpers.getFloat("max_ang_vel");
        float bumperMax = bumpers.getFloat("max_rotate");

        Servo bumperLeft = Aliases.servoMap.get(bumpers.subData("left_servo").getString("map_name"));
        Servo bumperRight = Aliases.servoMap.get(bumpers.subData("right_servo").getString("map_name"));
        if (bumpers.subData("left_servo").getBool("reversed"))
            bumperLeft.setDirection(Servo.Direction.REVERSE);
        if (bumpers.subData("right_servo").getBool("reversed"))
            bumperRight.setDirection(Servo.Direction.REVERSE);

        float power = settings.subData("drivetrain").getFloat("power");
        catapult = new Catapult(hardwareMap.dcMotor.get(settings.subData("catapult").getString("map_name")), hardwareMap.opticalDistanceSensor.get("ods"), controls, settings);

        lastTime = System.currentTimeMillis();
        waitForStart();
        catapult.catapultThread.start();
        while(opModeIsActive()){
            long changeTime = System.currentTimeMillis() - lastTime;
            lastTime += changeTime;
            if (intakePower < 0){
                intakePower = 0;
            } if (intakePower > 1){
                intakePower = 1;
            }
            intakePower += gamepad1.right_stick_y / settings.getInt("intake_divisor");

            double joystickTheta = Math.atan2((controls.getAnalog("drivetrain_y")),(controls.getAnalog("drivetrain_x"))); //declares the angle of joystick position in standard polar coordinates
            double joystickRadius = Math.sqrt((controls.getAnalog("drivetrain_x"))*(controls.getAnalog("drivetrain_x"))+(controls.getAnalog("drivetrain_y"))*(controls.getAnalog("drivetrain_y"))); //declares the magnitude of the radius of the joystick position

            double ACShaftPower = -((Math.sin(joystickTheta-(Math.PI/4)))*joystickRadius); //sets the power of the shaft containing motors A and C using the radius to scale the sin value of the joystickTheta
            double BDShaftPower = -((Math.cos(joystickTheta-(Math.PI/4)))*joystickRadius); //sets the power of the shaft containing motors B and D using the radius to scale the cos value of the joystickTheta

            // Halved rotationPower value to allow for simultaneous translation and rotation when fully depressed - Adam Li
            double rotationalPower = controls.getAnalog("drivetrain_rotate"); //sets the power of rotation by finding the difference between the left and right triggers
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
            motorFR.setPower(Range.scale(powerA, -1, 1, -power, power));
            motorFL.setPower(Range.scale(powerB, -1, 1, -power, power));
            motorBL.setPower(Range.scale(powerC, -1, 1, -power, power));
            motorBR.setPower(Range.scale(powerD, -1, 1, -power, power));

            telemetry.addData("Speed-FR", powerA);
            telemetry.addData("Speed-FL", powerB);
            telemetry.addData("Speed-BL", powerC);
            telemetry.addData("Speed-BR", powerD);
            telemetry.addData("x", controls.getAnalog("drivetrain_x"));
            telemetry.addData("y", controls.getAnalog("drivetrain_y"));
            telemetry.addData("rot", controls.getAnalog("drivetrain_rotate"));

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

            //servo


            bumperPos += controls.getAnalog("bumper_angle") * (bumperVel / bumperRange) * ((float) changeTime / 1000.0f);

            if (bumperPos > bumperMax / bumperRange){
                bumperPos = bumperMax / bumperRange;
            }
            if (bumperPos < 0) {
                bumperPos = 0;
            }
            if (controls.getDigital("bumper_preset"))
                bumperPos = bumperPreset / bumperRange;
            bumperLeft.setPosition(bumperPos + bumpers.subData("left_servo").getFloat("offset") / bumperRange);
            bumperRight.setPosition(bumperPos + bumpers.subData("right_servo").getFloat("offset") / bumperRange);

            telemetry.addData("X inches", getX());
            telemetry.addData("Y inches", getY());
            telemetry.update();

            //idle();

        }
        //fixme threads not allowed after loop
        catapult.catapultThread.interrupt();
        Aliases.clearAll();
    }

    private float getX(){
        float inch_per_pulse = 4f  * (float)Math.PI / settings.subData("encoder").getFloat("output_pulses");
        double motorAngle = Math.toRadians(settings.subData("drivetrain").getFloat("motor_angle"));
        float A = -motorFR.getCurrentPosition()*inch_per_pulse;
        float B = -motorFL.getCurrentPosition()*inch_per_pulse;
        float C = -motorBL.getCurrentPosition()*inch_per_pulse;
        float D = -motorBR.getCurrentPosition()*inch_per_pulse;
        float AC = ((A*(float)Math.cos(Math.PI-motorAngle)) + (C*(float)Math.cos(Math.PI-motorAngle)))/2.0f;
        float BD = ((B*(float)Math.cos(motorAngle)) + (D*(float)Math.cos(motorAngle)))/2.0f;
        return  ((AC + BD) / 2.0f) + initialX;
    }

    private float getY(){
        float inch_per_pulse = 4f  * (float)Math.PI / settings.subData("encoder").getFloat("output_pulses");
        double motorAngle = Math.toRadians(settings.subData("drivetrain").getFloat("motor_angle"));
        float A = -motorFR.getCurrentPosition()*inch_per_pulse;
        float B = -motorFL.getCurrentPosition()*inch_per_pulse;
        float C = -motorBL.getCurrentPosition()*inch_per_pulse;
        float D = -motorBR.getCurrentPosition()*inch_per_pulse;
        float AC = (A+C)/2.0f;
        float BD = (B+D)/2.0f;
        return  ((AC + BD) / 2.0f) + initialY;
    }

    private void refresh(){
        initialX = getX();
        initialY = getY();
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}